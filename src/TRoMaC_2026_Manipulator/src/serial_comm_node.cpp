// serial_comm_node — 精简版
// 串口通信已移交 TRoMaCHardwareInterface (ros2_control 插件)
// 本节点仅负责：
//   1. 订阅 /serial_recv（hardware interface 发布的摇杆数据）驱动 MoveIt Servo
//   2. 订阅 /joint_states 更新 RobotState 供 Jacobian 退化使用
//   3. Homing（回零）功能

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <Eigen/Dense>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

static constexpr char START_SERVO_SERVICE[] = "/servo_node/start_servo";

// 控制模式枚举
enum class ControlMode : int { CARTESIAN = 0, JOINT_GROUP_1 = 1, JOINT_GROUP_2 = 2, HOMING = 3 };

// 前三轴/后三轴关节名
static const std::vector<std::string> JOINT_GROUP_1_NAMES = {
    "1-joint", "2-joint", "3-joint"};
static const std::vector<std::string> JOINT_GROUP_2_NAMES = {
    "4-joint", "5-joint", "6-joint"};

// 阻尼伪逆求解：J^+ = V diag(s_i/(s_i²+λ²)) U^T
static Eigen::VectorXd dampedPinvSolve(const Eigen::MatrixXd& J,
                                        const Eigen::VectorXd& v,
                                        double lambda = 0.05)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto& S = svd.singularValues();
  Eigen::VectorXd S_inv(S.size());
  for (Eigen::Index i = 0; i < S.size(); ++i)
    S_inv[i] = S[i] / (S[i] * S[i] + lambda * lambda);
  return svd.matrixV() * S_inv.asDiagonal() * svd.matrixU().transpose() * v;
}

// 死区处理
static double applyJoyAxis(int16_t raw, int16_t deadzone, int16_t max_raw, double max_output)
{
  if (std::abs(raw) <= deadzone) return 0.0;
  const double sign      = (raw > 0) ? 1.0 : -1.0;
  const double magnitude = static_cast<double>(std::abs(raw) - deadzone) /
                           static_cast<double>(max_raw - deadzone);
  return sign * std::min(magnitude, 1.0) * max_output;
}

class SerialCommNode : public rclcpp::Node
{
public:
  SerialCommNode() : Node("serial_comm_node")
  {
    // 摇杆相关参数
    declare_parameter("enable_servo_control", true);
    declare_parameter("planning_frame", std::string("base_link"));
    declare_parameter("joy_deadzone_xyz", 10);
    declare_parameter("joy_deadzone_angular", 1);
    declare_parameter("joy_max_linear", 1.0);
    declare_parameter("joy_max_angular", 1.0);
    declare_parameter("decouple_ee_max", 1.0);
    declare_parameter("send_log", true);

    declare_parameter("joint_names", std::vector<std::string>{
        "1-joint", "2-joint", "3-joint",
        "4-joint", "5-joint", "6-joint"});

    planning_frame_       = get_parameter("planning_frame").as_string();
    joy_deadzone_xyz_     = static_cast<int16_t>(get_parameter("joy_deadzone_xyz").as_int());
    joy_deadzone_angular_ = static_cast<int16_t>(get_parameter("joy_deadzone_angular").as_int());
    joy_max_linear_       = get_parameter("joy_max_linear").as_double();
    joy_max_angular_      = get_parameter("joy_max_angular").as_double();
    decouple_ee_max_      = get_parameter("decouple_ee_max").as_double();
    send_log_             = get_parameter("send_log").as_bool();

    // 订阅 serial_recv topic（由 TRoMaCHardwareInterface 发布）
    recv_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "serial_recv", 10,
        std::bind(&SerialCommNode::recvCallback, this, std::placeholders::_1));

    // servo控制
    if (get_parameter("enable_servo_control").as_bool())
    {
      enable_servo_control_ = true;

      servo_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
          "/servo_node/delta_twist_cmds", rclcpp::SystemDefaultsQoS());

      joint_jog_pub_ = create_publisher<control_msgs::msg::JointJog>(
          "/servo_node/delta_joint_cmds", rclcpp::SystemDefaultsQoS());

      std::thread([this]() { this->callStartServo(); }).detach();

      RCLCPP_INFO(get_logger(),
                  "Servo 控制已启用 (linear_max=%.2f  angular_max=%.2f)",
                  joy_max_linear_, joy_max_angular_);
    }

    // 订阅 /joint_states 用于 Jacobian 计算
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&SerialCommNode::jointStateCallback, this, std::placeholders::_1));
  }

  // 初始化 Jacobian 退化功能（需在 make_shared 后调用以获取 robot_description 参数）
  void initJacobianFallback()
  {
    if (!enable_servo_control_) return;

    declare_parameter("robot_description", std::string(""));
    declare_parameter("robot_description_semantic", std::string(""));
    std::string urdf_xml = get_parameter("robot_description").as_string();
    std::string srdf_xml = get_parameter("robot_description_semantic").as_string();
    if (urdf_xml.empty() || srdf_xml.empty())
    {
      RCLCPP_WARN(get_logger(),
                  "robot_description / robot_description_semantic 参数缺失，"
                  "关节限位 Jacobian 退化功能不可用");
      return;
    }

    auto urdf = urdf::parseURDF(urdf_xml);
    if (!urdf)
    {
      RCLCPP_ERROR(get_logger(), "URDF 解析失败");
      return;
    }
    auto srdf = std::make_shared<srdf::Model>();
    if (!srdf->initString(*urdf, srdf_xml))
    {
      RCLCPP_ERROR(get_logger(), "SRDF 解析失败");
      return;
    }

    robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();
    jmg_ = robot_model_->getJointModelGroup("manipulator");
    if (!jmg_)
    {
      RCLCPP_ERROR(get_logger(), "未找到 'manipulator' 规划组");
      return;
    }

    auto joint_names = get_parameter("joint_names").as_string_array();
    for (int i = 0; i < 6; ++i)
    {
      const auto* jm = robot_model_->getJointModel(joint_names[i]);
      if (!jm) continue;
      const auto& bounds = jm->getVariableBounds();
      if (!bounds.empty() && bounds[0].position_bounded_)
      {
        joint_limits_[i] = {true, bounds[0].min_position_, bounds[0].max_position_};
        RCLCPP_INFO(get_logger(), "关节 %s 限位: [%.4f, %.4f] rad",
                    joint_names[i].c_str(), bounds[0].min_position_, bounds[0].max_position_);
      }
      else
      {
        joint_limits_[i] = {false, 0.0, 0.0};
        RCLCPP_INFO(get_logger(), "关节 %s: 连续旋转（无限位）", joint_names[i].c_str());
      }
    }

    jacobian_ready_ = true;

    jacobian_link_ = robot_model_->getLinkModel("4_Link");
    if (!jacobian_link_)
      RCLCPP_WARN(get_logger(), "未找到 4_Link，Jacobian 将使用默认末端 link");

    RCLCPP_INFO(get_logger(),
                "Jacobian 已初始化 (margin=%.1f°, ref_link=%s)",
                LIMIT_MARGIN_RAD * 180.0 / M_PI,
                jacobian_link_ ? jacobian_link_->getName().c_str() : "6Link");
  }

private:
  // ---- serial_recv topic 回调：处理摇杆数据并驱动 Servo ----
  void recvCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // 数据格式: [x, y, z, pitch, roll, button, real_j1~j6]
    if (msg->data.size() < 6) 
    {
      //RCLCPP_WARN(get_logger(), "没有下位机传值！检查遥控器是否连接正常！！");
      return;
    }

    int16_t arm_x     = static_cast<int16_t>(msg->data[0]);
    int16_t arm_y     = static_cast<int16_t>(msg->data[1]);
    int16_t arm_z     = static_cast<int16_t>(msg->data[2]);
    int16_t arm_pitch = static_cast<int16_t>(msg->data[3]);
    int16_t arm_roll  = static_cast<int16_t>(msg->data[4]);
    uint8_t button    = static_cast<uint8_t>(msg->data[5]);

    // Btn=17 上升沿 → 延迟 800ms 后触发 JTC 重启同步。
    // hw_interface 在 Btn=17 那一刻已抓拍 snapshot 并启动 1.2s force-sync 窗口
    // （TX 恒等于 snapshot），800ms 后 MCU 已稳在 snapshot 上，此时 resync 让 JTC
    // 内部 last_commanded 锁定到 hw_positions_ ≈ snapshot；force-sync 1.2s 结束
    // 后 JTC 接管，cmd 与 force-sync 输出一致，无阶跃。
    if (button == 17 && last_button_ != 17)
    {
      RCLCPP_INFO(get_logger(), "检测到 Btn=17 上升沿，800ms 后触发 JTC 重启同步");
      std::thread([this]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        this->resyncControllerToActual();
      }).detach();
    }
    last_button_ = button;

    //RCLCPP_INFO(get_logger(), "收到下位机传值！");

    if (send_log_)
    {
      // serial_recv 中 real_joint 段为 0.01° 定点 raw 值，显示时还原为度
      constexpr double kRawToDeg = 1.0 / 100.0;
      RCLCPP_INFO(get_logger(),
                  "RX: x=%d y=%d z=%d P=%d R=%d Btn=%u | "
                  "Real_J(deg): %.2f %.2f %.2f %.2f %.2f %.2f",
                  arm_x, arm_y, arm_z, arm_pitch, arm_roll, button,
                  msg->data.size() >= 12 ? msg->data[6]  * kRawToDeg : 0.0,
                  msg->data.size() >= 12 ? msg->data[7]  * kRawToDeg : 0.0,
                  msg->data.size() >= 12 ? msg->data[8]  * kRawToDeg : 0.0,
                  msg->data.size() >= 12 ? msg->data[9]  * kRawToDeg : 0.0,
                  msg->data.size() >= 12 ? msg->data[10] * kRawToDeg : 0.0,
                  msg->data.size() >= 12 ? msg->data[11] * kRawToDeg : 0.0);
    }

    if (!enable_servo_control_) return;

    // 按键切换控制模式
    ControlMode new_mode = ControlMode::CARTESIAN;
    if (button == 9)
      new_mode = ControlMode::JOINT_GROUP_1;
    else if (button == 11)
      new_mode = ControlMode::JOINT_GROUP_2;
    else if (button == 12)
      new_mode = ControlMode::HOMING;

    ControlMode old_mode = control_mode_.exchange(new_mode);
    if (old_mode != new_mode)
    {
      const char* mode_str =
          (new_mode == ControlMode::CARTESIAN)     ? "笛卡尔 Servo" :
          (new_mode == ControlMode::JOINT_GROUP_1) ? "关节遥控 (前三轴: 1, 2, 3)" :
          (new_mode == ControlMode::JOINT_GROUP_2) ? "关节遥控 (后三轴: 4, 5, 6)" :
                                                     "回初始位姿";
      RCLCPP_INFO(get_logger(), "控制模式切换 → %s", mode_str);

      if (new_mode == ControlMode::HOMING && !homing_active_.load())
      {
        homing_active_ = true;
        triggerHoming();
      }
    }

    if (new_mode == ControlMode::CARTESIAN)
    {
      double vx = applyJoyAxis(arm_x, joy_deadzone_xyz_, 1000, joy_max_linear_);
      double vy = applyJoyAxis(arm_y, joy_deadzone_xyz_, 1000, joy_max_linear_);
      double vz = applyJoyAxis(arm_z, joy_deadzone_xyz_, 1000, joy_max_linear_);
      double v_roll  = applyJoyAxis(arm_roll,  joy_deadzone_angular_, 100, joy_max_angular_);
      double v_pitch = applyJoyAxis(arm_pitch, joy_deadzone_angular_, 100, joy_max_angular_);

      double q_dot[6] = {vy, 0.0, 0.0, v_roll, v_pitch, 0.0};

      if (jacobian_ready_ && robot_state_ready_.load() &&
          (vx != 0.0 || vz != 0.0))
      {
        Eigen::MatrixXd J;
        double yaw = 0.0;
        {
          std::lock_guard<std::mutex> lock(robot_state_mutex_);
          yaw = current_joint_pos_[0];
          robot_state_->getJacobian(jmg_, jacobian_link_,
                                    Eigen::Vector3d::Zero(), J);
        }
        Eigen::MatrixXd J_pitch(3, 2);
        J_pitch.col(0) = J.block<3, 1>(0, 1);
        J_pitch.col(1) = J.block<3, 1>(0, 2);

        Eigen::Vector3d v_des(decouple_ee_max_ * vx * std::cos(yaw),
                              decouple_ee_max_ * vx * std::sin(yaw),
                              decouple_ee_max_ * vz);

        Eigen::Vector2d q_pitch = dampedPinvSolve(J_pitch, v_des, 0.05);
        q_dot[1] = std::clamp(q_pitch[0], -1.0, 1.0);
        q_dot[2] = std::clamp(q_pitch[1], -1.0, 1.0);
      }

      geometry_msgs::msg::TwistStamped zero_twist;
      zero_twist.header.stamp    = now();
      zero_twist.header.frame_id = planning_frame_;
      servo_pub_->publish(zero_twist);

      control_msgs::msg::JointJog jog;
      jog.header.stamp    = now();
      jog.header.frame_id = planning_frame_;
      jog.joint_names = {"1-joint", "2-joint", "3-joint",
                         "4-joint", "5-joint", "6-joint"};
      jog.velocities  = {q_dot[0], q_dot[1], q_dot[2],
                         q_dot[3], q_dot[4], q_dot[5]};
      joint_jog_pub_->publish(jog);
    }
    else if (new_mode == ControlMode::JOINT_GROUP_1 ||
             new_mode == ControlMode::JOINT_GROUP_2)
    {
      geometry_msgs::msg::TwistStamped zero_twist;
      zero_twist.header.stamp    = now();
      zero_twist.header.frame_id = planning_frame_;
      servo_pub_->publish(zero_twist);

      const auto& joint_names = (new_mode == ControlMode::JOINT_GROUP_1)
                                    ? JOINT_GROUP_1_NAMES
                                    : JOINT_GROUP_2_NAMES;

      double vel_x = applyJoyAxis(arm_x, joy_deadzone_xyz_, 1000, joy_max_linear_);
      double vel_y = applyJoyAxis(arm_y, joy_deadzone_xyz_, 1000, joy_max_linear_);
      double vel_z = applyJoyAxis(arm_z, joy_deadzone_xyz_, 1000, joy_max_linear_);

      control_msgs::msg::JointJog jog;
      jog.header.stamp    = now();
      jog.header.frame_id = planning_frame_;
      jog.joint_names     = {joint_names[0], joint_names[1], joint_names[2]};
      jog.velocities      = {vel_x, vel_y, vel_z};
      joint_jog_pub_->publish(jog);
    }
    // HOMING 模式：不发任何 servo 指令，由 MoveGroupInterface 控制
  }

  // ---- /joint_states 回调：更新 RobotState 供 Jacobian 使用 + TX 日志 ----
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::unordered_map<std::string, double> pos_map;
    pos_map.reserve(msg->name.size());
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
      pos_map[msg->name[i]] = msg->position[i];

    const auto joint_names = get_parameter("joint_names").as_string_array();
    if (static_cast<int>(joint_names.size()) < 6) return;

    // state 日志：/joint_states 反馈的当前位置（非实际 TX，实际 TX 由 hw_interface 的 log_serial 输出）
    if (send_log_)
    {
      double deg[6]{};
      bool all_found = true;
      for (int i = 0; i < 6; ++i)
      {
        auto it = pos_map.find(joint_names[i]);
        if (it == pos_map.end()) { all_found = false; break; }
        deg[i] = (it->second - joint_offset_rad_[i]) * (180.0 / M_PI);
      }
      if (all_found)
      {
        RCLCPP_INFO(get_logger(),
                    "STATE  deg: %.2f %.2f %.2f %.2f %.2f %.2f",
                    deg[0], deg[1], deg[2], deg[3], deg[4], deg[5]);
      }
    }

    if (!jacobian_ready_) return;

    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    robot_state_->setVariablePositions(msg->name, msg->position);
    robot_state_->update();
    for (int i = 0; i < 6; ++i)
    {
      auto it = pos_map.find(joint_names[i]);
      if (it != pos_map.end())
        current_joint_pos_[i] = it->second;
    }
    robot_state_ready_ = true;
  }

  // ---- Servo start (runs in a detached thread) ----
  void callStartServo()
  {
    auto tmp    = rclcpp::Node::make_shared("serial_comm_servo_starter");
    auto client = tmp->create_client<std_srvs::srv::Trigger>(START_SERVO_SERVICE);

    if (!client->wait_for_service(std::chrono::seconds(15)))
    {
      RCLCPP_WARN(get_logger(), "servo_node 未运行");
      return;
    }

    auto future = client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());

    if (rclcpp::spin_until_future_complete(tmp, future, std::chrono::seconds(10)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "start_servo 调用超时");
      return;
    }

    if (!future.get()->success)
    {
      RCLCPP_WARN(get_logger(), "start_servo 返回 success=false");
      return;
    }

    RCLCPP_INFO(get_logger(), "start_servo OK，等待 Servo 初始化…");
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    RCLCPP_INFO(get_logger(), "Servo 控制就绪，开始接受摇杆指令");
  }

  // ---- Homing (回初始位姿) ----
  // 下位机电机零点在 ROS 坐标系下的弧度值
  static constexpr double joint_offset_rad_[6] = {
    0.0, 0.0, 1.5708, 1.5447, -1.5936, 0.3654
  };

  std::atomic<bool> homing_active_{false};

  void triggerHoming()
  {
    std::thread(&SerialCommNode::executeHoming, this).detach();
  }

  static bool callServiceViaTemp(const std::string& service_name,
                                 const rclcpp::Logger& logger)
  {
    auto tmp = rclcpp::Node::make_shared("homing_service_caller");
    auto client = tmp->create_client<std_srvs::srv::Trigger>(service_name);
    if (!client->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_WARN(logger, "服务 %s 不可用", service_name.c_str());
      return false;
    }
    auto future = client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    if (rclcpp::spin_until_future_complete(tmp, future, std::chrono::seconds(3)) ==
        rclcpp::FutureReturnCode::SUCCESS)
      return future.get()->success;
    RCLCPP_WARN(logger, "调用 %s 超时", service_name.c_str());
    return false;
  }

  // deactivate→activate manipulator_controller，让 JTC 用 hw_positions_ 做新起点。
  // 配合 ros2_controllers.yaml 里的 set_last_command_interface_value_as_state_on_activation:false
  // 实现 cmd 链对齐到真实 RX 位置。
  // 必须拆成两次独立调用——controller_manager 预检查整个请求，发现
  // controller 当前状态与请求冲突就直接 abort（同一次请求里 deactivate+activate
  // 同一个 controller 不会被模拟成顺序执行）
  //
  // Servo 也要走 stop→start：手动掰期间 servo 看到的 /commanded_joint_states 一直是旧 cmd，
  // 它的内部 smoothing filter 和"持守目标"卡在那个值上。如果只重启 JTC，servo 恢复发轨迹
  // 时 target 还是旧 cmd，会把刚对齐好的 JTC 又拉回去（实测 ~15ms 后 snap 回 0）。
  // stop_servo 会停止发布；start_servo 触发完整初始化，从最新 joint_topic 重采样。
  void resyncControllerToActual()
  {
    using SwitchController = controller_manager_msgs::srv::SwitchController;
    auto logger = get_logger();
    constexpr const char* kCtrl = "manipulator_controller";

    auto tmp = rclcpp::Node::make_shared("resync_service_caller");
    auto client = tmp->create_client<SwitchController>(
        "/controller_manager/switch_controller");
    if (!client->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_WARN(logger, "/controller_manager/switch_controller 不可用，跳过同步");
      return;
    }

    auto call_switch = [&](const std::vector<std::string>& deact,
                           const std::vector<std::string>& act,
                           const char* tag) -> bool {
      auto req = std::make_shared<SwitchController::Request>();
      req->deactivate_controllers = deact;
      req->activate_controllers   = act;
      req->strictness             = SwitchController::Request::STRICT;
      req->activate_asap          = true;

      auto future = client->async_send_request(req);
      if (rclcpp::spin_until_future_complete(tmp, future, std::chrono::seconds(3)) !=
          rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_WARN(logger, "%s 调用超时", tag);
        return false;
      }
      if (!future.get()->ok)
      {
        RCLCPP_WARN(logger, "%s 失败 (controller_manager 拒绝)", tag);
        return false;
      }
      return true;
    };

    // 1. 先停 servo，避免其继续往 JTC 发 target=旧cmd 的轨迹
    callServiceViaTemp("/servo_node/stop_servo", logger);

    // 2. JTC 走 deactivate→activate，对齐 last_commanded 到 hw_positions_
    if (!call_switch({kCtrl}, {}, "deactivate")) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (!call_switch({}, {kCtrl}, "activate")) return;

    // 3. 等 JTC 至少跑一拍，把对齐后的 hw_commands_ 发布到 /commanded_joint_states，
    //    这样下一步 start_servo 重采样时能看到正确的当前状态
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 4. 重启 servo：内部 filter 复位，从最新 joint_topic 采样作为持守目标
    callServiceViaTemp("/servo_node/start_servo", logger);

    RCLCPP_INFO(logger, "JTC + Servo 重启完成，cmd 已对齐到真实位置");
  }

  void executeHoming()
  {
    RCLCPP_INFO(get_logger(), "开始回初始位姿…");
    auto logger = get_logger();

    callServiceViaTemp("/servo_node/pause_servo", logger);

    try
    {
      auto move_group_node = rclcpp::Node::make_shared(
          "homing_move_group_node",
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

      move_group_node->set_parameter(rclcpp::Parameter(
          "robot_description", get_parameter("robot_description").as_string()));
      move_group_node->set_parameter(rclcpp::Parameter(
          "robot_description_semantic", get_parameter("robot_description_semantic").as_string()));

      auto move_group = moveit::planning_interface::MoveGroupInterface(
          move_group_node, "manipulator");

      move_group.setMaxVelocityScalingFactor(0.5);
      move_group.setMaxAccelerationScalingFactor(0.5);
      move_group.setPlanningTime(1.0);

      if (jacobian_ready_ && robot_state_ready_.load())
      {
        std::lock_guard<std::mutex> lock(robot_state_mutex_);
        robot_state_->enforceBounds();
        move_group.setStartState(*robot_state_);
      }

      auto joint_names = get_parameter("joint_names").as_string_array();
      std::vector<double> target_joints(joint_offset_rad_, joint_offset_rad_ + 6);
      move_group.setJointValueTarget(joint_names, target_joints);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(logger, "规划成功，执行…");
        auto r = move_group.execute(plan);
        if (r == moveit::core::MoveItErrorCode::SUCCESS)
          RCLCPP_INFO(logger, "已到达初始位姿");
        else
          RCLCPP_WARN(logger, "执行失败 (code: %d)", r.val);
      }
      else
      {
        RCLCPP_WARN(logger, "规划失败");
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger, "Homing 异常: %s", e.what());
    }

    callServiceViaTemp("/servo_node/stop_servo", logger);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    callServiceViaTemp("/servo_node/start_servo", logger);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    homing_active_ = false;
    control_mode_ = ControlMode::CARTESIAN;
    RCLCPP_INFO(logger, "Homing 完成，恢复笛卡尔 Servo");
  }

  // ---- Members ----
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr recv_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     joint_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr     servo_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr         joint_jog_pub_;

  bool                     enable_servo_control_{false};
  std::atomic<ControlMode> control_mode_{ControlMode::CARTESIAN};

  // Btn=17 上升沿检测（与下位机 Btn 码一致）
  uint8_t last_button_{0};

  std::string  planning_frame_;
  int16_t      joy_deadzone_xyz_{30};
  int16_t      joy_deadzone_angular_{3};
  double       joy_max_linear_{0.5};
  double       joy_max_angular_{0.5};
  double       decouple_ee_max_{0.3};
  bool         send_log_{false};

  // ---- Jacobian 关节限位退化 ----
  moveit::core::RobotModelPtr                     robot_model_;
  std::shared_ptr<moveit::core::RobotState>       robot_state_;
  const moveit::core::JointModelGroup*             jmg_{nullptr};
  const moveit::core::LinkModel*                   jacobian_link_{nullptr};
  std::mutex                                       robot_state_mutex_;
  double                                           current_joint_pos_[6]{};
  std::atomic<bool>                                robot_state_ready_{false};
  bool                                             jacobian_ready_{false};

  struct JointLimitInfo { bool has_limits; double lower; double upper; };
  JointLimitInfo joint_limits_[6]{};
  static constexpr double LIMIT_MARGIN_RAD = 1.0 * M_PI / 180.0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<SerialCommNode>();
    node->initJacobianFallback();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("serial_comm_node"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
