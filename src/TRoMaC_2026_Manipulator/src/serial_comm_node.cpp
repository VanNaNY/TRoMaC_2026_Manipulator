#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <Eigen/Dense>

#include "Serial.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>
#include <sys/select.h>

static constexpr char START_SERVO_SERVICE[] = "/servo_node/start_servo";

// 控制模式枚举
enum class ControlMode : int { CARTESIAN = 0, JOINT_GROUP_1 = 1, JOINT_GROUP_2 = 2 };

// 前三轴/后三轴关节名
static const std::vector<std::string> JOINT_GROUP_1_NAMES = {
    "yaw-1-joint", "pitch-1-joint", "pitch-2-joint"};
static const std::vector<std::string> JOINT_GROUP_2_NAMES = {
    "roll-1-joint", "pitch-3-joint", "roll-2-joint"};

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

// Apply deadband then scale a raw joystick value to [-max_output, +max_output].
// Values within [-deadzone, +deadzone] map to exactly 0.
// At full deflection (±max_raw) the output is ±max_output.
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
    // ---- Serial parameters ----
    declare_parameter("device", "/dev/ttyUSB0");
    declare_parameter("baud_rate", 921600);
    declare_parameter("send_once_on_start", false);
    declare_parameter("auto_send", true);
    declare_parameter("send_rate_hz", 100.0);
    declare_parameter("tx_joints", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    declare_parameter("log_auto_send", true);

    // ---- TX 平滑参数 ----
    // tx_smooth_alpha : EMA 平滑因子，0.0~1.0；越小越平滑（延迟越大），1.0=不平滑
    declare_parameter("tx_smooth_alpha", 0.5);
    tx_smooth_alpha_ = get_parameter("tx_smooth_alpha").as_double();
    tx_smooth_alpha_ = std::clamp(tx_smooth_alpha_, 0.01, 1.0);

    // ---- Servo-control parameters ----
    // enable_servo_control : publish TwistStamped from RX joystick data
    // planning_frame       : frame_id in TwistStamped header (must match servo config)
    // joy_deadzone_xyz     : dead-zone for Arm_Pos_x/y/z   (raw units, range ±1000)
    // joy_deadzone_angular : dead-zone for Arm_Pos_Pitch/Roll (raw units, range ±100)
    // joy_max_linear       : max normalised linear output  [0.0, 1.0]; 0.5 = half scale
    // joy_max_angular      : max normalised angular output [0.0, 1.0]; 0.5 = half scale
    declare_parameter("enable_servo_control", true);
    declare_parameter("planning_frame", std::string("base_link"));
    declare_parameter("joy_deadzone_xyz", 10);
    declare_parameter("joy_deadzone_angular", 1);
    declare_parameter("joy_max_linear", 1.0);
    declare_parameter("joy_max_angular", 1.0);

    // Cache parameters so the readLoop thread doesn't hit the parameter map.
    planning_frame_      = get_parameter("planning_frame").as_string();
    joy_deadzone_xyz_    = static_cast<int16_t>(get_parameter("joy_deadzone_xyz").as_int());
    joy_deadzone_angular_= static_cast<int16_t>(get_parameter("joy_deadzone_angular").as_int());
    joy_max_linear_      = get_parameter("joy_max_linear").as_double();
    joy_max_angular_     = get_parameter("joy_max_angular").as_double();
    log_auto_send_       = get_parameter("log_auto_send").as_bool();

    auto device = get_parameter("device").as_string();
    auto baud   = static_cast<int>(get_parameter("baud_rate").as_int());

    if (!uart_.Open(device, baud))
    {
      RCLCPP_FATAL(get_logger(), "无法打开串口: %s @ %d", device.c_str(), baud);
      throw std::runtime_error("Serial port open failed");
    }
    RCLCPP_INFO(get_logger(), "串口已打开: %s @ %d baud", device.c_str(), baud);

    // 必须在后台线程启动前创建所有 publisher/subscriber，避免线程调用 null publisher
    send_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "serial_send", 12,
        std::bind(&SerialCommNode::sendCallback, this, std::placeholders::_1));

    recv_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("serial_recv", 10);

    running_ = true;
    read_thread_ = std::thread(&SerialCommNode::readLoop, this);

    if (get_parameter("send_once_on_start").as_bool())
    {
      auto tx = get_parameter("tx_joints").as_double_array();
      if (tx.size() >= 6)
      {
        sendFromDoubles(tx.data());
        RCLCPP_INFO(get_logger(), "启动时已向下位机发送一帧 (tx_joints)");
      }
      else
      {
        RCLCPP_WARN(get_logger(),
                    "send_once_on_start 为 true 但 tx_joints 不足 6 个数，跳过启动发送");
      }
    }

    if (get_parameter("auto_send").as_bool())
    {
      double hz = get_parameter("send_rate_hz").as_double();
      if (hz <= 0.0) hz = 10.0;
      const auto period_ns = static_cast<int64_t>(1e9 / hz);
      send_timer_ = create_wall_timer(
          std::chrono::nanoseconds(period_ns),
          std::bind(&SerialCommNode::autoSendTimer, this));
      RCLCPP_INFO(get_logger(), "auto_send 已启动 @ %.1f Hz", hz);
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "设 auto_send:=true，或向话题 serial_send 发 Float64MultiArray(6个数)");
    }

    // ---- Joint-state → serial TX ----
    // enable_joint_state_tx : subscribe /joint_states and forward angles to MCU
    // joint_names           : ordered list of the 6 joint names (matches VisionData Joint_1..6)
    // log_joint_state_tx    : log each TX frame
    declare_parameter("enable_joint_state_tx", true);
    declare_parameter("joint_names", std::vector<std::string>{
        "yaw-1-joint", "pitch-1-joint", "pitch-2-joint",
        "roll-1-joint", "pitch-3-joint", "roll-2-joint"});
    declare_parameter("log_joint_state_tx", false);

    if (get_parameter("enable_joint_state_tx").as_bool())
    {
      joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", 10,
          std::bind(&SerialCommNode::jointStateCallback, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "joint_state → serial TX 已启用，订阅 /joint_states");
    }

    // ---- Servo control setup ----
    if (get_parameter("enable_servo_control").as_bool())
    {
      enable_servo_control_ = true;

      // 使用 SystemDefaultsQoS 匹配 servo_node subscriber 的 BEST_EFFORT QoS
      servo_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
          "/servo_node/delta_twist_cmds", rclcpp::SystemDefaultsQoS());

      // 关节模式 JointJog publisher
      joint_jog_pub_ = create_publisher<control_msgs::msg::JointJog>(
          "/servo_node/delta_joint_cmds", rclcpp::SystemDefaultsQoS());

      // 不再使用定时器发布——改为在 readLoop 中收到串口数据后直接发布 twist，
      // 和键盘节点一样的事件驱动模式。
      std::thread([this]() { this->callStartServo(); }).detach();

      RCLCPP_INFO(get_logger(),
                  "串口 Servo 控制已启用 (事件驱动)  (linear_max=%.2f  angular_max=%.2f)",
                  joy_max_linear_, joy_max_angular_);
    }
  }

  // 初始化 Jacobian 退化功能（需在 make_shared 后调用以获取 robot_description 参数）
  void initJacobianFallback()
  {
    if (!enable_servo_control_) return;

    // 从 launch 传入的参数读取 URDF / SRDF
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

    // 从 URDF 提取每个关节的限位
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
    RCLCPP_INFO(get_logger(),
                "Jacobian 关节限位退化模式已初始化 (margin=%.1f°)",
                LIMIT_MARGIN_RAD * 180.0 / M_PI);
  }

  ~SerialCommNode() override
  {
    running_ = false;
    if (read_thread_.joinable())
      read_thread_.join();
    uart_.Close();
  }

private:
  // ---- TX helpers ----

  void sendFromDoubles(const double* j)
  {
    TRoMaC::VisionData vd{};
    vd.Joint_1 = static_cast<int16_t>(j[0]);
    vd.Joint_2 = static_cast<int16_t>(j[1]);
    vd.Joint_3 = static_cast<int16_t>(j[2]);
    vd.Joint_4 = static_cast<int16_t>(j[3]);
    vd.Joint_5 = static_cast<int16_t>(j[4]);
    vd.Joint_6 = static_cast<int16_t>(j[5]);
    uart_.send(vd);
  }

  void autoSendTimer()
  {
    double target[6];
    {
      std::lock_guard<std::mutex> lock(joint_tx_mutex_);
      if (joint_tx_ready_)
      {
        std::copy(latest_joint_tx_, latest_joint_tx_ + 6, target);
      }
      else
      {
        auto param = get_parameter("tx_joints").as_double_array();
        if (static_cast<int>(param.size()) < 6) return;
        std::copy(param.begin(), param.begin() + 6, target);
      }
    }

    // ---- EMA 平滑插值 ----
    // 首帧直接跳到目标，避免从 0 开始缓慢爬升
    if (!tx_smooth_initialized_)
    {
      std::copy(target, target + 6, smoothed_tx_);
      tx_smooth_initialized_ = true;
    }
    else
    {
      for (int i = 0; i < 6; ++i)
      {
        smoothed_tx_[i] += tx_smooth_alpha_ * (target[i] - smoothed_tx_[i]);
      }
    }

    sendFromDoubles(smoothed_tx_);
    if (log_auto_send_)
    {
      RCLCPP_INFO(get_logger(),
                  "auto_send(smooth): %.1f,%.1f,%.1f,%.1f,%.1f,%.1f deg",
                  smoothed_tx_[0], smoothed_tx_[1],
                  smoothed_tx_[2], smoothed_tx_[3],
                  smoothed_tx_[4], smoothed_tx_[5]);
    }
  }

  // ---- RX background thread ----

  void readLoop()
  {
    while (running_)
    {
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(uart_.serial_id, &fds);
      struct timeval tv{};
      tv.tv_sec  = 0;
      tv.tv_usec = 300000;  // 300 ms select timeout

      int ret = select(uart_.serial_id + 1, &fds, nullptr, nullptr, &tv);
      if (ret > 0 && FD_ISSET(uart_.serial_id, &fds))
      {
        while (running_ && uart_.ReadData())
        {
          TRoMaC::VisionFrameRX_structTypedef rx_copy{};
          uint64_t count = 0;
          {
            std::lock_guard<std::mutex> lock(rx_mutex_);
            latest_rx_    = uart_.read_data;
            last_rx_time_ = std::chrono::steady_clock::now();
            rx_count_++;
            rx_copy = latest_rx_;
            count   = rx_count_;
          }

          // 发布到 ROS 话题，字段顺序：x, y, z, pitch, roll, button, real_j1~j6
          std_msgs::msg::Float64MultiArray rx_msg;
          rx_msg.data = {
            static_cast<double>(rx_copy.Arm_Pos_x),
            static_cast<double>(rx_copy.Arm_Pos_y),
            static_cast<double>(rx_copy.Arm_Pos_z),
            static_cast<double>(rx_copy.Arm_Pos_Pitch),
            static_cast<double>(rx_copy.Arm_Pos_Roll),
            static_cast<double>(rx_copy.Button),
            static_cast<double>(rx_copy.Real_Joint_1),
            static_cast<double>(rx_copy.Real_Joint_2),
            static_cast<double>(rx_copy.Real_Joint_3),
            static_cast<double>(rx_copy.Real_Joint_4),
            static_cast<double>(rx_copy.Real_Joint_5),
            static_cast<double>(rx_copy.Real_Joint_6)
          };
          recv_pub_->publish(rx_msg);
          RCLCPP_INFO(
              get_logger(),
              "RX [#%lu] x=%d y=%d z=%d Pitch=%d Roll=%d Button=%u | "
              "Real_J: %d %d %d %d %d %d",
              count,
              rx_copy.Arm_Pos_x,
              rx_copy.Arm_Pos_y,
              rx_copy.Arm_Pos_z,
              rx_copy.Arm_Pos_Pitch,
              rx_copy.Arm_Pos_Roll,
              rx_copy.Button,
              rx_copy.Real_Joint_1,
              rx_copy.Real_Joint_2,
              rx_copy.Real_Joint_3,
              rx_copy.Real_Joint_4,
              rx_copy.Real_Joint_5,
              rx_copy.Real_Joint_6);

          // DEBUG: 打印 RX payload 原始 hex，排查下位机是否填充了 Real_Joint 字段
         /* {
            const auto* p = reinterpret_cast<const uint8_t*>(&rx_copy);
            char hex[RX_PAYLOAD_SIZE * 3 + 1];
            for (int i = 0; i < RX_PAYLOAD_SIZE; ++i)
              snprintf(hex + i * 3, 4, "%02X ", p[i]);
            hex[RX_PAYLOAD_SIZE * 3] = '\0';
            RCLCPP_INFO(get_logger(), "RX payload hex: %s", hex);
          }*/

          // 根据按键切换控制模式（电平式：按下瞬间触发，松开回到其他值）
          if (enable_servo_control_)
          {
            // 更新控制模式
            ControlMode new_mode = ControlMode::CARTESIAN;
            if (rx_copy.Button == 13)
              new_mode = ControlMode::JOINT_GROUP_1;
            else if (rx_copy.Button == 11)
              new_mode = ControlMode::JOINT_GROUP_2;

            ControlMode old_mode = control_mode_.exchange(new_mode);
            if (old_mode != new_mode)
            {
              const char* mode_str =
                  (new_mode == ControlMode::CARTESIAN)     ? "笛卡尔 Servo" :
                  (new_mode == ControlMode::JOINT_GROUP_1) ? "关节遥控 (前三轴: yaw-1, pitch-1, pitch-2)" :
                                                             "关节遥控 (后三轴: roll-1, pitch-3, roll-2)";
              RCLCPP_INFO(get_logger(), "控制模式切换 → %s", mode_str);
            }

            if (new_mode == ControlMode::CARTESIAN)
            {
              // 平移走笛卡尔 Servo，Roll/Pitch 直接控制末端关节
              double roll_vel  = applyJoyAxis(rx_copy.Arm_Pos_Roll,  joy_deadzone_angular_, 100, joy_max_angular_);
              double pitch_vel = applyJoyAxis(rx_copy.Arm_Pos_Pitch, joy_deadzone_angular_, 100, joy_max_angular_);
              double lx = applyJoyAxis(rx_copy.Arm_Pos_y, joy_deadzone_xyz_, 1000, joy_max_linear_);
              double ly = applyJoyAxis(rx_copy.Arm_Pos_z, joy_deadzone_xyz_, 1000, joy_max_linear_);
              double lz = applyJoyAxis(rx_copy.Arm_Pos_x, joy_deadzone_xyz_, 1000, joy_max_linear_);

              // ---------- Jacobian 关节限位退化判定 ----------
              bool use_fallback = false;

              if (jacobian_ready_ && robot_state_ready_.load() &&
                  (std::abs(lx) > 0.0 || std::abs(ly) > 0.0 || std::abs(lz) > 0.0))
              {
                // 方向切换检测：如果方向变了，立即回到笛卡尔模式
                if (directionChanged(lx, ly, lz))
                  fallback_active_ = false;

                // 取当前关节位置 + Jacobian
                double jpos[6];
                Eigen::MatrixXd jacobian;
                {
                  std::lock_guard<std::mutex> lock(robot_state_mutex_);
                  std::copy(current_joint_pos_, current_joint_pos_ + 6, jpos);
                  jacobian = robot_state_->getJacobian(jmg_);
                }

                // 取 Jacobian 线速度部分 (3×6)
                Eigen::MatrixXd J_lin = jacobian.block(0, 0, 3, 6);
                Eigen::Vector3d v_desired(lx, ly, lz);

                // 先用完整 Jacobian 算一次理论关节速度
                Eigen::VectorXd q_dot_full = dampedPinvSolve(J_lin, v_desired);

                // 检测哪些关节碰到限位
                bool is_limiting[6] = {};
                bool any_limiting = false;
                for (int i = 0; i < 6; ++i)
                {
                  if (!joint_limits_[i].has_limits) continue;
                  if (jpos[i] >= joint_limits_[i].upper - LIMIT_MARGIN_RAD && q_dot_full[i] > 0.0)
                  {
                    is_limiting[i] = true;
                    any_limiting = true;
                  }
                  if (jpos[i] <= joint_limits_[i].lower + LIMIT_MARGIN_RAD && q_dot_full[i] < 0.0)
                  {
                    is_limiting[i] = true;
                    any_limiting = true;
                  }
                }

                if (any_limiting)
                {
                  use_fallback = true;
                  fallback_active_ = true;

                  // 清零碰限位关节的 Jacobian 列，重新求解
                  Eigen::MatrixXd J_reduced = J_lin;
                  for (int i = 0; i < 6; ++i)
                    if (is_limiting[i]) J_reduced.col(i).setZero();

                  Eigen::VectorXd q_dot = dampedPinvSolve(J_reduced, v_desired);
                  for (int i = 0; i < 6; ++i)
                    if (is_limiting[i]) q_dot[i] = 0.0;

                  // 叠加末端 Roll/Pitch 直接控制
                  q_dot[4] += pitch_vel;  // pitch-3-joint
                  q_dot[3] += roll_vel;   // roll-1-joint

                  // 钳位到 [-1, 1]（unitless，Servo 会乘以 scale.joint）
                  for (int i = 0; i < 6; ++i)
                    q_dot[i] = std::clamp(q_dot[i], -1.0, 1.0);

                  // 发零 Twist 清 Servo 缓冲（Twist 优先于 JointJog）
                  geometry_msgs::msg::TwistStamped zero_twist;
                  zero_twist.header.stamp    = now();
                  zero_twist.header.frame_id = planning_frame_;
                  servo_pub_->publish(zero_twist);

                  // 发 JointJog 驱动未碰限位的关节
                  auto jnames = get_parameter("joint_names").as_string_array();
                  control_msgs::msg::JointJog jog;
                  jog.header.stamp    = now();
                  jog.header.frame_id = planning_frame_;
                  jog.joint_names.assign(jnames.begin(), jnames.end());
                  jog.velocities = {q_dot[0], q_dot[1], q_dot[2],
                                    q_dot[3], q_dot[4], q_dot[5]};
                  joint_jog_pub_->publish(jog);

                  if (!prev_fallback_active_)
                  {
                    std::string limiting_str;
                    for (int i = 0; i < 6; ++i)
                      if (is_limiting[i])
                        limiting_str += jnames[i] + " ";
                    RCLCPP_INFO(get_logger(),
                                "限位退化 → JointJog (碰限位关节: %s)", limiting_str.c_str());
                  }
                  prev_fallback_active_ = true;
                }
                else
                {
                  fallback_active_ = false;
                  if (prev_fallback_active_)
                  {
                    RCLCPP_INFO(get_logger(), "限位退化结束 → 笛卡尔 Servo");
                    prev_fallback_active_ = false;
                  }
                }
              }
              else
              {
                // 摇杆回中或 Jacobian 未就绪：重置退化状态
                fallback_active_ = false;
                if (prev_fallback_active_)
                {
                  RCLCPP_INFO(get_logger(), "限位退化结束 → 笛卡尔 Servo");
                  prev_fallback_active_ = false;
                }
              }

              // ---------- 正常笛卡尔模式（未触发退化时） ----------
              if (!use_fallback)
              {
                // 1) TwistStamped: 仅平移，angular 全零
                geometry_msgs::msg::TwistStamped twist;
                twist.header.stamp    = now();
                twist.header.frame_id = planning_frame_;
                twist.twist.linear.x  = lx;
                twist.twist.linear.y  = ly;
                twist.twist.linear.z  = lz;
                servo_pub_->publish(twist);

                // 2) JointJog: Roll → roll-1-joint, Pitch → pitch-3-joint
                if (std::abs(roll_vel) > 0.0 || std::abs(pitch_vel) > 0.0)
                {
                  control_msgs::msg::JointJog jog;
                  jog.header.stamp    = now();
                  jog.header.frame_id = planning_frame_;
                  jog.joint_names     = {"pitch-3-joint", "roll-1-joint"};
                  jog.velocities      = {pitch_vel, roll_vel};
                  joint_jog_pub_->publish(jog);
                }
              }
            }
            else
            {
              // 关节模式：先发全零 Twist 清除 Servo 内部缓存的笛卡尔指令，
              // 否则 Servo 会优先执行上一帧残留的 Twist 导致其他关节跟着动
              geometry_msgs::msg::TwistStamped zero_twist;
              zero_twist.header.stamp    = now();
              zero_twist.header.frame_id = planning_frame_;
              servo_pub_->publish(zero_twist);

              // 发布 JointJog，x/y/z 轴映射到对应的 3 个关节
              const auto& joint_names = (new_mode == ControlMode::JOINT_GROUP_1)
                                            ? JOINT_GROUP_1_NAMES
                                            : JOINT_GROUP_2_NAMES;

              double vel_x = applyJoyAxis(rx_copy.Arm_Pos_y, joy_deadzone_xyz_, 1000, joy_max_linear_);
              double vel_y = applyJoyAxis(rx_copy.Arm_Pos_z, joy_deadzone_xyz_, 1000, joy_max_linear_);
              double vel_z = applyJoyAxis(rx_copy.Arm_Pos_x, joy_deadzone_xyz_, 1000, joy_max_linear_);

              control_msgs::msg::JointJog jog;
              jog.header.stamp    = now();
              jog.header.frame_id = planning_frame_;
              jog.joint_names     = {joint_names[0], joint_names[1], joint_names[2]};
              jog.velocities      = {vel_x, vel_y, vel_z};
              joint_jog_pub_->publish(jog);
            }
          }
        }
      }
    }
  }

  // ---- Servo start (runs in a detached thread) ----

  void callStartServo()
  {
    // Use a temporary one-shot node for the service call so the main node's
    // executor is not affected.
    auto tmp    = rclcpp::Node::make_shared("serial_comm_servo_starter");
    auto client = tmp->create_client<std_srvs::srv::Trigger>(START_SERVO_SERVICE);

    if (!client->wait_for_service(std::chrono::seconds(15)))
    {
      RCLCPP_WARN(get_logger(),
                  "servo_node 未运行");
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
    RCLCPP_INFO(get_logger(), "Servo 控制就绪，开始接受下位机摇杆指令");
  }

  // ---- serial_send topic callback ----

  void sendCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 6) return;

    sendFromDoubles(msg->data.data());

    RCLCPP_INFO(get_logger(),
                "Send: Joint_1=%d, Joint_2=%d, Joint_3=%d, Joint_4=%d, Joint_5=%d, Joint_6=%d",
                static_cast<int>(msg->data[0]), static_cast<int>(msg->data[1]),
                static_cast<int>(msg->data[2]), static_cast<int>(msg->data[3]),
                static_cast<int>(msg->data[4]), static_cast<int>(msg->data[5]));
  }

  // ---- /joint_states → serial TX ----

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Build name→position map for O(1) lookup
    std::unordered_map<std::string, double> pos_map;
    pos_map.reserve(msg->name.size());
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
      pos_map[msg->name[i]] = msg->position[i];

    const auto joint_names = get_parameter("joint_names").as_string_array();
    if (static_cast<int>(joint_names.size()) < 6) return;

    double angles[6];
    for (int i = 0; i < 6; ++i)
    {
      auto it = pos_map.find(joint_names[i]);
      if (it == pos_map.end()) return;  // 本帧不含全部关节，跳过
      // ROS 弧度 → 下位机度数：先减去 ROS 零位偏移（即下位机零点对应的 ROS 弧度值），
      // 得到以下位机零点为基准的弧度差，再转整数度。
      angles[i] = (it->second - joint_offset_rad_[i]) * (180.0 / M_PI);
    }

    {
      std::lock_guard<std::mutex> lock(joint_tx_mutex_);
      std::copy(angles, angles + 6, latest_joint_tx_);
      joint_tx_ready_ = true;
    }

    // autoSendTimer already sends latest_joint_tx_ at send_rate_hz;
    // sending here too would double TX traffic to the MCU.

    // 更新 RobotState 供 Jacobian 退化使用
    if (jacobian_ready_)
    {
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
  }

  // ---- 方向切换检测 ----
  bool directionChanged(double lx, double ly, double lz)
  {
    auto sgn = [](double v) -> int { return (v > 0.0) ? 1 : (v < 0.0) ? -1 : 0; };
    int sx = sgn(lx), sy = sgn(ly), sz = sgn(lz);
    bool changed = false;

    // 摇杆回中
    if (sx == 0 && sy == 0 && sz == 0 &&
        (prev_sign_[0] || prev_sign_[1] || prev_sign_[2]))
      changed = true;

    // 任一轴反向
    if ((prev_sign_[0] && sx && prev_sign_[0] != sx) ||
        (prev_sign_[1] && sy && prev_sign_[1] != sy) ||
        (prev_sign_[2] && sz && prev_sign_[2] != sz))
      changed = true;

    prev_sign_[0] = sx;
    prev_sign_[1] = sy;
    prev_sign_[2] = sz;
    return changed;
  }

  // ---- Members ----

  TRoMaC::Uart uart_;

  std::thread       read_thread_;
  std::atomic<bool> running_{false};

  // 下位机电机零点在 ROS 坐标系下的弧度值 (initial_positions)
  // 顺序：yaw-1, pitch-1, pitch-2, roll-1, pitch-3, roll-2
  static constexpr double joint_offset_rad_[6] = {
    -3.1415,   // yaw-1-joint
    -3.1416,   // pitch-1-joint
    -3.7070,   // pitch-2-joint
     0.0868,   // roll-1-joint
     0.4750,   // pitch-3-joint
     0.0        // roll-2-joint
  };

  std::mutex                            rx_mutex_;
  TRoMaC::VisionFrameRX_structTypedef   latest_rx_{};
  uint64_t                              rx_count_{0};
  std::chrono::steady_clock::time_point last_rx_time_{};

  rclcpp::TimerBase::SharedPtr                                       send_timer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr  send_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr     recv_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr      joint_state_sub_;

  std::mutex joint_tx_mutex_;
  double     latest_joint_tx_[6]{};
  bool       joint_tx_ready_{false};

  // ---- TX EMA 平滑状态 ----
  double tx_smooth_alpha_{0.5};         // EMA 因子，参数 tx_smooth_alpha
  double smoothed_tx_[6]{};             // 当前平滑后角度 (度)
  bool   tx_smooth_initialized_{false}; // 首帧标志

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr     servo_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr         joint_jog_pub_;
  bool                                                               enable_servo_control_{false};
  std::atomic<ControlMode>                                           control_mode_{ControlMode::CARTESIAN};

  // Cached parameters (read once in ctor)
  std::string  planning_frame_;
  int16_t      joy_deadzone_xyz_{30};
  int16_t      joy_deadzone_angular_{3};
  double       joy_max_linear_{0.5};
  double       joy_max_angular_{0.5};
  bool         log_auto_send_{false};

  // ---- Jacobian 关节限位退化 ----
  moveit::core::RobotModelPtr                     robot_model_;
  std::shared_ptr<moveit::core::RobotState>       robot_state_;
  const moveit::core::JointModelGroup*             jmg_{nullptr};
  std::mutex                                       robot_state_mutex_;
  double                                           current_joint_pos_[6]{};
  std::atomic<bool>                                robot_state_ready_{false};
  bool                                             jacobian_ready_{false};
  bool                                             fallback_active_{false};
  bool                                             prev_fallback_active_{false};

  struct JointLimitInfo { bool has_limits; double lower; double upper; };
  JointLimitInfo joint_limits_[6]{};
  static constexpr double LIMIT_MARGIN_RAD = 1.0 * M_PI / 180.0;  // 1°

  int prev_sign_[3]{0, 0, 0};  // 方向追踪
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
