#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

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
    double tx[6];
    {
      std::lock_guard<std::mutex> lock(joint_tx_mutex_);
      if (joint_tx_ready_)
      {
        std::copy(latest_joint_tx_, latest_joint_tx_ + 6, tx);
      }
      else
      {
        auto param = get_parameter("tx_joints").as_double_array();
        if (static_cast<int>(param.size()) < 6) return;
        std::copy(param.begin(), param.begin() + 6, tx);
      }
    }
    sendFromDoubles(tx);
    if (log_auto_send_)
    {
      RCLCPP_INFO(get_logger(),
                  "auto_send: %d,%d,%d,%d,%d,%d deg",
                  static_cast<int>(tx[0]), static_cast<int>(tx[1]),
                  static_cast<int>(tx[2]), static_cast<int>(tx[3]),
                  static_cast<int>(tx[4]), static_cast<int>(tx[5]));
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

          // 发布到 ROS 话题，字段顺序：x, y, z, pitch, roll, button
          std_msgs::msg::Float64MultiArray rx_msg;
          rx_msg.data = {
            static_cast<double>(rx_copy.Arm_Pos_x),
            static_cast<double>(rx_copy.Arm_Pos_y),
            static_cast<double>(rx_copy.Arm_Pos_z),
            static_cast<double>(rx_copy.Arm_Pos_Pitch),
            static_cast<double>(rx_copy.Arm_Pos_Roll),
            static_cast<double>(rx_copy.Button)
          };
          recv_pub_->publish(rx_msg);
          RCLCPP_INFO(
              get_logger(),
              "RX [#%lu] x=%d  y=%d  z=%d  Pitch=%d  Roll=%d  Button=%u  EndFrame=%u",
              count,
              rx_copy.Arm_Pos_x,
              rx_copy.Arm_Pos_y,
              rx_copy.Arm_Pos_z,
              rx_copy.Arm_Pos_Pitch,
              rx_copy.Arm_Pos_Roll,
              rx_copy.Button,
              rx_copy.EndFrame);

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
              // 笛卡尔模式：发布 TwistStamped
              geometry_msgs::msg::TwistStamped twist;
              twist.header.stamp    = now();
              twist.header.frame_id = planning_frame_;
              twist.twist.linear.x  = applyJoyAxis(rx_copy.Arm_Pos_x,    joy_deadzone_xyz_,    1000, joy_max_linear_);
              twist.twist.linear.y  = applyJoyAxis(rx_copy.Arm_Pos_y,    joy_deadzone_xyz_,    1000, joy_max_linear_);
              twist.twist.linear.z  = applyJoyAxis(rx_copy.Arm_Pos_z,    joy_deadzone_xyz_,    1000, joy_max_linear_);
              twist.twist.angular.x = applyJoyAxis(rx_copy.Arm_Pos_Roll, joy_deadzone_angular_, 100, joy_max_angular_);
              twist.twist.angular.y = applyJoyAxis(rx_copy.Arm_Pos_Pitch,joy_deadzone_angular_, 100, joy_max_angular_);
              servo_pub_->publish(twist);
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

              double vel_x = applyJoyAxis(rx_copy.Arm_Pos_x, joy_deadzone_xyz_, 1000, joy_max_linear_);
              double vel_y = applyJoyAxis(rx_copy.Arm_Pos_y, joy_deadzone_xyz_, 1000, joy_max_linear_);
              double vel_z = applyJoyAxis(rx_copy.Arm_Pos_z, joy_deadzone_xyz_, 1000, joy_max_linear_);

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
      angles[i] = it->second * (180.0 / M_PI);  // rad → deg
    }

    {
      std::lock_guard<std::mutex> lock(joint_tx_mutex_);
      std::copy(angles, angles + 6, latest_joint_tx_);
      joint_tx_ready_ = true;
    }

    // autoSendTimer already sends latest_joint_tx_ at send_rate_hz;
    // sending here too would double TX traffic to the MCU.
  }

  // ---- Members ----

  TRoMaC::Uart uart_;

  std::thread       read_thread_;
  std::atomic<bool> running_{false};

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
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<SerialCommNode>();
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
