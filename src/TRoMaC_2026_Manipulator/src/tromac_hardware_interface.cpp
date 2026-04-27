// TRoMaC Hardware Interface — ros2_control SystemInterface 插件
// 通过串口与下位机通信：
//   read()  : 从 RX 帧读取 Real_Joint_1~6（下位机度数）→ 转 ROS 弧度 → state position
//             同时把摇杆数据发布到 /serial_recv topic
//   write() : command position（ROS 弧度）→ 减零位偏移 → 转下位机度数 → 串口 TX

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "Serial.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <sys/select.h>

namespace tromac_hardware
{

class TRoMaCHardwareInterface : public hardware_interface::SystemInterface
{
public:
  // 下位机电机零点在 ROS 坐标系下的弧度值 (与 initial_positions.yaml 一致)
  static constexpr double JOINT_OFFSET_RAD[6] = {
    0.0,
    0.0,
    1.5708,
    1.5447,
    -1.5936,
    0.3654
  };

  // 下位机 ↔ ROS 关节旋转方向系数
  static constexpr double JOINT_DIR[6] = {
    1.0, 1.0, -1.0, -1.0, 1.0, 1.0
  };

  // 串口收发使用 int16_t，但单位为 0.01° (定点)，避免整数度量化导致 MCU 死区
  // 量程 ±327.67°，关节角不会越界
  static constexpr double DEG_SCALE = 100.0;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 从 xacro <param> 读取串口配置
    device_ = info_.hardware_parameters.count("device")
                  ? info_.hardware_parameters.at("device")
                  : "/dev/ttyUSB0";
    baud_rate_ = info_.hardware_parameters.count("baud_rate")
                     ? std::stoi(info_.hardware_parameters.at("baud_rate"))
                     : 921600;

    // 日志开关（xacro <param name="log_serial">true</param>）
    if (info_.hardware_parameters.count("log_serial"))
    {
      const auto& val = info_.hardware_parameters.at("log_serial");
      log_serial_ = (val == "true" || val == "True" || val == "1");
    }

    // 初始化关节数据
    hw_positions_.resize(6, 0.0);
    hw_velocities_.resize(6, 0.0);
    hw_commands_.resize(6, 0.0);

    // 设置初始位置为零位偏移（与 initial_positions.yaml 一致）
    for (int i = 0; i < 6; ++i)
    {
      hw_positions_[i] = JOINT_OFFSET_RAD[i];
      hw_commands_[i]  = JOINT_OFFSET_RAD[i];
    }

    // 创建内部 ROS node 用于发布摇杆数据 + commanded joint states
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("tromac_hw_iface_internal");
    recv_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("serial_recv", 10);
    // /commanded_joint_states：发的是 hw_commands_（servo 用作 "current state" 视图，
    // 让 servo 内部 open-loop 累积 cmd，避免 MCU 慢响应导致 cmd 在原地振荡）
    cmd_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "/commanded_joint_states", rclcpp::SystemDefaultsQoS());

    RCLCPP_INFO(rclcpp::get_logger("TRoMaCHardwareInterface"),
                "on_init: device=%s, baud=%d, log_serial=%s (closed-loop)",
                device_.c_str(), baud_rate_, log_serial_ ? "true" : "false");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    // 打开串口
    if (!uart_.Open(device_, baud_rate_))
    {
      RCLCPP_FATAL(rclcpp::get_logger("TRoMaCHardwareInterface"),
                   "无法打开串口: %s @ %d", device_.c_str(), baud_rate_);
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("TRoMaCHardwareInterface"),
                "串口已打开: %s @ %d baud", device_.c_str(), baud_rate_);

    // 启动后台 RX 线程
    running_ = true;
    read_thread_ = std::thread(&TRoMaCHardwareInterface::readLoop, this);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    // 激活前必须把 hw_commands_ 同步到下位机当前位置，否则控制器会用 OFFSET
    // 作起点，把机械臂从任意位置一拍突跳到 home
    auto logger = rclcpp::get_logger("TRoMaCHardwareInterface");
    constexpr int kMaxRetry = 30;
    for (int retry = 0; retry < kMaxRetry; ++retry)
    {
      {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        if (rx_data_valid_)
        {
          for (int i = 0; i < 6; ++i)
          {
            hw_positions_[i] = rx_joint_rad_[i];
            hw_commands_[i]  = rx_joint_rad_[i];
          }
          RCLCPP_INFO(logger,
                      "硬件接口已激活，hw_commands_ 已同步到下位机真实位置");
          return hardware_interface::CallbackReturn::SUCCESS;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_WARN(logger,
                "硬件接口已激活，但 3s 内未收到下位机 RX 数据，"
                "hw_commands_ 仍为 OFFSET 初值——控制器启动可能导致大幅突跳");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    RCLCPP_INFO(rclcpp::get_logger("TRoMaCHardwareInterface"), "硬件接口已停用");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    running_ = false;
    if (read_thread_.joinable()) read_thread_.join();
    uart_.Close();
    RCLCPP_INFO(rclcpp::get_logger("TRoMaCHardwareInterface"), "串口已关闭");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    running_ = false;
    if (read_thread_.joinable()) read_thread_.join();
    uart_.Close();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
    }
    return command_interfaces;
  }

  // ros2_control update loop 调用 (100 Hz)
  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // 标准闭环：永远把下位机真实编码器值灌给 hw_positions_。
    // 若 RX 暂未到达，hw_positions_ 保留上一周期值（on_activate 已用首帧 RX 初始化）。
    //
    // hw_commands_ 同步到 hw_positions_：兜底 controller deactivate→activate 间隙
    // （resync 期间 ~50ms）。配 open_loop_control:true，JTC active 时不读 hw_commands_，
    // update() 会覆盖；force-sync 窗口内 write() 也会覆盖成 snapshot_pos_。
    std::lock_guard<std::mutex> lock(rx_mutex_);
    if (rx_data_valid_)
    {
      for (int i = 0; i < 6; ++i)
      {
        hw_positions_[i] = rx_joint_rad_[i];
        hw_commands_[i]  = rx_joint_rad_[i];
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // force-sync 窗口：TX 恒等于 Btn=17 那一刻抓拍的 snapshot_pos_，不跟 RX 漂移。
    // 避免「RX 抖动 → TX 跟随 → MCU 跟随 → encoder 跟随 → RX 进一步漂移」的正反馈环。
    // serial_comm_node 在 ~800ms 后做 JTC restart，那时 MCU 已稳在 snapshot 上，JTC
    // 内部 last_commanded 锁定到此值；窗口结束后无缝交接，TX 不阶跃。
    const auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
    if (now_ns < force_sync_until_ns_.load(std::memory_order_acquire))
    {
      for (int i = 0; i < 6; ++i)
        hw_commands_[i] = snapshot_pos_[i];
    }

    // command position (ROS 弧度) → 减零位偏移 → 转下位机定点度数 → 串口 TX
    TRoMaC::VisionData vd{};
    int16_t joints[6];
    double  degs[6];
    for (int i = 0; i < 6; ++i)
    {
      degs[i] = JOINT_DIR[i] * (hw_commands_[i] - JOINT_OFFSET_RAD[i]) * (180.0 / M_PI);
      joints[i] = static_cast<int16_t>(std::lround(degs[i] * DEG_SCALE));
    }
    vd.Joint_1 = joints[0];
    vd.Joint_2 = joints[1];
    vd.Joint_3 = joints[2];
    vd.Joint_4 = joints[3];
    vd.Joint_5 = joints[4];
    vd.Joint_6 = joints[5];
    uart_.send(vd);

    // 发布 commanded joint state，供 servo 作为 "current state" 使用
    // 这样 servo 不读 RX 反馈，target = last_cmd + Δ 会持续累积，
    // TX 不再在原地振荡，下位机能收到稳定推进的指令
    sensor_msgs::msg::JointState cmd_msg;
    cmd_msg.header.stamp = node_->now();
    cmd_msg.name.reserve(info_.joints.size());
    cmd_msg.position.reserve(info_.joints.size());
    cmd_msg.velocity.assign(info_.joints.size(), 0.0);
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      cmd_msg.name.push_back(info_.joints[i].name);
      cmd_msg.position.push_back(hw_commands_[i]);
    }
    cmd_state_pub_->publish(cmd_msg);

    if (log_serial_)
    {
      RCLCPP_INFO(rclcpp::get_logger("TRoMaCHardwareInterface"),
                  "TX: cmd_deg: %.2f %.2f %.2f %.2f %.2f %.2f",
                  degs[0], degs[1], degs[2],
                  degs[3], degs[4], degs[5]);
    }

    return hardware_interface::return_type::OK;
  }

private:
  // 后台线程：持续从串口读取 RX 帧
  void readLoop()
  {
    auto logger = rclcpp::get_logger("TRoMaCHardwareInterface");
    while (running_.load())
    {
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(uart_.serial_id, &fds);
      struct timeval tv{};
      tv.tv_sec  = 0;
      tv.tv_usec = 300000;  // 300 ms timeout

      int ret = select(uart_.serial_id + 1, &fds, nullptr, nullptr, &tv);
      if (ret > 0 && FD_ISSET(uart_.serial_id, &fds))
      {
        while (running_.load() && uart_.ReadData())
        {
          TRoMaC::VisionFrameRX_structTypedef rx = uart_.read_data;

          // 更新关节角度 (下位机度数 → ROS 弧度)
          const bool button_rising = (rx.Button == 17 && prev_button_ != 17);
          {
            std::lock_guard<std::mutex> lock(rx_mutex_);
            constexpr double RAW_TO_RAD = M_PI / 180.0 / DEG_SCALE;
            rx_joint_rad_[0] = JOINT_DIR[0] * static_cast<double>(rx.Real_Joint_1) * RAW_TO_RAD + JOINT_OFFSET_RAD[0];
            rx_joint_rad_[1] = JOINT_DIR[1] * static_cast<double>(rx.Real_Joint_2) * RAW_TO_RAD + JOINT_OFFSET_RAD[1];
            rx_joint_rad_[2] = JOINT_DIR[2] * static_cast<double>(rx.Real_Joint_3) * RAW_TO_RAD + JOINT_OFFSET_RAD[2];
            rx_joint_rad_[3] = JOINT_DIR[3] * static_cast<double>(rx.Real_Joint_4) * RAW_TO_RAD + JOINT_OFFSET_RAD[3];
            rx_joint_rad_[4] = JOINT_DIR[4] * static_cast<double>(rx.Real_Joint_5) * RAW_TO_RAD + JOINT_OFFSET_RAD[4];
            rx_joint_rad_[5] = JOINT_DIR[5] * static_cast<double>(rx.Real_Joint_6) * RAW_TO_RAD + JOINT_OFFSET_RAD[5];
            rx_data_valid_ = true;

            // Btn=17 上升沿：抓拍当前 actual 作为 snapshot。force-sync 窗口里 TX
            // 恒等于此 snapshot（不跟 RX），让 MCU 看到稳定 cmd 就地不动。
            if (button_rising)
            {
              for (int i = 0; i < 6; ++i)
                snapshot_pos_[i] = rx_joint_rad_[i];
            }
          }

          // Btn=17 上升沿：启动 1.2s force-sync 窗口。release 语义保证 snapshot_pos_
          // 写入对 write() 的 acquire load 可见。1.2s = 800ms 等 MCU 稳定 + ~200ms
          // serial_comm_node 走 JTC restart + 200ms 缓冲。
          if (button_rising)
          {
            const auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::milliseconds(1200);
            force_sync_until_ns_.store(
                deadline.time_since_epoch().count(),
                std::memory_order_release);
            RCLCPP_INFO(logger,
                        "Btn=17 上升沿，snapshot=[%.3f %.3f %.3f %.3f %.3f %.3f] rad，"
                        "启动 1.2s force-sync 窗口",
                        snapshot_pos_[0], snapshot_pos_[1], snapshot_pos_[2],
                        snapshot_pos_[3], snapshot_pos_[4], snapshot_pos_[5]);
          }
          prev_button_ = rx.Button;

          if (log_serial_)
          {
            RCLCPP_INFO(logger,
                        "RX  Joy: x=%d y=%d z=%d P=%d R=%d Btn=%u | "
                        "Real_J(deg): %.2f %.2f %.2f %.2f %.2f %.2f",
                        rx.Arm_Pos_x, rx.Arm_Pos_y, rx.Arm_Pos_z,
                        rx.Arm_Pos_Pitch, rx.Arm_Pos_Roll, rx.Button,
                        rx.Real_Joint_1 / DEG_SCALE, rx.Real_Joint_2 / DEG_SCALE,
                        rx.Real_Joint_3 / DEG_SCALE, rx.Real_Joint_4 / DEG_SCALE,
                        rx.Real_Joint_5 / DEG_SCALE, rx.Real_Joint_6 / DEG_SCALE);
          }

          // 发布摇杆数据到 /serial_recv，供 serial_comm_node 订阅
          std_msgs::msg::Float64MultiArray msg;
          msg.data = {
            static_cast<double>(rx.Arm_Pos_x),
            static_cast<double>(rx.Arm_Pos_y),
            static_cast<double>(rx.Arm_Pos_z),
            static_cast<double>(rx.Arm_Pos_Pitch),
            static_cast<double>(rx.Arm_Pos_Roll),
            static_cast<double>(rx.Button),
            static_cast<double>(rx.Real_Joint_1),
            static_cast<double>(rx.Real_Joint_2),
            static_cast<double>(rx.Real_Joint_3),
            static_cast<double>(rx.Real_Joint_4),
            static_cast<double>(rx.Real_Joint_5),
            static_cast<double>(rx.Real_Joint_6)
          };
          recv_pub_->publish(msg);
        }
      }
    }
  }

  // 串口
  TRoMaC::Uart uart_;
  std::string device_;
  int baud_rate_{921600};
  bool log_serial_{false};

  // 后台 RX 线程
  std::thread       read_thread_;
  std::atomic<bool> running_{false};

  // RX 数据（mutex 保护，readLoop 写，read() 读）
  std::mutex rx_mutex_;
  double     rx_joint_rad_[6]{};
  bool       rx_data_valid_{false};

  // Btn=17 检测（仅 readLoop 单线程访问，无需原子）
  uint8_t prev_button_{0};

  // Btn=17 抓拍位置（rad）：写=readLoop（上升沿），读=write()。
  // 同步靠 force_sync_until_ns_ 的 release-acquire 提供 happens-before；
  // write() 只在 force_sync_until_ns_ 还在未来才会读，此时 snapshot 已稳定写入。
  double snapshot_pos_[6]{};

  // force-sync 窗口截止时间（steady_clock ns since epoch）。窗口内 write() 强制
  // hw_commands_ = snapshot_pos_，覆盖 JTC 输出，避免 RX 抖动正反馈。
  std::atomic<int64_t> force_sync_until_ns_{0};

  // ros2_control 接口数据
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  // 内部 ROS node（用于发布摇杆 topic + commanded joint states）
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr recv_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_state_pub_;
};

}  // namespace tromac_hardware

PLUGINLIB_EXPORT_CLASS(
  tromac_hardware::TRoMaCHardwareInterface,
  hardware_interface::SystemInterface)
