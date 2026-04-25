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

    // 创建内部 ROS node 用于发布摇杆数据
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("tromac_hw_iface_internal");
    recv_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("serial_recv", 10);

    RCLCPP_INFO(rclcpp::get_logger("TRoMaCHardwareInterface"),
                "on_init: device=%s, baud=%d, log_serial=%s",
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
    RCLCPP_INFO(rclcpp::get_logger("TRoMaCHardwareInterface"), "硬件接口已激活");
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
    // 从后台 RX 线程获取最新关节角度
    std::lock_guard<std::mutex> lock(rx_mutex_);
    if (rx_data_valid_)
    {
      for (int i = 0; i < 6; ++i)
      {
        // 下位机度数 → ROS 弧度：度转弧度 + 零位偏移
        hw_positions_[i] = rx_joint_rad_[i];
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // command position (ROS 弧度) → 减零位偏移 → 转下位机度数 → 串口 TX
    TRoMaC::VisionData vd{};
    int16_t joints[6];
    //double degs[6];
    for (int i = 0; i < 6; ++i)
    {
      double deg = (hw_commands_[i] - JOINT_OFFSET_RAD[i]) * (180.0 / M_PI); //degs[i]
      joints[i] = static_cast<int16_t>(deg); //degs[i]
    }
    vd.Joint_1 = joints[0];
    vd.Joint_2 = joints[1];
    vd.Joint_3 = joints[2];
    vd.Joint_4 = joints[3];
    vd.Joint_5 = joints[4];
    vd.Joint_6 = joints[5];
    uart_.send(vd);

    if (log_serial_)
    {
      RCLCPP_INFO(rclcpp::get_logger("TRoMaCHardwareInterface"),
                  "TX: cmd_deg: %d %d %d %d %d %d",
                  joints[0], joints[1], joints[2],
                  joints[3], joints[4], joints[5]);
      /*
      RCLCPP_INFO(rclcpp::get_logger("TRoMaCHardwareInterface"),
                  "TX: cmd_deg: %.2f %.2f %.2f %.2f %.2f %.2f",
                  degs[0], degs[1], degs[2],
                  degs[3], degs[4], degs[5]);
      */
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
          {
            std::lock_guard<std::mutex> lock(rx_mutex_);
            rx_joint_rad_[0] = static_cast<double>(rx.Real_Joint_1) * (M_PI / 180.0) + JOINT_OFFSET_RAD[0];
            rx_joint_rad_[1] = static_cast<double>(rx.Real_Joint_2) * (M_PI / 180.0) + JOINT_OFFSET_RAD[1];
            rx_joint_rad_[2] = static_cast<double>(rx.Real_Joint_3) * (M_PI / 180.0) + JOINT_OFFSET_RAD[2];
            rx_joint_rad_[3] = static_cast<double>(rx.Real_Joint_4) * (M_PI / 180.0) + JOINT_OFFSET_RAD[3];
            rx_joint_rad_[4] = static_cast<double>(rx.Real_Joint_5) * (M_PI / 180.0) + JOINT_OFFSET_RAD[4];
            rx_joint_rad_[5] = static_cast<double>(rx.Real_Joint_6) * (M_PI / 180.0) + JOINT_OFFSET_RAD[5];
            rx_data_valid_ = true;
          }

          if (log_serial_)
          {
            RCLCPP_INFO(logger,
                        "RX  Joy: x=%d y=%d z=%d P=%d R=%d Btn=%u | "
                        "Real_J: %d %d %d %d %d %d",
                        rx.Arm_Pos_x, rx.Arm_Pos_y, rx.Arm_Pos_z,
                        rx.Arm_Pos_Pitch, rx.Arm_Pos_Roll, rx.Button,
                        rx.Real_Joint_1, rx.Real_Joint_2, rx.Real_Joint_3,
                        rx.Real_Joint_4, rx.Real_Joint_5, rx.Real_Joint_6);
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

  // ros2_control 接口数据
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  // 内部 ROS node（用于发布摇杆 topic）
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr recv_pub_;
};

}  // namespace tromac_hardware

PLUGINLIB_EXPORT_CLASS(
  tromac_hardware::TRoMaCHardwareInterface,
  hardware_interface::SystemInterface)
