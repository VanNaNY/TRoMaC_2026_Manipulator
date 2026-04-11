#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "Serial.h"

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>
#include <sys/select.h>

class SerialCommNode : public rclcpp::Node
{
public:
  SerialCommNode() : Node("serial_comm_node")
  {
    declare_parameter("device", "/dev/ttyACM0");
    declare_parameter("baud_rate", 921600);
    declare_parameter("send_once_on_start", false);
    declare_parameter("auto_send", true);
    declare_parameter("send_rate_hz", 100.0);
    declare_parameter("tx_joints", std::vector<double>{10.0, 20.0, 30.0, 40.0, 50.0, 60.0});
    declare_parameter("log_auto_send", true);

    auto device = get_parameter("device").as_string();
    auto baud = static_cast<int>(get_parameter("baud_rate").as_int());

    if (!uart_.Open(device, baud))
    {
      RCLCPP_FATAL(get_logger(), "无法打开串口: %s @ %d", device.c_str(), baud);
      throw std::runtime_error("Serial port open failed");
    }
    RCLCPP_INFO(get_logger(), "串口已打开: %s @ %d baud", device.c_str(), baud);

    running_ = true;
    read_thread_ = std::thread(&SerialCommNode::readLoop, this);

    send_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "serial_send", 12,
        std::bind(&SerialCommNode::sendCallback, this, std::placeholders::_1));

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
      RCLCPP_INFO(get_logger(), " %.1f Hz", hz);
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "设 auto_send:=true，或向话题 serial_send 发 Float64MultiArray(6个数)");
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
    auto tx = get_parameter("tx_joints").as_double_array();
    if (tx.size() < 6) return;
    sendFromDoubles(tx.data());
    if (get_parameter("log_auto_send").as_bool())
    {
      RCLCPP_INFO(get_logger(),
                  "auto_send: %d,%d,%d,%d,%d,%d",
                  static_cast<int>(tx[0]), static_cast<int>(tx[1]),
                  static_cast<int>(tx[2]), static_cast<int>(tx[3]),
                  static_cast<int>(tx[4]), static_cast<int>(tx[5]));
    }
  }

  void readLoop()
  {
    while (running_)
    {
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(uart_.serial_id, &fds);
      struct timeval tv{};
      tv.tv_sec = 0;
      tv.tv_usec = 300000;

      int ret = select(uart_.serial_id + 1, &fds, nullptr, nullptr, &tv);
      if (ret > 0 && FD_ISSET(uart_.serial_id, &fds))
      {
        while (running_ && uart_.ReadData())
        {
          std::lock_guard<std::mutex> lock(rx_mutex_);
          latest_rx_ = uart_.read_data;
          rx_count_++;
          RCLCPP_INFO(
              get_logger(),
              "RX [#%lu] Arm_Pos=(%d, %d, %d)  Pitch=%d  Roll=%d  Button=%u  EndFrame=%u",
              rx_count_,
              latest_rx_.Arm_Pos_x,
              latest_rx_.Arm_Pos_y,
              latest_rx_.Arm_Pos_z,
              latest_rx_.Arm_Pos_Pitch,
              latest_rx_.Arm_Pos_Roll,
              latest_rx_.Button,
              latest_rx_.EndFrame);
        }
      }
    }
  }

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

  TRoMaC::Uart uart_;

  std::thread read_thread_;
  std::atomic<bool> running_{false};

  std::mutex rx_mutex_;
  TRoMaC::VisionFrameRX_structTypedef latest_rx_{};
  uint64_t rx_count_{0};

  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr send_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<SerialCommNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("serial_comm_node"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
