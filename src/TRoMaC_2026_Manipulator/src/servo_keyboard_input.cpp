#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <cstring>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <fcntl.h>

// Arrow key codes (3rd byte of \x1b[X escape sequence, also maps to uppercase A-D
// which we don't use as commands, so there is no practical conflict)
constexpr char KEYCODE_UP = 0x41;
constexpr char KEYCODE_DOWN = 0x42;
constexpr char KEYCODE_RIGHT = 0x43;
constexpr char KEYCODE_LEFT = 0x44;

// ---------- robot-specific constants ----------

static const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
static const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
static const std::string START_SERVO_SERVICE = "/servo_node/start_servo";

static const std::string BASE_FRAME_ID = "base_link";
static const std::string EEF_FRAME_ID = "6Link";

static const std::vector<std::string> JOINT_NAMES = {
    "1-joint", "2-joint", "3-joint",
    "4-joint", "5-joint", "6-joint"};

static constexpr size_t ROS_QUEUE_SIZE = 10;

// ===========================================================================
// KeyboardReader
//   Opens a TTY in *cbreak* mode (ICANON|ECHO off, ISIG stays on).
//   Falls back to /dev/tty when stdin is not a terminal (ros2 launch case).
//   Keeping ISIG means Ctrl+C still generates SIGINT.
// ===========================================================================
class KeyboardReader
{
public:
  KeyboardReader() : fd_(-1), owns_fd_(false)
  {
    if (isatty(STDIN_FILENO))
    {
      fd_ = STDIN_FILENO;
    }
    else
    {
      fd_ = open("/dev/tty", O_RDONLY);
      if (fd_ < 0)
      {
        throw std::runtime_error(
            "No interactive terminal available for keyboard teleop. "
            "Launch with start_keyboard:=false and run the keyboard node "
            "separately in your own shell.");
      }
      owns_fd_ = true;
    }

    tcgetattr(fd_, &original_);
    struct termios cbreak{};
    std::memcpy(&cbreak, &original_, sizeof(struct termios));
    cbreak.c_lflag &= ~(ICANON | ECHO);  // cbreak, NOT raw
    cbreak.c_cc[VEOL] = 1;
    cbreak.c_cc[VEOF] = 2;
    tcsetattr(fd_, TCSANOW, &cbreak);
  }

  void readOne(char* c)
  {
    int rc = read(fd_, c, 1);
    if (rc < 0)
      throw std::runtime_error("read failed");
  }

  void shutdown()
  {
    if (fd_ >= 0)
    {
      tcsetattr(fd_, TCSANOW, &original_);
      if (owns_fd_)
        close(fd_);
      fd_ = -1;
    }
  }

private:
  int fd_;
  bool owns_fd_;
  struct termios original_;
};

static std::unique_ptr<KeyboardReader> g_input;

static void signalHandler(int /*sig*/)
{
  if (g_input)
    g_input->shutdown();
  rclcpp::shutdown();
  _exit(0);
}

// ===========================================================================
// callStartServo – waits for the servo start service and triggers it
// ===========================================================================
static bool callStartServo()
{
  auto node = rclcpp::Node::make_shared("servo_keyboard_start_client");
  auto client = node->create_client<std_srvs::srv::Trigger>(START_SERVO_SERVICE);

  if (!client->wait_for_service(std::chrono::seconds(15)))
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Service %s not available. Is servo_node running?",
                 START_SERVO_SERVICE.c_str());
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(30)) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to call %s (timeout)",
                 START_SERVO_SERVICE.c_str());
    return false;
  }

  if (!future.get()->success)
  {
    RCLCPP_ERROR(node->get_logger(), "%s returned success=false",
                 START_SERVO_SERVICE.c_str());
    return false;
  }

  RCLCPP_INFO(node->get_logger(),
              "start_servo OK; waiting briefly for Servo to initialise...");
  std::this_thread::sleep_for(std::chrono::milliseconds(800));
  return true;
}

// ===========================================================================
// KeyboardServo – reads keyboard, publishes Twist / JointJog commands
// ===========================================================================
class KeyboardServo
{
public:
  KeyboardServo();
  int keyLoop();

private:
  void spin();
  static void printHelp();

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;

  std::string command_frame_;
  double joint_vel_cmd_;
  double linear_scale_;
  double angular_scale_;
};

KeyboardServo::KeyboardServo()
  : command_frame_(BASE_FRAME_ID),
    joint_vel_cmd_(1.0),
    linear_scale_(1.0),
    angular_scale_(1.0)
{
  nh_ = rclcpp::Node::make_shared("servo_keyboard_input");
  twist_pub_ =
      nh_->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
  joint_pub_ =
      nh_->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);
}

void KeyboardServo::spin()
{
  while (rclcpp::ok())
    rclcpp::spin_some(nh_);
}

void KeyboardServo::printHelp()
{
  puts("");
  puts("TRoMaC Manipulator – Keyboard Servo");
  puts("====================================");
  puts("Cartesian jog (末端平移/旋转):");
  puts("  Arrow Up / Down      +X / -X");
  puts("  Arrow Left / Right   -Y / +Y");
  puts("  i / k   (or ; / .)  +Z / -Z");
  puts("  u / o                +Rx / -Rx");
  puts("  j / l                +Ry / -Ry");
  puts("  n / m                +Rz / -Rz");
  puts("");
  puts("Joint jog (单关节):");
  puts("  1-6                  jog joint 1-6");
  puts("  r                    reverse direction");
  puts("");
  puts("Frame & scale:");
  puts("  b / w                base_link frame");
  puts("  t / e                end-effector frame");
  puts("  + / =                increase step");
  puts("  -                    decrease step");
  puts("");
  puts("  h  help   q  quit");
  puts("");
}

int KeyboardServo::keyLoop()
{
  setvbuf(stdout, nullptr, _IOLBF, 0);

  char c;
  bool publish_twist = false;
  bool publish_joint = false;

  std::thread{std::bind(&KeyboardServo::spin, this)}.detach();

  printHelp();
  printf("[frame: %s | linear: %.2f | angular: %.2f | joint_vel: %.2f]\n\n",
         command_frame_.c_str(), linear_scale_, angular_scale_,
         std::abs(joint_vel_cmd_));

  for (;;)
  {
    try
    {
      g_input->readOne(&c);
    }
    catch (const std::runtime_error&)
    {
      perror("read()");
      return -1;
    }

    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    switch (c)
    {
      // ---- Cartesian linear ----
      case KEYCODE_UP:
        twist_msg->twist.linear.x = linear_scale_;
        publish_twist = true;
        break;
      case KEYCODE_DOWN:
        twist_msg->twist.linear.x = -linear_scale_;
        publish_twist = true;
        break;
      case KEYCODE_LEFT:
        twist_msg->twist.linear.y = -linear_scale_;
        publish_twist = true;
        break;
      case KEYCODE_RIGHT:
        twist_msg->twist.linear.y = linear_scale_;
        publish_twist = true;
        break;
      case 'i':
      case ';':
        twist_msg->twist.linear.z = linear_scale_;
        publish_twist = true;
        break;
      case 'k':
      case '.':
        twist_msg->twist.linear.z = -linear_scale_;
        publish_twist = true;
        break;

      // ---- Cartesian angular ----
      case 'u':
        twist_msg->twist.angular.x = angular_scale_;
        publish_twist = true;
        break;
      case 'o':
        twist_msg->twist.angular.x = -angular_scale_;
        publish_twist = true;
        break;
      case 'j':
        twist_msg->twist.angular.y = angular_scale_;
        publish_twist = true;
        break;
      case 'l':
        twist_msg->twist.angular.y = -angular_scale_;
        publish_twist = true;
        break;
      case 'n':
        twist_msg->twist.angular.z = angular_scale_;
        publish_twist = true;
        break;
      case 'm':
        twist_msg->twist.angular.z = -angular_scale_;
        publish_twist = true;
        break;

      // ---- Joint jog (keys 1-6 → joints 0-5) ----
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      {
        size_t idx = static_cast<size_t>(c - '1');
        joint_msg->joint_names.push_back(JOINT_NAMES[idx]);
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      }
      case 'r':
        joint_vel_cmd_ *= -1;
        printf("[joint direction reversed → %.2f]\n", joint_vel_cmd_);
        break;

      // ---- Frame selection ----
      case 'b':
      case 'w':
        command_frame_ = BASE_FRAME_ID;
        printf("[frame: %s]\n", command_frame_.c_str());
        break;
      case 't':
      case 'e':
        command_frame_ = EEF_FRAME_ID;
        printf("[frame: %s]\n", command_frame_.c_str());
        break;

      // ---- Scale adjustment ----
      case '+':
      case '=':
        linear_scale_ = std::min(linear_scale_ * 1.2, 3.0);
        angular_scale_ = std::min(angular_scale_ * 1.2, 3.0);
        printf("[linear: %.2f | angular: %.2f]\n", linear_scale_, angular_scale_);
        break;
      case '-':
        linear_scale_ = std::max(linear_scale_ / 1.2, 0.1);
        angular_scale_ = std::max(angular_scale_ / 1.2, 0.1);
        printf("[linear: %.2f | angular: %.2f]\n", linear_scale_, angular_scale_);
        break;

      // ---- Help & quit ----
      case 'h':
        printHelp();
        break;
      case 'q':
        return 0;
    }

    if (publish_twist)
    {
      twist_msg->header.stamp = nh_->now();
      twist_msg->header.frame_id = command_frame_;
      twist_pub_->publish(std::move(twist_msg));
      publish_twist = false;
    }
    else if (publish_joint)
    {
      joint_msg->header.stamp = nh_->now();
      joint_msg->header.frame_id = BASE_FRAME_ID;
      joint_pub_->publish(std::move(joint_msg));
      publish_joint = false;
    }
  }

  return 0;
}

// ===========================================================================
// main
// ===========================================================================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  try
  {
    g_input = std::make_unique<KeyboardReader>();
  }
  catch (const std::runtime_error& e)
  {
    fprintf(stderr, "Error: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }

  if (!callStartServo())
  {
    g_input->shutdown();
    rclcpp::shutdown();
    return 1;
  }

  KeyboardServo keyboard_servo;
  signal(SIGINT, signalHandler);

  int rc = keyboard_servo.keyLoop();
  g_input->shutdown();
  rclcpp::shutdown();

  return rc;
}
