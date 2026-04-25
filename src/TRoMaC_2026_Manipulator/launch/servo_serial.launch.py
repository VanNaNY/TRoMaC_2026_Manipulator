from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, relative_path):
    package_path = Path(get_package_share_directory(package_name))
    with open(package_path / relative_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    package_name = "TRoMaC_2026_Manipulator"
    package_path = Path(get_package_share_directory(package_name))

    moveit_config = (
        MoveItConfigsBuilder(
            "机械臂-总装-新-4.SLDASM",
            package_name=package_name,
        ).to_moveit_configs()
    )

    servo_params = {
        "moveit_servo": load_yaml(package_name, "config/servo_settings.yaml"),
    }

    # Brings up move_group, robot_state_publisher, ros2_control, and optionally RViz
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_path / "launch" / "demo.launch.py")),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
            "db": LaunchConfiguration("db"),
        }.items(),
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Servo controller: subscribes /serial_recv (from hardware interface) and drives MoveIt Servo.
    # Serial communication (TX/RX) is handled by TRoMaCHardwareInterface (ros2_control plugin).
    serial_comm_node = Node(
        package=package_name,
        executable="serial_comm_node",
        name="serial_comm_node",
        output="screen",
        parameters=[
            {
                "enable_servo_control":  True,
                "planning_frame":        "base_link",
                "joy_deadzone_xyz":      30,
                "joy_deadzone_angular":  3,
                "joy_max_linear":        1.0,
                "joy_max_angular":       1.0,
            },
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz",      default_value="true"),
        DeclareLaunchArgument("db",            default_value="false"),
        DeclareLaunchArgument("serial_device", default_value="/dev/ttyUSB0",
                              description="Serial port of the lower computer"),
        demo_launch,
        servo_node,
        serial_comm_node,
    ])
