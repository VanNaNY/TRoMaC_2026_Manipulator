#!/usr/bin/env python3
"""Publish URDF XML on /robot_description so RViz RobotModel (and joint_state_publisher) can subscribe."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotDescriptionPub(Node):
    def __init__(self):
        super().__init__("robot_description_pub")
        self.declare_parameter("urdf_file", "")
        path = self.get_parameter("urdf_file").get_parameter_value().string_value
        if not path:
            raise RuntimeError("Parameter urdf_file is required")
        with open(path, "r", encoding="utf-8") as f:
            self._xml = f.read()

        # Default QoS matches typical RViz / joint_state_publisher_gui subscribers
        self._pub = self.create_publisher(String, "/robot_description", 10)
        self._timer = self.create_timer(0.5, self._publish)
        self.get_logger().info("Publishing /robot_description at 2 Hz")

    def _publish(self):
        msg = String()
        msg.data = self._xml
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = RobotDescriptionPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
