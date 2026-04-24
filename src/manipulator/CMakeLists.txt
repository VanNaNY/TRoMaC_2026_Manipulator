cmake_minimum_required(VERSION 3.22)
project(manipulator)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/robot_description_pub.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME robot_description_pub
)

ament_package()