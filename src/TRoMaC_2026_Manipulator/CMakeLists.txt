cmake_minimum_required(VERSION 3.22)
project(TRoMaC_2026_Manipulator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_manager_msgs REQUIRED)

add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
ament_target_dependencies(servo_keyboard_input
  rclcpp geometry_msgs control_msgs std_srvs)

add_executable(serial_comm_node src/serial_comm_node.cpp)
ament_target_dependencies(serial_comm_node rclcpp std_msgs geometry_msgs control_msgs std_srvs sensor_msgs moveit_core moveit_ros_planning_interface controller_manager_msgs Eigen3)

# Hardware interface plugin (shared library)
add_library(tromac_hardware_interface SHARED
  src/tromac_hardware_interface.cpp
  src/Serial.cpp)
ament_target_dependencies(tromac_hardware_interface
  rclcpp rclcpp_lifecycle hardware_interface pluginlib std_msgs sensor_msgs)
pluginlib_export_plugin_description_file(hardware_interface tromac_hardware.xml)

install(TARGETS servo_keyboard_input serial_comm_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS tromac_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
