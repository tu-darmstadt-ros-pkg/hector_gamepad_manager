cmake_minimum_required(VERSION 3.15)
project(hector_gamepad_manager_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/GamepadAction.msg"
  "msg/GamepadButtonMapping.msg"
  "msg/GamepadAxisMapping.msg"
  "msg/GamepadConfigSwitch.msg"
  "msg/GamepadConfigMapping.msg"
  "msg/GamepadMapping.msg"
  "srv/SetTargetRobot.srv")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
