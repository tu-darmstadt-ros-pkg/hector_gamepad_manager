cmake_minimum_required(VERSION 3.16)
project(hector_gamepad_manager_plugins LANGUAGES CXX)

# Default to C++17 if(NOT CMAKE_CXX_STANDARD) set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON) endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
# add_compile_options(-Wall -Wextra -Wpedantic) endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gen_version_h REQUIRED)

# Define a list of dependencies in alphabetical order
set(DEPENDENCIES
    controller_manager_msgs
    controller_orchestrator
    geometry_msgs
    hector_gamepad_plugin_interface
    hector_ros2_utils
    moveit_msgs
    pluginlib
    ros_babel_fish
    rcl_interfaces
    rclcpp
    rclcpp_action
    sensor_msgs
    srdfdom
    std_srvs)

foreach(dependency IN LISTS DEPENDENCIES)
  find_package(${dependency} REQUIRED)
endforeach()
find_package(backward_ros REQUIRED)
set(HEADERS
    include/hector_gamepad_manager_plugins/controller_helper.hpp
    include/hector_gamepad_manager_plugins/drive_plugin.hpp
    include/hector_gamepad_manager_plugins/battery_monitor_plugin.hpp
    include/hector_gamepad_manager_plugins/flipper_plugin.hpp
    include/hector_gamepad_manager_plugins/manipulation_plugin.hpp
    include/hector_gamepad_manager_plugins/moveit_plugin.hpp
    include/hector_gamepad_manager_plugins/blackboard_plugin.hpp)

set(SOURCES
    src/controller_helper.cpp
    src/drive_plugin.cpp
    src/flipper_plugin.cpp
    src/battery_monitor_plugin.cpp
    src/manipulation_plugin.cpp
    src/moveit_plugin.cpp
    src/blackboard_plugin.cpp)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})
# target_link_libraries(${PROJECT_NAME} PRIVATE
# controller_orchestrator::controller_orchestrator)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)

# Install the library to the plugins directory
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

# Install the plugin XML
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(hector_gamepad_manager plugins.xml)

#[[# Linting and tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()]]

ament_export_targets(${PROJECT_NAME}-targets)
ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
