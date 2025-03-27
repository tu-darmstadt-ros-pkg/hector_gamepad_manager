cmake_minimum_required(VERSION 3.8)
project(hector_gamepad_manager_interface)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


add_library(hector_gamepad_manager_interface src/gamepad_plugin_interface.cpp)

target_include_directories(hector_gamepad_manager_interface PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(hector_gamepad_manager_interface
        rclcpp
        std_msgs
)

ament_export_include_directories(include)
install(TARGETS hector_gamepad_manager_interface
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY include/
        DESTINATION include
)


ament_package()
