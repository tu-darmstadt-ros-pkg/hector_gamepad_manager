cmake_minimum_required(VERSION 3.8)
project(hector_gamepad_manager)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)

set(HEADERS
        include/hector_gamepad_manager/gamepad_function_plugin.hpp
        include/hector_gamepad_manager/hector_gamepad_manager.hpp
)

set(SOURCES
        src/hector_gamepad_manager.cpp
)
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)

ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp sensor_msgs pluginlib yaml-cpp)

add_executable(hector_gamepad_manager_node src/hector_gamepad_manager_node.cpp)
target_link_libraries(hector_gamepad_manager_node PUBLIC ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

install(TARGETS hector_gamepad_manager_node DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_export_targets(${PROJECT_NAME}-targets)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs pluginlib yaml-cpp)

ament_package()
