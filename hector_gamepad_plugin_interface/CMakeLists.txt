cmake_minimum_required(VERSION 3.8)
project(hector_gamepad_plugin_interface)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Header-only interface target
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(
        ${PROJECT_NAME}
        INTERFACE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT "export_${PROJECT_NAME}"
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib
        INCLUDES DESTINATION include
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

ament_export_targets("export_${PROJECT_NAME}")

# --------------------------
# Tests
# --------------------------
include(CTest)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # test/test_blackboard.cpp should include:
  #   #include <hector_gamepad_plugin_interface/blackboard.hpp>
  ament_add_gtest(test_blackboard test/test_blackboard.cpp)
  if(TARGET test_blackboard)
    target_link_libraries(test_blackboard ${PROJECT_NAME})
    target_compile_features(test_blackboard PRIVATE cxx_std_17)
    target_compile_options(test_blackboard PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

ament_package()
