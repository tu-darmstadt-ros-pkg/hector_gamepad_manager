cmake_minimum_required(VERSION 3.15)
project(hector_gamepad_manager_rqml_plugins)

find_package(ament_cmake REQUIRED)

install(DIRECTORY qml DESTINATION share/${PROJECT_NAME})

ament_index_register_resource(rqml_plugin CONTENT_FILE rqml_plugins.yaml.in)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Tests the ROS-free QML components, which need no Ros2 module.
  find_package(Qt6 QUIET COMPONENTS QuickTest)
  if(Qt6QuickTest_FOUND)
    add_executable(test_virtual_gamepad test/test_qml.cpp)
    target_compile_definitions(
      test_virtual_gamepad
      PRIVATE QUICK_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")
    target_link_libraries(test_virtual_gamepad Qt6::QuickTest)

    set(TEST_RESULT_DIR "${CMAKE_BINARY_DIR}/test_results/${PROJECT_NAME}")
    ament_add_test(
      test_virtual_gamepad
      COMMAND
      $<TARGET_FILE:test_virtual_gamepad>
      -platform
      offscreen
      -o
      "${TEST_RESULT_DIR}/test_virtual_gamepad.xml,junitxml"
      OUTPUT_FILE
      "${TEST_RESULT_DIR}/test_virtual_gamepad.txt"
      RESULT_FILE
      "${TEST_RESULT_DIR}/test_virtual_gamepad.xml")
  else()
    message(
      STATUS "Qt6 QuickTest not found, skipping VirtualGamepadState tests")
  endif()
endif()

ament_package()
