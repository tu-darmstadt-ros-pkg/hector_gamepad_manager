cmake_minimum_required(VERSION 3.10)
project(fake_joy_publisher)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})


install(FILES
        resource/${PROJECT_NAME}
        DESTINATION share/ament_index/resource_index/packages
)



# ---------------------------
# Testing (pytest via ament)
# ---------------------------
if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)
    # Add your test file (can be unit or rostest/launch-test as you like)
    add_launch_test(test/test_fake_joy_publisher.py TIMEOUT 60)
endif()

ament_package()
