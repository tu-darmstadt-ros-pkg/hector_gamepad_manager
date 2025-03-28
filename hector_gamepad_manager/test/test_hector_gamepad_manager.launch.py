import os
import yaml
import unittest
import pytest

import launch_ros
import launch
import launch_testing.actions
import launch_testing.asserts

from launch import LaunchDescription
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


# see https://github.com/tylerjw/moveit2/blob/115bfde368e40907f858c43c4e8da4f8c6997d54/moveit_ros/moveit_servo/test/launch/test_servo_pose_tracking.test.py
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_hector_gamepad_manager_test_description(
    *args, gtest_name: SomeSubstitutionsType
):
    config = os.path.join(
        os.path.dirname(__file__), "config", "athena_plugin_config.yaml"
    )
    hector_gamepad_manager = Node(
        package="hector_gamepad_manager",
        name="hector_gamepad_manager",
        executable="hector_gamepad_manager_node",
        parameters=[config],
    )
    hector_gamepad_manager_gtest = launch_ros.actions.Node(
        executable=PathJoinSubstitution(
            [LaunchConfiguration("test_binary_dir"), gtest_name]
        ),
        output="screen",
        # prefix=['xterm -e gdb --args']
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package "
                "containing test executables",
            ),
            hector_gamepad_manager,
            hector_gamepad_manager_gtest,
            launch_testing.actions.ReadyToTest(),
        ]
    ), {"hector_gamepad_manager_gtest": hector_gamepad_manager_gtest}


def generate_test_description():
    return generate_hector_gamepad_manager_test_description(
        gtest_name="hector_gamepad_manager_test"
    )


class TestGTestProcessActive(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, hector_gamepad_manager_gtest):
        proc_info.assertWaitForShutdown(hector_gamepad_manager_gtest, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    def test_gtest_pass(self, proc_info, hector_gamepad_manager_gtest):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=hector_gamepad_manager_gtest
        )
