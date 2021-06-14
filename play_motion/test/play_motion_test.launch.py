import os
import unittest

import launch_testing

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_test_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('rviz',
                              default_value="False",
                              description='Whether run rviz',
                              choices=["True", "False"]))

    rrbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('play_motion'),
                         'test', 'rrbot.launch.py')))

    play_motion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('play_motion'),
                         'test', 'play_motion.launch.py')))

    play_motion_test = launch_testing.actions.GTest(
        path=os.path.join(get_package_share_directory('play_motion'),
                          'test', 'play_motion_test'),
        output='screen')

    return LaunchDescription(declared_arguments + [
        rrbot,
        play_motion,
        play_motion_test,

        launch_testing.actions.ReadyToTest(),
    ]), {'play_motion_test': play_motion_test}


class TestPlayMotion(unittest.TestCase):

    def test_wait(self, play_motion_test, proc_info):
        proc_info.assertWaitForShutdown(process=play_motion_test, timeout=(60))


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, play_motion_test, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=play_motion_test)
