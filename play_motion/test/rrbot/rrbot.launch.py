import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_pal.include_utils import include_launch_py_description
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('rviz',
                              default_value="False",
                              description='Whether run rviz',
                              choices=["True", "False"]))

    rviz = LaunchConfiguration('rviz')

    robot_description_path = os.path.join(
        get_package_share_directory('play_motion'), 'test', 'rrbot.xacro')
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    rrbot_controllers = os.path.join(
        get_package_share_directory('play_motion'),
        'test', 'controller_manager.yaml')

    rviz_config_file = os.path.join(
        get_package_share_directory('play_motion'), 'test', 'rrbot.rviz')

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, rrbot_controllers],
        output={'stdout': 'screen', 'stderr': 'screen'})

    controllers = include_launch_py_description(
        'play_motion', ['test', 'controllers.launch.py'])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output={'stdout': 'screen', 'stderr': 'log'},
        arguments=['-d', rviz_config_file],
        condition=IfCondition(rviz)
    )

    launch_description = [control_node,
                          controllers,
                          robot_state_publisher_node,
                          rviz_node]

    return LaunchDescription(
        declared_arguments + launch_description)
