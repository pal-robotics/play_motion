import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    cfg = os.path.join(
        get_package_share_directory('play_motion'),
        'test', 'play_motion_cfg.yaml')

    play_motion = Node(package='play_motion',
                       executable='play_motion',
                       output='both',
                       parameters=[cfg])

    return LaunchDescription([play_motion])
