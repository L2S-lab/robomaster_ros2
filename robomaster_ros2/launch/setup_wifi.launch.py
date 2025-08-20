import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
      get_package_share_directory('robomaster_ros2'),
      'config_ros',
      'setup_wifi.yaml'
      )

    return LaunchDescription([
        Node(
            package='robomaster_ros2',
            executable='setup_wifi',
            name='setup_wifi',
            parameters=[config]
        ),
    ])