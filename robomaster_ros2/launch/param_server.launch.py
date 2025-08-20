import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,Shutdown
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    rmtt= os.path.join(get_package_share_directory('robomaster_ros2'), 'config_ros', 'rmtt_param.yaml')
    rmep = os.path.join(get_package_share_directory('robomaster_ros2'), 'config_ros', 'rmep_param.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('num_of_drones', default_value=TextSubstitution(text='1')),
        DeclareLaunchArgument('num_of_eps1', default_value=TextSubstitution(text='1')),
        Node(
            package='robomaster_ros2',
            executable='param_server',
            name='param_server',
            parameters=[rmtt, rmep,{'num_of_drones': LaunchConfiguration('num_of_drones'), 'num_of_eps1': LaunchConfiguration('num_of_eps1')}],
            output='screen',
            on_exit=Shutdown(),
        ),
    ])