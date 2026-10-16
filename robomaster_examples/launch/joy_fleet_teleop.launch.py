from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_names',
            default_value='rmep_1',
            description='Comma-separated EP/S1 robot names (example: rmep_1,rmep_2)',
        ),
        DeclareLaunchArgument(
            'drone_names',
            default_value='rmtt_1',
            description='Comma-separated drone names (example: rmtt_1,rmtt_2)',
        ),
        DeclareLaunchArgument(
            'start_mode',
            default_value='drone',
            description='Initial control mode: robot or drone',
        ),
        DeclareLaunchArgument(
            'takeoff_height',
            default_value='1.0',
            description='Takeoff height (m) used when buttons[2] is pressed in drone mode',
        ),
        DeclareLaunchArgument(
            'require_deadman',
            default_value='true',
            description='Require holding buttons[4] for motion commands',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='robomaster_examples',
            executable='joy_fleet_teleop',
            name='joy_fleet_teleop',
            output='screen',
            parameters=[{
                'robot_names_csv': LaunchConfiguration('robot_names'),
                'drone_names_csv': LaunchConfiguration('drone_names'),
                'start_mode': LaunchConfiguration('start_mode'),
                'takeoff_height': LaunchConfiguration('takeoff_height'),
                'require_deadman': LaunchConfiguration('require_deadman'),
            }],
        ),
    ])
