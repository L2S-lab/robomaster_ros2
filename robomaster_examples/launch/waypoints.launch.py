import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

PKG_NAME = 'robomaster_examples'

def node_fn(context, *args, **kwargs):
    ld = []
    world_frame = LaunchConfiguration('world_frame')
    # takeoff all the drones before launch
    # takeoff_height is WIP, used to set the height of the drone at takeoff
    # takeoff_time is WIP, used to delay the takeoff of the drone
    robots = {
        # <name>: {'tf_frame': <tf_frame>, 'frequency': 20, 'takeoff_height': 0.0, 'time_to_takeoff': 0.0, 'waypoints': <.wps file for the drone>}
        "rmtt_1": {'tf_frame': 'rmtt_1', 'frequency': 20, 'takeoff_height': 0.0, 'time_to_takeoff': 0.0, 'waypoints': 'drone_1.wps'},
        "rmtt_2": {'tf_frame': 'rmtt_2', 'frequency': 20, 'takeoff_height': 0.0, 'time_to_takeoff': 0.0, 'waypoints': 'drone_2.wps'},
        }

    for robot,val in robots.items():
        node = Node(
            package='robomaster_examples',
            executable='waypoints',
            name=robot,
            parameters=[{'world_frame': world_frame, 'tf_frame': val['tf_frame'], 'frequency': val['frequency'], 
                        'robot_name': robot, 'takeoff_height': val['takeoff_height'], 'time_to_takeoff': val['time_to_takeoff'], 
                        'waypoints': val['waypoints']}],
            output='screen',
        )
        ld.append(node)
    return ld

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('world_frame', default_value='world', description='tf world frame'),
        OpaqueFunction(function=node_fn)
    ])