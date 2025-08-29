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
    trajectory = get_package_share_directory('robomaster_examples')+'/waypoints/'+LaunchConfiguration('trajectory').perform(context)
    frequency = 20    
    # Takeoff all the drones before executing the trajectory
    # idx is used to identify the robot in the trajectory file
    # takeoff_height is WIP, used to set the height of the robot at takeoff
    # takeoff_time is WIP, used to delay the takeoff of the robot
    robots = {
        # <name>: {'idx': <idx in yml>, 'tf_frame': <tf_frame>, 'takeoff_height': 0.0, 'takeoff_time': 0.0}
        "rmtt_1": {'idx': 1,'tf_frame': 'rmtt_1','takeoff_height': 0.0,'takeoff_time': 0.0},
        "rmtt_2": {'idx': 2, 'tf_frame': 'rmtt_2','takeoff_height': 0.0,'takeoff_time': 0.0},
        "rmtt_3": {'idx': 3, 'tf_frame': 'rmtt_3','takeoff_height': 0.0,'takeoff_time': 0.0},
        }
    
    for robot,val in robots.items():
        node = Node(
            package='robomaster_examples',
            executable='execute_trajectory',
            name=robot,
            parameters=[{'trajectory':trajectory, 'world_frame': world_frame, 'frequency': frequency,
                        'idx': val['idx'], 'tf_frame': val['tf_frame'], 'robot_name': robot, 
                        'takeoff_height': val['takeoff_height'], 'takeoff_time': val['takeoff_time']}],
            output='screen',
        )
        ld.append(node)

    return ld

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('world_frame', default_value='world', description='tf world frame'),
        DeclareLaunchArgument('trajectory', default_value='multi_drone_spiral.yml', description='Trajectory file'),
        OpaqueFunction(function=node_fn)
    ])
