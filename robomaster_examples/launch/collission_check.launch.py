from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

PKG_NAME = 'robomaster_examples'

def node_fn(context, *args, **kwargs):
    ld = []
    trajectory = get_package_share_directory('robomaster_examples')+'/waypoints/'+LaunchConfiguration('trajectory').perform(context)
    nb_drones = LaunchConfiguration('nb_drones')
    safe_distance = LaunchConfiguration('safe_distance')
    quality = LaunchConfiguration('quality')
    
    node = Node(
        package='robomaster_examples',
        executable='collission_check',
        name='collission_check',
        parameters=[{'trajectory':trajectory, 'nb_drones': nb_drones, 'safe_distance': safe_distance, 'quality': quality}],
        output='screen',
    )
    ld.append(node)

    return ld

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('trajectory', default_value='multi_drone_spiral.yml', description='Trajectory file YAML or CSV'),
        DeclareLaunchArgument('nb_drones', default_value='2', description='Number of drones'),
        DeclareLaunchArgument('safe_distance', default_value='0.2', description='Safe distance between drones'),
        DeclareLaunchArgument('quality', default_value='50', description='Quality of trajectory for collision checking'),
        OpaqueFunction(function=node_fn)
    ])
