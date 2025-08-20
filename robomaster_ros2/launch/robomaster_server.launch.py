PKG_NAME = 'robomaster_ros2'
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import yaml



def node_fn(context, *args, **kwargs):
    ld = []
    server_conf = None
    num_of_drones = LaunchConfiguration('num_of_drones')
    num_of_eps1 = LaunchConfiguration('num_of_eps1')
    local_ip = LaunchConfiguration('local_ip')
    random_assign = LaunchConfiguration('random_assign')
    keyboard_cmd = LaunchConfiguration('keyboard_cmd')
    if str(num_of_drones.perform(context)) == '0' and str(num_of_eps1.perform(context)) == '0':
        server_conf = os.path.join(get_package_share_directory(PKG_NAME), 'config_ros', 'robomaster_server.yaml')
        with open(server_conf, 'r') as f:
            params_yaml = yaml.safe_load(f)
            num_of_drones = TextSubstitution(text=str(params_yaml['robomaster_server']['ros__parameters']['num_of_drones']))
            num_of_eps1 = TextSubstitution(text=str(params_yaml['robomaster_server']['ros__parameters']['num_of_eps1']))
            local_ip = TextSubstitution(text=str(params_yaml['robomaster_server']['ros__parameters']['local_ip']))
            random_assign = TextSubstitution(text=str(params_yaml['robomaster_server']['ros__parameters']['random_assign']))
            f.close()
    else:
        server_conf = {
            'num_of_drones': num_of_drones,
            'num_of_eps1': num_of_eps1,
            'local_ip': local_ip,
            'random_assign': random_assign,
            'keyboard_cmd': keyboard_cmd,
        }
    print('num_of_drones:', num_of_drones)
    print('num_of_eps1:', num_of_eps1)   
    param_server = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory(PKG_NAME), 'launch'), '/param_server.launch.py']),
         launch_arguments={'num_of_drones': num_of_drones, 'num_of_eps1': num_of_eps1}.items())
        
    ld.append(param_server)

    node = Node(
        package='robomaster_ros2',
        executable='robomaster_server',
        name='robomaster_server',
        parameters=[server_conf],
        output='screen',
    )
    ld.append(node)


    return ld

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'num_of_drones',
            default_value='0',
            description='Number of drones to be simulated',
        ),
        DeclareLaunchArgument(
            'num_of_eps1',
            default_value='0',
            description='Number of eps1 to be simulated'
        ),
        DeclareLaunchArgument(
            'local_ip',
            default_value='192.168.0.106',
            description='Local IP address of the computer running the simulation'
        ),
        DeclareLaunchArgument(
            'random_assign',
            default_value='true',
            description='Random assignment for drone/robot allocation'
        ),
        DeclareLaunchArgument(
            'keyboard_cmd',
            default_value='true',
            description='Enable keyboard command control for drones'
        ),
        OpaqueFunction(function=node_fn)
    ])