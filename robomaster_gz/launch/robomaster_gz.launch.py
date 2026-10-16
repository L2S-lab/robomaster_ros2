
# gz topic -t "/model/rmtt-1/joint/prop_1_joint/cmd_force" -m gz.msgs.Double -p "data: 3"
from pathlib import Path
from string import Template
from typing import List, Tuple
import ast
import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from rclpy.logging import get_logger

PKG_NAME = "robomaster_gz"

def node_fn(context,*args, **kwargs):
    logger = get_logger(PKG_NAME)
    nb_drones = int(LaunchConfiguration("nb_drones").perform(context))
    spawn_only = True if LaunchConfiguration("spawn_only").perform(context).lower() == "true" else False  
    world = LaunchConfiguration("world").perform(context)
    use_x_default = False
    use_y_default = False
    use_z_default = False
    drone_x = ast.literal_eval(LaunchConfiguration("drone_x").perform(context))
    if len(drone_x) < nb_drones:
        logger.warn("Number of x positions does not match the number of drones. Using default values.")
        use_x_default = True
        drone_x = [i for i in range(nb_drones)]
    drone_y = ast.literal_eval(LaunchConfiguration("drone_y").perform(context))
    if len(drone_y) < nb_drones:
        logger.warn("Number of y positions does not match the number of drones. Using default values.")
        use_y_default = True
        drone_y = [0 for i in range(nb_drones)]
    drone_z = ast.literal_eval(LaunchConfiguration("drone_z").perform(context))
    if len(drone_z) < nb_drones:
        logger.warn("Number of z positions does not match the number of drones. Using default values.")
        use_z_default = True

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_project_gazebo = get_package_share_directory(PKG_NAME)
    try:
        get_package_share_directory("sdformat_urdf")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "The SDF robot_description requires sdformat_urdf. "
            "Install ros-humble-sdformat-urdf and source the ROS environment before launching."
        ) from exc

    sdf_file = os.path.join(
            pkg_project_gazebo, "models", "rmtt", "rmtt.sdf"
        )
    
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()
        infp.close()
    
    ld = []
    bridge_topics = []
    rd_template = Template(robot_desc)

    # joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    ld.append(joint_state_publisher)

    # gz sim
    if spawn_only:
        cmd = "gz service -s /gazebo/worlds --reqtype gz.msgs.Empty --reptype gz.msgs.StringMsg_V --timeout 1000 --req ' '"
        stream = os.popen(cmd)
        output = stream.read()  # 'data: "empty"\n\n'
        exit_code = stream.close()
        if exit_code is not None:
            logger.error(f"Error while finding the world name: {output}")
        else:
            output = output.strip().split('"')[1] 
            logger.info(f"World available: {output}")
        if output != world:
            logger.error(f"World {world} not available. Please use the 'world' argument to specify the world to spawn the robots in.")
            raise RuntimeError(f"World {world} not available. Please use the 'world' argument to specify the world to spawn the robots in.")
        rd_template = Template(rd_template.template.replace(f"package://{PKG_NAME}",pkg_project_gazebo))
    else:
        gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': PathJoinSubstitution([
                pkg_project_gazebo,
                'worlds',
                f'{Path(pkg_project_gazebo) / "worlds" / f"{world}.sdf"}'
            ])}.items(),
        )
        ld.append(gz_sim)
        
    for i in range(1, nb_drones+1):
        x = drone_x[i-1] if not use_x_default else i-1
        y = 0 if use_y_default else drone_y[i-1]
        z = 1 if use_z_default else drone_z[i-1]
        
            
        # Spawn entity
        nodes, bridges = add_rmtt(i, x, y, z, rd_template, world)
        ld.extend(nodes)
        bridge_topics.extend(bridges)
        
    # Bridge.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                # "config_file": os.path.join(
                #     pkg_project_gazebo, "config", "demo.yaml"
                # ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        arguments=bridge_topics,
        output="screen",
    )
    ld.append(bridge)

    # RViz.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            #"-d",
            #f'{Path(pkg_project_gazebo) / "rviz" / "demo.rviz"}',
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    ld.append(rviz)

    return ld

def generate_launch_description():

    return LaunchDescription(
    [   DeclareLaunchArgument("spawn_only", default_value="False", description="Spawn the robots in the existing instance of gzsim world"),
        DeclareLaunchArgument("world", default_value="voliere", description="Name of world to spawn the robots in (without .sdf)"),
        DeclareLaunchArgument("rviz", default_value="false", description="Open RViz."), # Yet to configure RViz
        DeclareLaunchArgument("nb_drones", default_value="3", description="Number of drones to spawn."),
        DeclareLaunchArgument("drone_x", default_value="[0,0,0]", description="x position of the robots. If not provided, the robots will be spawned at x= (current robot number - 1)"),
        DeclareLaunchArgument("drone_y", default_value="[1.4,-1.4,0]", description="y position of the robots. If not provided, the robots will be spawned at y=0"),
        DeclareLaunchArgument("drone_z", default_value="[0.13,0.13,0.13]", description="z position of the robots. If not provided, the robots will be spawned at z=1"),
        OpaqueFunction(function=node_fn),
    ]
    )


def add_rmtt(i, x, y, z, rd_template:Template, world) -> Tuple[List[Node], List]:
    print(f"Adding drone {i}")
    print(f"x: {x}, y: {y}, z: {z}")
    sdf_description = rd_template.substitute(namespace=f"rmtt_{i}")
    state_description = ET.fromstring(sdf_description)
    state_description.set("version", "1.9")
    for link in state_description.findall("./model/link"):
        for visual in link.findall("visual"):
            if visual.find("geometry/cone") is not None:
                link.remove(visual)
    bridge_topics = []
    nodes = []
    bridge_topics.append(f"/rmtt_{i}/cmd_vel_sim@geometry_msgs/msg/Twist]gz.msgs.Twist")
    bridge_topics.append(f"/rmtt_{i}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry")
    bridge_topics.append(f"/rmtt_{i}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU")
    bridge_topics.append(f"/rmtt_{i}/baro@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure")
    bridge_topics.append(f"/rmtt_{i}/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image")
    bridge_topics.append(f"/rmtt_{i}/camera_bottom/image_raw@sensor_msgs/msg/Image[gz.msgs.Image")
    bridge_topics.append(f"/rmtt_{i}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo")
    bridge_topics.append(f"/rmtt_{i}/altimeter@ros_gz_interfaces/msg/Altimeter[gz.msgs.Altimeter")
    bridge_topics.append(f"/rmtt_{i}/enable_velocity_controller@std_msgs/msg/Bool]gz.msgs.Boolean")
    # Spawn entity
    nodes.append(Node(
            package="ros_gz_sim",
            executable="create",
            output='screen',
            arguments=[
                "-world",
                f"{world}",
                "-allow_renaming",
                "true",
                "-name",
                f"rmtt_{i}",
                "-string",
                sdf_description,
                "-x",
                f"{x}",
                "-y",
                f"{y}",
                "-z",
                f"{z}",
                "-R",
                "0",
                "-P",
                "0",
                "-Y",
                "0",
            ],
        ))
    # Sim server
    nodes.append(Node(
        package=PKG_NAME,
        executable="drone_srv",
        name=f"drone_srv_{i}",
        namespace=f"rmtt_{i}",
        output="screen",
        ))
    # Robot state publisher
    nodes.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters= [   
                    {'use_sim_time': True},
                    {"robot_description": ET.tostring(state_description, encoding="unicode")},
                    {"frame_prefix": f"rmtt_{i}"},],
        ))
    print(f"Drone {i} spawned")
    # Static TF publisher
    nodes.append(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_tf_publisher_{i}",
        output="screen",
        parameters=[{'use_sim_time': True}],
        arguments=[
            "-x", f"{x}",
            "-y", f"{y}",
            "-z", f"{z}",
            "-r", "0", "0", "0",
            "-q", "0", "0", "0", "1",
            f"rmtt_{i}", f"world", "100"
        ],
    ))
    return nodes, bridge_topics
