.. _configuration:

Configuration
=============

All configuration files are located in the `robomaster_ros2/config_ros` directory.

* robomaster_server.yaml: Setting up the server node and parameters that are changed often.
* rmtt_param.yaml: Configuration for the RoboMaster Tello Talent (RMTT) drones.
* rmep_param.yaml: Configuration for the RoboMaster EP Core (RMEP) robots.
* custom_char.yaml: Custom characters for the RMTT drones external module LED matrix.
* setup_wifi.yaml: Configuration for the Wi-Fi connection of the RMTT drones and RMEP robots.
* retrieve_robot_info.yaml: Configuration for the retrieval of information of connected robots and drones.

robomaster_server.yaml
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: yaml

    local_ip: "192.168.0.106"      # local ip of the network on which drones are connected
    num_of_drones: 0               # number of drones to be used
    num_of_eps1: 0                 # number of EP or S1 Robots to be used
    random_assign: True            # whether to assign drones and robot numbers randomly (Ascending order of IPs last octet)


rmtt_param.yaml
~~~~~~~~~~~~~~~

.. code-block:: yaml

    rmtt:
        mled_display_num: True         # whether to display the number of the drone on the mled  on initalisation
        external_position: False       # (WIP) whether to use external positioning system such as mocap
        drone_name_list:               # list of names of all the drones that you want to add now or in future (mandatory)
            - rmtt_1   
            - rmtt_2  
            - rmtt_3

        pub_front_tof: False           # (Too heavy on process, avoid using with multi drones) whether to publish front tof sensor data
        pub_bottom_tof: True           # whether to publish bottom tof sensor data
        pub_attitude: True             # whether to publish attitude data
        pub_barometer: False           # (WIP units issue) 
        pub_imu: False                 # (WIP units issue) 
        pub_mpad: False                # Publish info of mission pad below the drone

        # It is advisible to use the random_assign option in robomaster_server.yaml
        
        #####MANDATORY#####
        # Individual drone configurations and parameter control for all the drones
        # name in the list should be the same as the name in the drone_name_list
        # topic_type: Pose/ PoseStamped/ Point/ PointStamped
        rmtt_1:
            ip: "192.168.0.151"
            pub_cam: False
            cam_direction: 0             # 0: forward, 1: down
            yaw_hold: False              # WIP
            ext_pose_topic_type: PoseStamped
            ext_pose_topic_name: /rmtt_1/pose

        rmtt_2:
            ip: "192.168.0.152"
            pub_cam: False
            cam_direction: 0             # 0: forward, 1: down
            yaw_hold: False              # WIP
            ext_pose_topic_type: PoseStamped
            ext_pose_topic_name: /rmtt_2/pose

        rmtt_3:
            ip: "192.168.0.153"
            pub_cam: False
            cam_direction: 0             # 0: forward, 1: down
            yaw_hold: False              # WIP
            ext_pose_topic_type: PoseStamped
            ext_pose_topic_name: /rmtt_3/pose
