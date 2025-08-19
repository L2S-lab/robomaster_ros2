.. _configuration:

Configuration
=============

All configuration files are located in the `robomaster_ros2/config_ros` directory.

.. note::
   Configuration files are subject to minor changes, please check the latest version in the repository.

* `robomaster_server.yaml`: Setting up the server node and parameters that are changed often.
* `rmtt_param.yaml`: Configuration for the RoboMaster Tello Talent (RMTT) drones.
* `rmep_param.yaml`: Configuration for the RoboMaster EP Core (RMEP) robots.
* `setup_wifi.yaml`: Configuration for the Wi-Fi connection of the RMTT drones and RMEP robots.
* `custom_char.yaml`: Custom characters for the RMTT drones external module LED matrix.
* `retrieve_robot_info.yaml`: Configuration for the retrieval of information of connected robots and drones.

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

rmep_param.yaml
~~~~~~~~~~~~~~~

.. code-block:: yaml

    rmep:
        led_num: False                 # whether to display specific led pattern for specific number (R1,R2,R3,R4,G1,G2,G3,G4,B1,B2,B3,B4) etc for 1 to 12
        external_position: False       # (WIP) whether to use external positioning system such as mocap 
        robot_name_list:               # list of names of the robots (mandatory)
            - rmep_1   
            - rmep_2  

        # WIP (Work in Progress) not working now
        pub_imu: False                 # (WIP units issue)
        pub_cam: False                 # (WIP units issue)
        pub_marker: False              # WIP
        pub_armpose: True              # Publish arm pose of the robot

    # Individual robot configurations and parameter control
    # name in the list should be the same as the name in the robot_name_list
    # mandatory while usuing external position
    # topic_type: Pose/ PoseStamped/ Point/ PointStamped
        rmep_1:
            ip: "192.168.0.161"
            pub_cam: False # WIP
            ext_pose_topic_type: PoseStamped
            ext_pose_topic_name: /rmep_1/pose

        rmep_2:
            ip: "192.168.0.162"
            pub_cam: False # WIP
            ext_pose_topic_type: PoseStamped
            ext_pose_topic_name: /rmep_2/pose

setup_wifi.yaml
~~~~~~~~~~~~~~~

.. code-block:: yaml
    
    # ssid of the router that all robots will connect to
    SSID: "TP-link_1234"

    # password of the router that you want to connect
    # can be "-1" to retrive the password from the saved network 
    # OR "None" if there is no password
    PSK: "-1"

    # use only provided numbers in form of the list to modify the details
    # [0] if you don't use drones
    UPDATE: [8,6]

    # For EP/S1 robots, you have to connect manually scanning a QR code. 
    # It will dispaly the QR code at end and save the image in the base directory 
    EP: False

    drone_list:
        #pwd: password of all the tello if they are the same, 
        # use n#>pwd if there is different passwords or "None" if there is no password
        pwd: "12345678"
    # n+number:
    #   ssid: ssid of the tello hotspot
    #   pwd: password associated with the above ssid ('' for no password)
        n1:
            ssid: "RMTT-1"
            pwd: "12345678"

        n2:
            ssid: "RMTT-2"
            pwd: "12345678"

        n3:
            ssid: "RMTT-3"
            pwd: "12345678"

custom_char.yaml
~~~~~~~~~~~~~~~~

.. code-block:: yaml

    # You can put your custom characters here with the following format.
    # Replace bits with the color you want to put at that pixel
    # 0: off, r: red, b: blue, p:purple
    # smile:
    # 0 0 ∎ ∎ ∎ ∎ 0 0
    # 0 ∎ 0 0 0 0 ∎ 0
    # ∎ 0 ∎ 0 0 ∎ 0 ∎
    # ∎ 0 0 0 0 0 0 ∎
    # ∎ 0 ∎ 0 0 ∎ 0 ∎
    # ∎ 0 0 ∎ ∎ 0 0 ∎
    # 0 ∎ 0 0 0 0 ∎ 0
    # 0 0 ∎ ∎ ∎ ∎ 0 0
    # For visualisation: https://xantorohara.github.io/led-matrix-editor/

    smile: '00rrrr000r0000r0r0r00r0rr000000rr0r00r0rr00rr00r0r0000r000rrrr00'
    smile2: '000000000b0000b0b0b00b0b000000000000000000r00r00000rr00000000000'
    ok: '000000000000000r000000r000000r00r000r0000r0r000000r0000000000000'
    star: '000000000rr00rr000rrrr00rrrrrrrr00rrrr000rr00rr00000000000000000'
    question: '00rrrr000rr00rr000000rr00000rr00000rr00000000000000rr00000000000'
    stop: '0000000000rrrr000rr000r00r0r00r00r00r0r00r000rr000rrrr0000000000'
    exclamation: '000rr00000rrrr0000rrrr00000rr000000rr00000000000000rr00000000000'
    double_exclamation: '0rr00rr00rr00rr00rr00rr00rr00rr0000000000rr00rr00rr00rr000000000'
    heart: '0000000000r00r000rrrrrr00rrrrrr00rrrrrr000rrrr00000rr00000000000'


retrieve_robot_info.yaml
~~~~~~~~~~~~~~~~~~~~~~~~

This is not used in the latest version but can be handy, hence it is included here.

.. code-block:: yaml

    # number of drones connected to network (0 if you are not using any)
    RMTT: 1
    # number of EP/S1 robots connected to network (0 if you are not using any)
    EP: 0