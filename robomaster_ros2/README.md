# RoboMaster ROS 2 driver

## Introduction

This ROS 2 driver focus on the DJI RoboMaster serise robots and drones. 

## Installation

Required:
```
pip install pillow myqr pynput
sudo apt-get install network-manager libopus-dev
sudo apt install ros-humble-tf-transformations
```

Optional:
```
For video streaming
pip install git+https://github.com/aarsht7/RoboMaster-SDK.git@libmedia_codec

For UI interface (WIP)
pip install nicegui
```
 

## Getting started

To use this driver and control the robot(s) over wifi, it is suggested to assign a static IP to the robot's mac address in your loacl network. You can always use random assigning if you do not have facilities to assign static IP.

### Connecting the robots to the local network

First you will need to setup the robots to be able to connect to the local wifi network. RoboMaster seriese of robots have mainly 2 connection possibilities, Connecting robot directly to the mobile device (mobile mode) and connecting robot to the local wifi network (router mode).

To setup the connection to the local wifi network,

- Modify `robomaster_ros2/config_ros/setup_wifi.yaml` according to need.
- Run `ros2 launch robomaster_ros2 setup_wifi.launch.py`
- Follow the instruction on terminal.


### Retriving IP address and SN 

This is helpful when you have static IP assigned to the robots, So you can save all the IP, SN and type of the robots in a file and you can name the robots to remember for later.

- Connect PC and all the robots and drones to the local wifi network.
- Modify `robomaster_ros2/config_ros/retrive_robot_info.yaml` according to need.
- Run `ros2 launch robomaster_ros2 retrive_robot_info.launch.py`
- This will save all the data to the `robomaster_ros2/config_ros/sn_to_name.yaml`

### Running the RoboMaster Server

- Modify `robomaster_ros2/config_ros/rmtt_param.yaml` according to need. 
- Modify `robomaster_ros2/config_ros/rmep_param.yaml` according to need. 
- Modify `robomaster_ros2/config_ros/robomaster_server.yaml` according to need. 
- Run `ros2 launch robomaster_ros2 robomaster_server.launch.py`

