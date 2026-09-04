# RoboMaster ROS 2 driver

## Introduction

This ROS 2 driver focus on the DJI RoboMaster serise robots and drones. 

## Installation

Required:
```
pip install pillow myqr pynput
sudo apt-get install network-manager
sudo apt install python3-av ros-humble-tf-transformations ros-humble-cv-bridge
```

Optional:
```
For EP audio streaming only
sudo apt-get install libopus-dev
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

### Camera streaming

Enable `pub_cam` for each RMTT or EP/Core that should publish video. Both
robot types publish `sensor_msgs/msg/Image` on `/<name>/image` with a
best-effort, depth-one sensor QoS profile.

Video is decoded with PyAV. The older DJI `libmedia_codec` H.264 binding uses
deprecated FFmpeg APIs and can corrupt native heap memory on current Ubuntu and
FFmpeg versions; it is retained only as an optional EP audio decoder.

RMTT camera load is controlled globally under `rmtt.camera`:

```yaml
camera:
  fps: high
  bitrate: 2
  resolution: low
  publish_fps: 20.0
```

These defaults target multi-drone use: the drone keeps a high capture rate,
but 480p and 2 Mbit/s reduce Wi-Fi airtime, decoding cost, and ROS image
serialization. For maximum single-drone quality use `high`, `5`, `high`.

EP/Core camera resolution and ROS publication rate are configured under
`rmep.camera`:

```yaml
camera:
  resolution: 540p
  publish_fps: 20.0
```

Valid EP resolutions are `360p`, `540p`, and `720p`. The EP stream uses the
DJI SDK TCP video endpoint on port `40921`.

### EP telemetry and LED controls

Set `pub_position: True` under `rmep` to publish chassis x/y displacement and
yaw-aware pose on `/<name>/position` at 10 Hz as `geometry_msgs/msg/PoseStamped`.
Orientation is published as a quaternion in `pose.orientation`. Set `pub_gimbal_angle: True` to publish
`/<name>/gimbal_angle` for EP/S1 robots that have a gimbal.

EP/S1 gimbal robots expose `/led_gimbal` for selecting armor LEDs 0-7 and
`/led_blaster` for blaster brightness. RMTT drones expose `/set_mled`
(`robomaster_interface/srv/TelloMled`) for matrix brightness, color,
character, optional custom graph, and scrolling options.

### EP battery and armor

EP/Core/S1 robots expose the same percentage-oriented battery service as an
RMTT: `/<name>/get_battery` uses `std_srvs/srv/Trigger` and returns a message
such as `73%`. EP battery data is supplied by DDS, so the service returns
`success: false` until the first sample arrives.

When `rmep.armor.enabled` is true, armor events are published on
`/<name>/armor_hit` and `/<name>/ir_hit`. The first topic includes the armor ID,
component, water/IR type, and impact strength; the second includes cumulative IR
hit count and DJI receiver metadata. Set hit sensitivity with
`/<name>/set_armor_sensitivity`.

### EP Core gripper and robotic arm

The legacy `/<name>/gripper` percentage service remains available and now
returns immediately after accepting a command. Progress and the estimated
percentage are published on `/<name>/gripper_state`. Select `timed` or
`feedback` control using `/<name>/set_gripper_mode`, and use
`/<name>/gripper_command` for `open`, `close`, `pause`, and `reset`.

DJI gripper feedback reports only `opened`, `closed`, or `normal`. Feedback mode
therefore uses hardware feedback for the 0% and 100% endpoints; 25%, 50%, and
75% still use the calibrated timing table. An endpoint command is required
before an intermediate command when the startup position is unknown. Pausing or
failing after motion begins invalidates the estimate and requires another
endpoint calibration. Recalibrate `full_travel_time` when changing the power
used for timed motion.

EP Core arm services include relative `move_arm`, absolute `move_arm_to`,
`recenter_arm`, `reset_arm`, and `cancel_arm`. Action ID, state, percentage, and
completion are published on `/<name>/arm_action_status`. `reset_arm` cancels an
active action and recenters because DJI's SDK does not provide a distinct
hardware reset operation. Completion waits must use a finite timeout of at most
60 seconds.

### Motion safety

The EP chassis/gimbal and RMTT flight velocity paths use a shared command
gate. By default, a non-zero command must be refreshed within `0.5` seconds.
If it expires, EP sends zero velocity and RMTT sends an `rc 0 0 0 0` hover
command. Existing topic and service names are unchanged.

Configure the gate under `rmep.safety` in `rmep_param.yaml` and
`rmtt.safety` in `rmtt_param.yaml`:

- `command_timeout`: deadman timeout in seconds; `0.0` disables expiry.
- `require_arm`: start with motion disarmed when `True`.
- `latch_deadman`: require explicit re-arming after a timeout when `True`.

Arm or disarm an individual robot with:

```
ros2 service call /rmtt_1/set_armed std_srvs/srv/SetBool "{data: true}"
ros2 service call /rmep_1/set_armed std_srvs/srv/SetBool "{data: true}"
```

Disarming always sends a safe zero/hover command. Landing and emergency
commands remain available regardless of the motion arm state.
