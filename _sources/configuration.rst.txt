.. _configuration:

Configuration
=============

Configuration files live in ``robomaster_ros2/config_ros``. Values below are
examples, not a replacement for the files in the local checkout. Camera flags,
network addresses, command deadlines and fleet size can differ between setups.

Server and launch selection
---------------------------

``robomaster_server.yaml`` configures the server node:

.. code-block:: yaml

   robomaster_server:
     ros__parameters:
       local_ip: "192.168.0.101"
       num_of_drones: 1
       num_of_eps1: 1
       random_assign: true
       keyboard_cmd: false

``local_ip`` must be the PC's address on the robots' LAN. ``random_assign``
enables discovery and assigns the discovered IP addresses in sorted order;
false uses the per-device configured addresses. Requested counts must fit the
name lists and have matching per-device entries.

The server launch arguments are ``num_of_drones``, ``num_of_eps1``, ``local_ip``,
``random_assign`` and ``keyboard_cmd``. If both count arguments are zero, the
launcher loads the server YAML file. If either count is non-zero, it constructs
the server parameters from launch arguments instead. In that case, supply the
intended LAN address and both counts explicitly. To disable keyboard shortcuts
in the YAML-driven path, set ``keyboard_cmd: false`` in the file itself.

Device parameter files
----------------------

``rmtt_param.yaml`` and ``rmep_param.yaml`` belong to the ``param_server`` node,
under ``ros__parameters.rmtt`` and ``ros__parameters.rmep`` respectively. For
example, the structure of an RMTT configuration is:

.. code-block:: yaml

   param_server:
     ros__parameters:
       rmtt:
         drone_name_list: [rmtt_1]
         mled_display_num: true
         external_position: false
         pub_front_tof: false
         pub_bottom_tof: true
         pub_attitude: true
         pub_barometer: false
         pub_imu: false
         pub_mpad: false
         camera:
           fps: high
           bitrate: 2
           resolution: low
           publish_fps: 20.0
         safety:
           command_timeout: 0.5
           require_arm: false
           latch_deadman: false
         rmtt_1:
           ip: "192.168.0.151"
           pub_cam: true
           cam_direction: 0
           yaw_hold: false
           ext_pose_topic_type: PoseStamped
           ext_pose_topic_name: /rmtt_1/pose

RMTT camera FPS accepts ``high``, ``middle`` or ``low``. Bitrate is ``0`` for
automatic selection or an SDK bitrate setting from 1 to 5. Resolution is
``low`` or ``high``. ``publish_fps`` caps ROS publication frequency; it does not
guarantee the same capture or delivery rate. ``cam_direction`` selects forward
(0) or downward (1) video per drone. Front ToF polling can slow communication;
barometer/IMU units and ``yaw_hold`` remain incomplete areas.

External pose input is enabled with ``external_position``. Each device selects
its input topic and one of ``Pose``, ``PoseStamped``, ``Point`` or
``PointStamped``. These are subscriptions, not an automatically provided
positioning system. The drone ``move`` and ``goto`` callbacks remain
unimplemented even when their services appear.

EP/Core/S1 settings use the following structure:

.. code-block:: yaml

   param_server:
     ros__parameters:
       rmep:
         robot_name_list: [rmep_1]
         led_num: false
         external_position: false
         pub_imu: false
         pub_cam: false
         pub_marker: false
         pub_armpose: true
         pub_position: true
         pub_gimbal_angle: true
         camera:
           resolution: 540p
           publish_fps: 20.0
         armor:
           enabled: true
         gripper:
           control_mode: timed
           power: 50
           full_travel_time: 1.25
           feedback_timeout: 3.0
         arm:
           completion_timeout: 10.0
         safety:
           command_timeout: 0.5
           require_arm: false
           latch_deadman: false
         rmep_1:
           ip: "192.168.0.161"
           pub_cam: true
           ext_pose_topic_type: PoseStamped
           ext_pose_topic_name: /rmep_1/pose

The global EP ``pub_cam`` flag enables cameras for all configured robots. A
per-robot true flag can enable a camera when the global flag is false; a
per-robot false flag does not disable a globally enabled camera. Supported
camera resolutions are ``360p``, ``540p`` and ``720p``. Gimbal endpoints require
a model with a gimbal; arm/gripper endpoints require the EP Core manipulator.

Motion gate
-----------

The ``safety`` settings apply to RMTT flight velocity and ground chassis/gimbal
velocity paths:

- ``command_timeout``: seconds until a non-zero command expires; 0.0 disables
  expiry. Read the selected device file for its configured value.
- ``require_arm``: true starts the gate disarmed; use ``/<name>/set_armed``
  (``std_srvs/srv/SetBool``) before gated motion.
- ``latch_deadman``: true requires explicit rearming after an expiry.

Landing, hover and emergency operations remain available for stopping a drone.
The arm velocity stream is separate: in this implementation it does not consult
the motion gate. Its 20 Hz rate, 0.25-second ROS-clock timeout, +/-0.20 m/s
velocity clamps and +/-0.03 m increment bounds are constructor defaults, not
parameters loaded from these YAML files. ``arm.completion_timeout`` controls
tracked service waits only. See :ref:`howtos` for the current stream limitations.

Other configuration files
-------------------------

- ``setup_wifi.yaml``: the ``setup_wifi`` node's LAN SSID/PSK, selected drone
  hotspot entries and EP QR-code setup. ``PSK: "-1"`` requests a saved password;
  ``UPDATE`` selects drone entries and ``EP`` enables the EP setup step.
- ``custom_char.yaml``: 8x8 matrix patterns, encoded as 64 characters using
  ``0`` (off), ``r``, ``b`` and ``p``. Used with ``set_mled``.
- ``retrive_robot_info.yaml``: device counts for the optional IP/serial-number
  discovery helper. The executable and filename retain the ``retrive`` spelling.
- ``sn_to_name.yaml``: device identity/configuration data produced by the
  discovery helper.

Joystick parameters are configured on the teleop node, not in these server
files; see :ref:`fleet_tools`.
