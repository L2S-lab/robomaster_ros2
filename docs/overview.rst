.. _overview:

Packages and ROS API
====================

``robomaster_ros2`` contains the hardware server, parameter server, network
helpers and device drivers. ``robomaster_interface`` defines their custom
messages and services. ``robomaster_examples`` supplies waypoint/trajectory
examples, fleet diagnostics and joystick control. Simulation is documented
separately in :ref:`simulation`.

Namespaces and availability
---------------------------

The server uses root fleet services. Each device node is created in its own
namespace: a device named ``rmtt_1`` has node name ``/rmtt_1/rmtt_1`` and topics
such as ``/rmtt_1/image``. Replace ``<name>`` below with the configured name.
Sensor flags and detected model capabilities control endpoint creation; not
every robot provides every endpoint.

Server services
---------------

.. list-table::
   :header-rows: 1

   * - Endpoint
     - Service type
     - Behavior
   * - ``/add_drone``
     - AddDrone
     - Register an RMTT with address, camera and optional external-pose settings.
   * - ``/remove_robot``
     - RemoveRobot
     - Remove an RMTT by name and type ``rmtt``.
   * - ``/add_robot``
     - AddRobot
     - Advertised, but the current handler is unimplemented.
   * - ``/takeoff_all`` / ``/land_all``
     - Trigger
     - Created only while at least one drone is registered.

Custom service names refer to ``robomaster_interface/srv``. ``Trigger``,
``Empty`` and ``SetBool`` refer to ``std_srvs/srv``.

Drone services
--------------

.. list-table::
   :header-rows: 1

   * - Relative endpoint
     - Type
     - Behavior
   * - ``get_sn``, ``get_battery``, ``status``
     - Trigger
     - Serial number, battery percentage and flight state.
   * - ``set_speed``
     - SetSpeed
     - Set SDK movement speed; this does not change the normalized cmd_vel interface.
   * - ``takeoff``
     - Takeoff
     - Height in metres and a sync flag.
   * - ``land``, ``soft_emergency``
     - Trigger
     - Landing or emergency landing.
   * - ``hover``, ``emergency``, ``reboot``
     - Empty
     - Stop RC commands, stop motors, or reboot.
   * - ``set_armed``
     - SetBool
     - Control the flight motion gate.
   * - ``set_led``, ``set_mled``
     - TelloLED / TelloMled
     - External-module RGB and matrix LEDs.
   * - ``move``, ``goto``
     - Move / GoTo
     - May be advertised with external positioning, but callbacks are unimplemented.

Ground robot services
---------------------

.. list-table::
   :header-rows: 1

   * - Relative endpoint
     - Type
     - Availability / behavior
   * - ``get_sn``, ``get_battery``
     - Trigger
     - EP/Core/S1 identity and DDS-cached battery percentage.
   * - ``set_armed``
     - SetBool
     - Control chassis/gimbal motion gate; does not guard the current arm stream.
   * - ``move_chassis``
     - MoveChassis
     - Tracked chassis movement.
   * - ``close_robot``
     - Empty
     - Close the device connection.
   * - ``led_chassis``
     - RobotLED
     - Chassis armor LEDs.
   * - ``set_armor_sensitivity``
     - SetArmorSensitivity
     - Available when armor support is enabled.
   * - ``move_gimbal``, ``reset_gimbal``
     - MoveGimbal / Trigger
     - Models with a gimbal.
   * - ``led_gimbal``, ``led_blaster``, ``fire``
     - GimbalLED / BlasterLED / Fire
     - Models with a gimbal/blaster.
   * - ``move_arm``, ``move_arm_to``
     - MoveArm
     - EP Core relative/absolute arm actions with optional completion wait.
   * - ``recenter_arm``, ``reset_arm``, ``cancel_arm``
     - Trigger
     - EP Core tracked-action lifecycle.
   * - ``gripper``
     - Gripper
     - EP Core percentage command, accepted asynchronously.
   * - ``gripper_command``, ``set_gripper_mode``
     - GripperCommand / SetGripperMode
     - EP Core named operations and timed/feedback control.

Topics and units
----------------

The endpoint is ``/<name>/<topic>`` unless stated otherwise. Standard message
names below come from ``geometry_msgs/msg`` or ``sensor_msgs/msg``; named
RoboMaster messages come from ``robomaster_interface/msg``.

.. list-table::
   :header-rows: 1

   * - Device / topic
     - Message
     - Direction / semantics
   * - Both: ``image``
     - sensor_msgs/Image
     - Published raw bgr8 frames; best-effort, volatile, depth 1.
   * - Drone: ``cmd_vel``
     - Twist
     - Subscribed normalized RC inputs in [-1, 1]; not m/s or rad/s.
   * - Drone: ``front_tof``, ``bottom_tof``
     - Range
     - Enabled distance sensors.
   * - Drone: ``attitude``
     - QuaternionStamped
     - Enabled attitude telemetry.
   * - Drone: ``baro``, ``imu``, ``mpad``
     - FluidPressure / Imu / TelloMpad
     - Optional telemetry; barometer/IMU unit handling remains incomplete.
   * - Drone: ``rmtt_server/pose``
     - Pose
     - Republished external-pose input when enabled.
   * - Ground: ``cmd_vel``
     - Twist
     - Subscribed chassis translation in m/s and yaw in rad/s.
   * - Ground: ``position``
     - PoseStamped
     - DDS position with quaternion orientation from chassis attitude.
   * - Gimbal: ``gimbal_cmd_vel``
     - GimbalVel
     - Subscribed pitch/yaw speed in degrees/second.
   * - Gimbal: ``gimbal_angle``
     - GimbalAngle
     - Enabled angle feedback.
   * - EP Core: ``arm_cmd_vel``
     - Twist
     - Subscribed X/Z stream velocities in m/s; independent of the motion gate.
   * - EP Core: ``arm_position``
     - PointStamped
     - Enabled legacy arm DDS feedback; service command units must not be inferred from this topic.
   * - EP Core: ``arm_action_status``
     - ArmActionStatus
     - Tracked service action ID, state, progress and completion.
   * - EP Core: ``gripper_state``
     - GripperState
     - Hardware endpoint status plus estimated percentage, mode and operation state.
   * - Armor: ``armor_hit``, ``ir_hit``
     - ArmorHit / IrHit
     - Event topics; silence can be normal.

External pose subscriptions use the configured ``ext_pose_topic_name`` and
``ext_pose_topic_type``. There is no fixed ``external_position`` input topic.
The real driver does not create a positioning system from these settings.

Camera publication is direct; the driver creates no compressed image topic or
camera bridge. Most other topics use reliable QoS. Arm velocity uses depth 20
and gimbal velocity depth 60. See :ref:`howtos` for lifecycle/units and
:ref:`fleet_tools` for current example limitations.
