.. _howtos:

Using the hardware driver
=========================

Network and startup
-------------------

Connect the PC and devices to the same LAN. Configure the PC's LAN address,
device names, counts and IPs in the files described in :ref:`configuration`.
For static assignment, give each device a stable address and set
``random_assign: false``. Discovery assigns found devices in sorted IP order;
it does not establish permanent identity-to-name mapping.

For initial Wi-Fi setup, edit ``setup_wifi.yaml`` and run the helper. RMTTs
begin in direct/AP mode for configuration, then must be switched to router/STA
mode. EP devices use router mode and the generated QR code:

.. code-block:: bash

   source install/setup.bash
   ros2 launch robomaster_ros2 setup_wifi.launch.py

The optional IP/serial helper uses ``retrive_robot_info.yaml`` and writes
``sn_to_name.yaml``:

.. code-block:: bash

   ros2 launch robomaster_ros2 retrive_robot_info.launch.py

Start the server with the configured YAML values:

.. code-block:: bash

   ros2 launch robomaster_ros2 robomaster_server.launch.py

For explicit counts, also supply the intended LAN address. For example, after
setting ``ROBOT_LAN_IP`` to the PC's address:

.. code-block:: bash

   ros2 launch robomaster_ros2 robomaster_server.launch.py \
     num_of_drones:=1 num_of_eps1:=1 local_ip:="$ROBOT_LAN_IP" \
     random_assign:=false keyboard_cmd:=false

When keyboard control is enabled, Alt+A arms all registered drones, Alt+T
requests takeoff, Alt+L requests landing and Alt+E requests emergency stop.
Disable server keyboard shortcuts when another tool owns motion commands.

Direct camera images
--------------------

Enable ``pub_cam`` for the intended devices. Both RMTT and EP/Core drivers
publish directly on ``/<name>/image`` using ``sensor_msgs/msg/Image`` with
``bgr8`` encoding. QoS is best-effort, volatile and depth one. In RViz choose
the raw image topic and Best Effort reliability; a reliable-only subscriber
does not match this publisher.

Video is decoded with PyAV. The ROS header uses the device name as ``frame_id``
and the node clock at publication as the timestamp. ``publish_fps`` is a ceiling,
not a guaranteed delivered frequency: Wi-Fi, device capture, decoding and
subscriber load can reduce it. No compression plugin or extra bridge process
is used. Standard ROS topic remapping applies to the direct publisher.

RMTT FPS, bitrate and resolution live under ``rmtt.camera``; direction is
selected per drone. EP/Core supports 360p, 540p and 720p and receives the device
video stream on TCP port 40921. See :ref:`configuration` for camera controls.
Stopping/removing a device stops its video path; the device node owns its ROS
publisher. The optional legacy ``libmedia_codec`` binding is for EP audio only.

Motion gate and telemetry
-------------------------

Use ``set_armed`` for the chassis/gimbal or RMTT flight gate when arming is
required:

.. code-block:: bash

   ros2 service call /rmtt_1/set_armed std_srvs/srv/SetBool "{data: true}"
   ros2 service call /rmep_1/set_armed std_srvs/srv/SetBool "{data: true}"

Refresh non-zero gated velocities within the configured ``command_timeout``.
Expiry sends chassis/gimbal zeros or an RMTT RC hover command. A latched
expiry requires rearming. Disarming explicitly stops these gated channels.
This gate does not cover the current EP arm velocity stream.

Enable ``pub_position`` for ``/<robot>/position`` (PoseStamped). Publication
waits for position and chassis-attitude samples, and orientation is a quaternion
with yaw relative to the first attitude sample. This is chassis telemetry,
not externally measured world pose. ``pub_gimbal_angle`` enables GimbalAngle
feedback on models with a gimbal. EP battery percentage comes from DDS;
``get_battery`` returns failure until a battery sample is available.

LEDs and armor
--------------

Ground robots provide ``led_chassis``. Gimbal-equipped robots additionally
provide ``led_gimbal`` and ``led_blaster``. RGB channels use 0--255; gimbal LED
indices are 0--7 and an empty selection uses the driver's default indices 0--3.

.. code-block:: bash

   ros2 service call /rmep_1/led_blaster robomaster_interface/srv/BlasterLED "{'on': true, brightness: 128}"
   ros2 service call /rmtt_1/set_mled robomaster_interface/srv/TelloMled "{brightness: 64, color: 'r', mled_char: 'A', graph: '', scroll_dir: '', freq: 0.0}"

RMTT matrix LEDs accept a character or a 64-character custom graph using
``0``, ``r``, ``b`` and ``p``. Armor support publishes ``armor_hit`` and
``ir_hit`` events and exposes ``set_armor_sensitivity``. The armor topic includes
component/type/strength; IR feedback includes cumulative count and receiver
metadata. Event topics do not need to publish continuously.

EP Core gripper
---------------

``gripper`` accepts percentage targets 0, 25, 50, 75 and 100, returning after
acceptance. ``gripper_state`` reports busy/completion state and the estimate.
``gripper_command`` accepts ``open``, ``close``, ``pause`` or ``reset``;
``set_gripper_mode`` selects ``timed`` or ``feedback`` control.

.. code-block:: bash

   ros2 service call /rmep_1/gripper robomaster_interface/srv/Gripper "{distance: 100}"
   ros2 service call /rmep_1/set_gripper_mode robomaster_interface/srv/SetGripperMode "{mode: 'feedback'}"
   ros2 service call /rmep_1/gripper_command robomaster_interface/srv/GripperCommand "{command: 'pause', power: 0}"

Hardware feedback reports only opened/closed/normal, not a continuous percentage.
Feedback mode confirms endpoints; intermediate targets use calibrated timing.
At startup, or after an interrupted/failed movement invalidates the estimate,
command an endpoint before an intermediate target. Recalibrate full-travel time
if the configured power changes.

EP Core tracked arm services
----------------------------

``move_arm`` is relative and ``move_arm_to`` absolute, using X/Z metres. The
response contains acceptance, action ID, state, percentage and completion.
Relative moves clamp X to +/-0.22 m and Z to +/-0.15 m; out-of-range absolute
targets are rejected. Optional completion waits must be finite and at most
60 seconds; a non-positive request timeout selects ``arm.completion_timeout``.

.. code-block:: bash

   ros2 service call /rmep_1/move_arm robomaster_interface/srv/MoveArm "{x: 0.02, z: 0.01, wait_for_completion: false, timeout: 0.0}"
   ros2 service call /rmep_1/cancel_arm std_srvs/srv/Trigger "{}"

``arm_action_status`` describes tracked actions. ``recenter_arm`` starts an
absolute move to zero; ``reset_arm`` cancels an active action and recenters,
because there is no distinct implemented SDK hardware reset. ``arm_position``
is a legacy PointStamped topic that passes DDS values through; do not infer
its units from the metre-based service interface.

EP Core arm velocity stream
---------------------------

``/<name>/arm_cmd_vel`` accepts Twist ``linear.x`` and ``linear.z`` in m/s.
Other components are ignored. The current server uses 20 Hz timer output,
clamps each velocity to +/-0.20 m/s and each relative increment to +/-0.03 m,
and stops issuing increments after 0.25 seconds without a command. The deadline
uses the ROS node clock. An increment already dispatched can finish.

These settings are driver constructor defaults, not YAML/launch parameters.
Zero X/Z velocities stop new increments. The stream skips dispatch while a
tracked arm action is observed busy; stream increments themselves are untracked.

Current limitations: the stream does not check ``set_armed`` or explicitly
reject NaN/Inf, and disarm/expiry do not clear the cached command. The busy check
and dispatch are separate, so concurrent streaming and tracked services do not
have an atomic priority handoff. Stop the streaming publisher and send an
explicit zero instead of relying on disarm to stop this arm path. These are
implementation limits, not functionality supplied by the motion gate.

See :ref:`fleet_tools` for joystick and diagnostic behavior, including their
current monitoring and stop limitations.
