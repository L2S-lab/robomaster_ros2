.. _fleet_tools:

Fleet diagnostics and joystick control
======================================

These examples require an already-running hardware server. Device names must
match its registered namespaces. Configure the actual counts and LAN address
as described in :ref:`configuration`; disable server keyboard shortcuts when
the example owns motion commands.

Fleet diagnostics
-----------------

``fleet_stress_test`` defaults to five drones (``rmtt_1`` through ``rmtt_5``),
two robots (``rmep_1`` and ``rmep_2``) and the read-only ``diagnostic`` mode:

.. code-block:: bash

   ros2 run robomaster_examples fleet_stress_test --mode diagnostic --rounds 20 --duration 30

Use ``--drones`` and ``--robots`` for comma-separated namespace lists. A smaller
mixed-fleet invocation is:

.. code-block:: bash

   ros2 run robomaster_examples fleet_stress_test --drones rmtt_1 --robots rmep_1

.. list-table::
   :header-rows: 1

   * - Mode
     - Behavior
   * - ``diagnostic``
     - Concurrent identity/battery/state checks, service latency summaries and topic counts/rates. Sends no motion commands.
   * - ``ground``
     - Also arms motion gates, calls auxiliary LED/speed services and sends zero cmd_vel streams.
   * - ``motion``
     - Also takes off and sends bounded non-zero streams; requires the exact confirmation phrase.

Motion mode requires ``--confirm-motion MOVE_5_DRONES_2_ROBOTS``, including when
custom namespace lists select a smaller fleet. ``--speed`` defaults to 0.10:
ground translation uses m/s, while drone translation uses normalized RC input.
The CLI help currently labels both as m/s; the underlying drone interface is
normalized. Accepted speed inputs are 0.01--0.25. Takeoff height defaults to
0.8 m and must be within the utility's 0.8--1.5 m range.

.. list-table::
   :header-rows: 1

   * - Option
     - Default
     - Meaning
   * - ``--rounds``
     - 10
     - Concurrent service-check rounds.
   * - ``--duration``
     - 10.0
     - Observation or command-stream duration in seconds.
   * - ``--rate``
     - 20.0
     - Command publication Hz per device.
   * - ``--service-wait`` / ``--call-timeout``
     - 20.0 / 5.0
     - Service discovery and response deadlines in seconds.
   * - ``--min-battery``
     - 30.0
     - Minimum reported drone battery percentage for motion.
   * - ``--preflight-delay``
     - 5.0
     - Motion-mode countdown in seconds.
   * - ``--deadman-wait``
     - 1.0
     - Silence interval; choose a value longer than the configured driver timeout.
   * - ``--no-topic-monitor``
     - Off
     - Skip topic subscriptions/counting; keep service diagnostics.

The report records failures, service latency and telemetry counts/rates; a
required failure produces a non-zero exit status. Deadman observation entries
leave physical stop/hover confirmation to the operator rather than measuring
device motion automatically. Hazardous/disruptive APIs such as firing, reboot,
emergency and arm/gripper/gimbal movement are inventoried without being invoked.

Ground/motion cleanup attempts zero commands and disarming. If flight was
attempted it also attempts hover and landing. Cleanup covers normal completion,
Ctrl-C and SIGTERM; diagnostic mode does not run those motion cleanup calls.

Current telemetry-monitor limitations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The monitor discovers advertised topics below fleet namespaces and excludes
only names ending in ``/cmd_vel``. It subscribes with reliable depth-10 QoS and
requires at least one message on every monitored topic. Therefore:

- Best-effort camera publishers do not match these reliable subscriptions.
  A zero image count can be a QoS mismatch, not a failed camera.
- Silent armor/IR events and other command topics can be included and marked
  failed even though no event or command was expected.

Interpret topic failures with those limits in mind. For service-only checks,
use ``--no-topic-monitor``; inspect cameras with a best-effort image subscriber.

Joystick fleet control
----------------------

Install ``ros-humble-joy`` as described in :ref:`installation`. The current
examples manifest does not declare every joystick/launch runtime dependency,
so a successful package build alone does not guarantee the input node is present.

.. code-block:: bash

   ros2 launch robomaster_examples joy_fleet_teleop.launch.py \
     robot_names:=rmep_1,rmep_2 drone_names:=rmtt_1,rmtt_2 start_mode:=robot

The launcher starts ``joy_node`` and ``joy_fleet_teleop``. It does not explicitly
configure joystick autorepeat; do not confuse Joy input frequency with the
teleop node's default 20 Hz output timer.

.. list-table::
   :header-rows: 1

   * - Launch argument
     - Default
     - Meaning
   * - ``robot_names``
     - ``rmep_1``
     - Comma-separated robot namespaces.
   * - ``drone_names``
     - ``rmtt_1``
     - Comma-separated drone namespaces.
   * - ``start_mode``
     - ``drone``
     - ``robot`` or ``drone``. Target cycling stays within this mode.
   * - ``takeoff_height``
     - 1.0
     - Drone takeoff height in metres.
   * - ``require_deadman``
     - true
     - Require L1 for velocity commands.

The node consumes ``/joy`` (sensor_msgs/Joy). Indices below are zero-based and
follow the actual callback bindings; X is index 3 and Y index 2 even if a
controller's printed labels or an older help string use another order.

.. list-table::
   :header-rows: 1

   * - Button / index
     - Robot mode
     - Drone mode
   * - Start / 7
     - Toggle selected robot motion gate.
     - Toggle selected drone motion gate.
   * - Select / 6
     - Cycle robot targets.
     - Cycle drone targets.
   * - L1 / 4
     - Hold for non-zero velocity input.
     - Hold for non-zero velocity input.
   * - R1 / 5
     - Right stick controls gimbal; chassis yaw is zero.
     - No binding.
   * - B / 0
     - Close gripper.
     - No binding.
   * - A / 1
     - Open gripper.
     - Hover.
   * - X / 3
     - Reset gripper.
     - Take off.
   * - Y / 2
     - Pause gripper.
     - Land.

Left-stick axes 0/1 control planar translation; right-stick X (axis 3) controls
yaw. Holding R1 in robot mode uses axes 3/4 for gimbal yaw/pitch. D-pad axis 7
controls arm X and axis 6 arm Z. In drone mode, L2 axis 2 ascends and R2 axis 5
descends when the value exceeds 0.5. A different controller layout must match
these indices before its labels can be relied upon.

L1 gates velocity output only, not button-triggered service calls. Start calls
``set_armed``; its protection is limited to the driver's gated paths and does
not guard the current arm velocity stream.

Current stop behavior
^^^^^^^^^^^^^^^^^^^^^

There is no joystick-disconnect watchdog or ``joy_timeout`` parameter. The
timer retains the last axes/buttons and can keep republishing them after Joy
input stops; the driver's watchdog may therefore continue receiving fresh
commands. Input loss must not be treated as an automatic stop.

Releasing L1 stops chassis/drone velocity publication, leaving their stop to
the configured driver deadline; selected gimbal and arm streams receive zeros.
Target changes do not send an explicit zero to the previous target. Node
shutdown does not explicitly publish final stop commands. Stopping the node
ends its stream, but already accepted service actions are not cancelled and
drones are not automatically landed. These limitations are present in the
current implementation; the launcher does not provide additional safeguards.

Additional node parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^

Parameters not exposed as launch arguments can be set on a directly run node
with ``--ros-args -p ...`` or in a custom launch file. A separate ``joy_node``
must publish ``/joy`` in that case.

.. list-table::
   :header-rows: 1

   * - Parameter
     - Default
     - Units / purpose
   * - ``robot_names_csv`` / ``drone_names_csv``
     - ``rmep_1`` / ``rmtt_1``
     - Namespace fallbacks used by the launcher.
   * - ``start_mode``
     - ``robot``
     - Direct-node default; launcher overrides it to drone.
   * - ``publish_rate`` / ``deadzone``
     - 20.0 / 0.1
     - Output Hz and axis deadzone.
   * - ``robot_max_linear`` / ``robot_max_yaw``
     - 1.8 / 2.4
     - m/s and rad/s.
   * - ``drone_max_linear`` / ``drone_max_yaw``
     - 0.8 / 0.8
     - Normalized RC inputs, not physical velocity units.
   * - ``gimbal_max_speed``
     - 180.0
     - Degrees/second.
   * - ``arm_max_linear_x`` / ``arm_max_linear_z``
     - 0.18 / 0.18
     - Metres/second; driver applies its increment/velocity bounds.
   * - ``stream_status_line`` / ``show_control_legend``
     - true / true
     - Terminal status and controller help.

``takeoff_height`` and ``require_deadman`` have the same defaults as the
launcher. ``robot_names`` and ``drone_names`` array parameters also exist, but
are declared with empty lists rather than explicit string-array types; prefer
the CSV parameters on Humble instead of assuming typed string-array overrides.
