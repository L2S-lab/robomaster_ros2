.. _tutorials:

Command-line and trajectory examples
====================================

Start the configured hardware server first. The examples below use ``rmtt_1``
and ``rmep_1``; substitute the actual device namespaces. Motion gate settings
and current example limitations are described in :ref:`configuration` and
:ref:`fleet_tools`.

Drone services
--------------

.. code-block:: bash

   ros2 service call /rmtt_1/set_armed std_srvs/srv/SetBool "{data: true}"
   ros2 service call /rmtt_1/takeoff robomaster_interface/srv/Takeoff "{height: 1.0, sync: false}"
   ros2 service call /rmtt_1/hover std_srvs/srv/Empty "{}"
   ros2 service call /rmtt_1/land std_srvs/srv/Trigger "{}"

``/takeoff_all`` and ``/land_all`` use Trigger and are created only while the
server has registered drones. ``soft_emergency`` uses Trigger;
``emergency`` and ``reboot`` use Empty. Emergency requests motor shutdown,
rather than a normal landing.

To add a drone dynamically, supply its name/address and camera settings to
AddDrone. Remove it with RemoveRobot and type ``rmtt``:

.. code-block:: bash

   ros2 service call /add_drone robomaster_interface/srv/AddDrone "{name: 'rmtt_extra', ip: '192.168.0.159', cam: false, cam_direction: 0, ext_pose_topic_name: '', ext_pose_topic_type: ''}"
   ros2 service call /remove_robot robomaster_interface/srv/RemoveRobot "{name: 'rmtt_extra', type: 'rmtt'}"

Velocity commands
-----------------

Ground cmd_vel uses m/s and rad/s. Drone cmd_vel uses normalized RC inputs in
[-1, 1], with X forward, Y left, Z up and positive yaw counterclockwise. Stream
commands within the configured deadline and publish an explicit zero to stop:

.. code-block:: bash

   ros2 service call /rmep_1/set_armed std_srvs/srv/SetBool "{data: true}"
   ros2 topic pub --once /rmep_1/cmd_vel geometry_msgs/msg/Twist "{}"
   ros2 topic pub --once /rmtt_1/cmd_vel geometry_msgs/msg/Twist "{}"

An external controller must convert its output to the drone's normalized input
range. Subscribing to an external Pose/PoseStamped/Point/PointStamped topic does
not implement the drone ``move`` or ``goto`` services.

Existing example executables
----------------------------

.. list-table::
   :header-rows: 1

   * - Executable / launch file
     - Purpose
   * - ``waypoints`` / ``waypoints.launch.py``
     - Waypoint tracking with external positioning.
   * - ``execute_trajectory`` / ``execute_trajectory.launch.py``
     - Execute configured time-based trajectories.
   * - ``collission_check`` / ``collission_check.launch.py``
     - Collision checking and trajectory visualization; spelling matches the executable.
   * - ``fleet_stress_test``
     - Concurrent public-API diagnostics and optional ground/motion runs.
   * - ``joy_fleet_teleop`` / ``joy_fleet_teleop.launch.py``
     - Joystick control of a selected robot or drone.

Waypoint CSV/WPS and multi-drone YAML examples live in
``robomaster_examples/waypoints``. Configure the example's frames, namespaces and
input files before launching; external positioning is required by the tracking
controllers. These examples command the existing cmd_vel interface, rather than
using the unimplemented drone move/goto callbacks. See :ref:`fleet_tools` for
the newer diagnostic and joystick tools.
