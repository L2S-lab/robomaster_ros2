.. _faq:

Troubleshooting and current limits
==================================

No image or a low publication rate
----------------------------------

Enable the device's camera, select ``/<name>/image`` as a raw Image topic and
use Best Effort reliability. Reliable-only subscriptions do not match camera
QoS. Source the local workspace overlay after rebuilding, particularly when
switching from another checkout or an older camera implementation.

``publish_fps`` is an upper bound. Actual delivery also depends on camera FPS,
bitrate/resolution, Wi-Fi contention, decoding and subscriber load. Camera
messages use the node clock at publication, not a hardware capture timestamp.
The driver publishes raw images directly and starts no camera bridge process.
PyAV is required for video; the optional legacy SDK binding is for EP audio.

Fleet diagnostics say telemetry failed
--------------------------------------

The current fleet monitor requests reliable QoS, so best-effort images can have
zero counts despite a working camera. It can also subscribe to silent armor/IR
events and other command topics and then require a message. Use
``--no-topic-monitor`` for service-only diagnostics and interpret topic failures
with the limitations in :ref:`fleet_tools`.

An arm or joystick stream ignores an expected stop
--------------------------------------------------

The EP arm velocity stream is not guarded by ``set_armed`` and uses a ROS-clock
deadline without clearing cached velocity on expiry/disarm. The joystick node
has no input-loss watchdog and can continue republishing cached input; target
changes and shutdown do not explicitly zero the previous target. See
:ref:`howtos` and :ref:`fleet_tools` for the actual stop behavior. These are
current code limitations, not configuration options that enable hidden guards.

Gripper feedback is not a continuous percentage
-----------------------------------------------

Hardware reports opened/closed/normal. Intermediate percentages use a timing
estimate, including in feedback mode. Establish an endpoint after startup or
after the estimate is invalidated. Recalibrate the full-travel time when power
changes.

Missing endpoint or service failure
-----------------------------------

Endpoints depend on sensor flags and detected model capabilities. EP Core arm
services and EP/S1 gimbal services are not interchangeable. Battery requests
can fail before a DDS sample arrives. The server's AddRobot handler and drone
move/goto callbacks are unimplemented. Barometer/IMU unit handling and some
legacy arm feedback semantics remain incomplete.

Reporting problems
------------------

Include the source revision, device models/names, launch command, relevant
configuration, sourced workspace and complete error output. Use the project's
GitLab issue tracker at
https://gitlab-research.centralesupelec.fr/aarsht7/robomaster_ros2/-/issues.
