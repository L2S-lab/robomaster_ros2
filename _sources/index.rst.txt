.. _introduction:

ROS 2 driver for DJI RoboMaster drones and robots
=================================================

This repository provides ROS 2 Humble hardware drivers for RoboMaster Tello
Talent (RMTT) drones and EP/Core/S1 robots. Each device has its own namespace,
with a server for discovery, configuration and fleet services.

The hardware camera path publishes raw ``sensor_msgs/msg/Image`` messages
directly. EP Core arm/gripper services, chassis telemetry, armor events,
fleet diagnostics and joystick examples are included. Model support and enabled
sensors determine which ROS endpoints exist.

These pages describe the current local implementation. Example addresses,
camera settings and fleet sizes are illustrative; use the configuration files
in your checkout for the selected values. Known gaps in arm streaming, joystick
stop behavior and fleet telemetry monitoring are documented alongside those
features.

Contents
--------

.. toctree::
   :maxdepth: 1

   installation
   overview
   configuration
   howto
   fleet_tools
   tutorials
   faq
   simulation
