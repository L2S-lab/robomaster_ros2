.. _introduction:

ROS 2 driver for DJI RoboMaster series drones and robots
========================================================

robomaster_ros2 is a **work-in-progress** port of original `RoboMaster-SDK <https://github.com/dji-sdk/RoboMaster-SDK>`_ to ROS 2.
It is fully open-source and will be available on `github <https://github.com/L2S-lab/robomaster_ros2>`_. 

It is primarily made for research purposes. The package allows the operation of a team of RoboMaster Tello Talent (RMTT) drones and/or RoboMaster EP Core (RMEP) robots over the local wi-fi network. It opens up the possibilities of a heterogeneous network of robots and drones.

.. warning::
  robomaster_ros2 is already usable for most tasks, however configuration file formats might still change. While many features of original SDK are ported, there are currently still some limitations.
  
  - Tested only for small team sizes (less than 7 drones and 2 robots)


Contents
--------

.. toctree::
   installation
   overview
   usage
   simulation
   tutorials
   howto
   faq
   :maxdepth: 1