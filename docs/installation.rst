.. _installation:

Installation
============

robomaster_ros2 runs on **Ubuntu Linux**, tested with following configuration:

====== ======== ====== 
Ubuntu Python   ROS 2
------ -------- ------
22.04  3.10     Humble
====== ======== ======

.. warning::
   Avoid using a virtual machine if possible: they add additional latency and might cause issues with the visualization tools.

First Installation
------------------

1. If needed, install ROS 2 using the instructions at https://docs.ros.org/en/humble/Installation.html.

2. If needed, install gzsim Harmonic using this instruction at https://gazebosim.org/docs/latest/ros_installation/#gazebo-harmonic-with-ros-2-humble-or-rolling-use-with-caution

3. Install dependencies

   .. code-block:: bash
      pip install pillow myqr pynput
      sudo apt-get install network-manager libopus-dev
      sudo apt install ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-nav-msgs

   Optional Installation for video streaming

   .. code-block:: bash
      pip install git+https://github.com/aarsht7/RoboMaster-SDK.git@libmedia_codec

4. Set up your ROS 2 workspace

    .. code-block:: bash

        mkdir -p ros2_ws/src
        cd ros2_ws/src
        git clone https://github.com/L2S-lab/robomaster_ros2.git

5. Build your ROS 2 workspace

    .. code-block:: bash

        cd ../
        source /opt/ros/$ROS_DISTRO/setup.bash
        colcon build --symlink-install robomaster_interface robomaster_ros2 robomaster_examples robomaster_gz

    .. note::
       symlink-install allows you to edit Python and config files without running `colcon build` every time.

    .. note::
       If you install it for the first time, it will check for some dependencies
       As long as the build of the package finish, you can ignore any other warnings.