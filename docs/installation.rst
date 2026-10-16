.. _installation:

Installation
============

The hardware driver targets Ubuntu 22.04, Python 3.10 and ROS 2 Humble. Gazebo
is not required to connect to physical robots or run the hardware examples.

Dependencies
------------

Install ROS 2 Humble first, then install the driver/example dependencies:

.. code-block:: bash

   source /opt/ros/humble/setup.bash
   sudo apt install network-manager python3-av python3-netifaces python3-numpy \
     python3-matplotlib python3-pip \
     python3-yaml python3-opencv python3-colcon-common-extensions \
     ros-humble-tf-transformations ros-humble-cv-bridge \
     ros-humble-tf2-ros ros-humble-tf2-geometry-msgs ros-humble-nav-msgs \
     ros-humble-joy
   python3 -m pip install pillow myqr pynput

``joy`` supplies the joystick input node. It must be installed explicitly for
the current teleoperation launcher. Cameras use PyAV to decode the device's
H.264 stream and publish raw ROS images directly; no compression bridge is
required. Package manifests declare dependencies but ``colcon build`` does not
install system packages.

Optional EP audio support uses the legacy SDK binding:

.. code-block:: bash

   sudo apt install libopus-dev
   python3 -m pip install git+https://github.com/aarsht7/RoboMaster-SDK.git@libmedia_codec

That binding is not the video decoder. The driver uses PyAV for video to avoid
the older H.264 binding's native-memory failures.

Source and build
----------------

For a new workspace, obtain the source:

.. code-block:: bash

   mkdir -p ros2_ws/src
   cd ros2_ws/src
   git clone https://gitlab-research.centralesupelec.fr/aarsht7/robomaster_ros2.git
   cd ..

For an existing workspace, start in its root directory, which contains ``src``.
Build the hardware packages and their dependencies:

.. code-block:: bash

   source /opt/ros/humble/setup.bash
   colcon build --symlink-install --packages-up-to robomaster_ros2 robomaster_examples
   source install/setup.bash

The build includes ``robomaster_interface``. It does not select the simulation
package. Source the ROS setup and this workspace's overlay in each terminal
used for the driver or examples. Avoid mixing overlays from another checkout.

After changing interface definitions, rebuild the interface and its consumers.
With symlink installation, Python source is linked where supported; rebuild
after changing installed resources, package metadata or executable registration.

Next, configure the network and devices using :ref:`configuration`, then follow
:ref:`howtos` to start the server.
