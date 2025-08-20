from setuptools import setup
import glob

try:
    import libmedia_codec
except ImportError:
    from rclpy.logging import get_logger
    logger = get_logger('Build')
    from subprocess import check_output
    if check_output(["apt","list","--installed","libopus-dev"]).decode('utf-8').find("libopus-dev") == -1:
        logger.error("libopus-dev not found. Please make sure you have libopus-dev installed. If not, you can do it with\n \"sudo apt install libopus-dev -y\" ")
        raise ImportError("libopus-dev not found")
    logger.error("libmedia_codec not found. Please make sure you have pip3 and libopus-dev installed and run the following command: \n pip3 install git+https://github.com/aarsht7/RoboMaster-SDK.git@libmedia_codec")
    #print("Running following commands: \n pip3 install git+https://github.com/aarsht7/RoboMaster-SDK.git@libmedia_codec")
    #import os
    #os.system("sudo apt install libopus-dev -y")
    #os.system("pip3 install git+https://github.com/aarsht7/RoboMaster-SDK.git@libmedia_codec")

package_name = 'robomaster_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,package_name + '.modules'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/config_ros', glob.glob('config_ros/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aarsh',
    maintainer_email='aarsh@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "setup_wifi = robomaster_ros2.setup_wifi:main",
            "retrive_robot_info = robomaster_ros2.retrive_robot_info:main",
            "param_server = robomaster_ros2.param_server:main",
            "robomaster_server = robomaster_ros2.robomaster_server:main",
        ],
    },
)
