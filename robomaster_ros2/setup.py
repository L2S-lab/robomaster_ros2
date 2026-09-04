from setuptools import setup
import glob

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
