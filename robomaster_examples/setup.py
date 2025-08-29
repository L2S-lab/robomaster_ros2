from setuptools import setup
import glob

package_name = 'robomaster_examples'

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
        ('share/' + package_name + '/waypoints', glob.glob('waypoints/*.csv')),
        ('share/' + package_name + '/waypoints', glob.glob('waypoints/*.wps')),
        ('share/' + package_name + '/waypoints', glob.glob('waypoints/*.yml')),
        ('share/' + package_name + '/waypoints', glob.glob('waypoints/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aarsh',
    maintainer_email='aarsh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "waypoints = robomaster_examples.waypoints:main",  
            "execute_trajectory = robomaster_examples.execute_trajectory:main",
            "collission_check = robomaster_examples.collission_check:main",
        ],
    },
)
