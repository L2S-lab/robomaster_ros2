# robomaster_examples

### Current work

- trajectory and waypoint tracking for drones
- collission check for trajectory
- trajectory as a funtion of time


### How to use
- build pkg
```
colcon build --symlink-install --packages-select robomaster_examples
source install/setup.bash
```

- for examples of trajectoy, check 'waypoints/*.yml' files
- for collission check in trajectory and simple visualisation, 
```
ros2 launch robomaster_examples collission_check.launch.py nb_drones:=3
```

- to execute the trajectory on real drones,
```
ros2 service call /takeoff_all std_srvs/srv/Trigger "{}"
ros2 launch robomaster_ros2_examples execute_trajectory.launch.py
```


- for examples of waypoints, check 'waypoints/*.wps' files
```
ros2 service call /takeoff_all std_srvs/srv/Trigger "{}"
ros2 launch robomaster_examples waypoints.launch.py
```