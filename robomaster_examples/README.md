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
ros2 launch robomaster_examples execute_trajectory.launch.py
```


- for examples of waypoints, check 'waypoints/*.csv'
```
ros2 launch robomaster_examples waypoints.launch.py
```

## Five-drone/two-robot stress test

`fleet_stress_test` exercises an already-running RoboMaster server through its
public ROS topics and services. Its default fleet is `rmtt_1` through `rmtt_5`
and `rmep_1` through `rmep_2`.

The supplied static-IP example assumes the additional drones are
`192.168.0.154` and `192.168.0.155`. Verify every address in
`robomaster_ros2/config_ros/rmtt_param.yaml` before starting the server.

Build and source both affected packages:

```
colcon build --symlink-install \
  --packages-select robomaster_ros2 robomaster_examples
source install/setup.bash
```

Start the server with static assignment and keyboard flight shortcuts disabled:

```
ros2 launch robomaster_ros2 robomaster_server.launch.py \
  num_of_drones:=5 num_of_eps1:=2 \
  local_ip:=192.168.0.106 random_assign:=false keyboard_cmd:=false
```

Run the read-only test first. It concurrently checks serial, battery and state
services, subscribes to advertised telemetry, reports message rates, and returns
a non-zero exit status if a required check fails:

```
ros2 run robomaster_examples fleet_stress_test \
  --mode diagnostic --rounds 20 --duration 30
```

The ground test additionally arms each motion gate, exercises LEDs and drone
speed settings, and sends zero `cmd_vel` traffic to all seven drivers:

```
ros2 run robomaster_examples fleet_stress_test \
  --mode ground --rounds 20 --duration 30 --rate 20
```

Motion mode requires an exact acknowledgement. Clear and supervise the flight
and floor areas, provide separate takeoff positions, and keep the normal manual
emergency controls available:

```
ros2 run robomaster_examples fleet_stress_test \
  --mode motion --rounds 5 --duration 10 --rate 20 \
  --deadman-wait 1.0 \
  --confirm-motion MOVE_5_DRONES_2_ROBOTS
```

Motion mode uses a maximum default translation of `0.10 m/s` and a takeoff
height of `0.8 m`. It alternates commands to limit net displacement, then makes
one robot at a time silent so the operator can confirm its independent
stop/hover. `--deadman-wait` must be longer than the configured driver command
timeout. Zero commands, hover, landing, and disarming are attempted on every
normal or Ctrl-C exit path.

Emergency, soft-emergency, reboot, firing, close/remove, arm/gripper and gimbal
movement services are inventoried but deliberately not stress-invoked.
