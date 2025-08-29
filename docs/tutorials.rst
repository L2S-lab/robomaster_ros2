.. _tutorials:

Tutorials
=========

Command line
------------

Drones
^^^^^^
- Takeoff all the connected drones
.. code-block:: bash

   ros2 service call /takeoff_all std_srvs/srv/Trigger "{}"

- Land all the connected drones
.. code-block:: bash

   ros2 service call /land_all std_srvs/srv/Trigger "{}"

- Takeoff individual drones
.. code-block:: bash

   ros2 service call /drone_1/takeoff robomaster_interface/srv/Takeoff "{height: 0.0}"

- Land individual drones
.. code-block:: bash

   ros2 service call /drone_1/land std_srvs/srv/Trigger "{}"

- Soft emergency landing
.. code-block:: bash

   ros2 service call /drone_1/soft_emergency std_srvs/srv/Trigger "{}"

- Emergency shutdown
.. code-block:: bash

   ros2 service call /drone_1/emergency std_srvs/srv/Empty "{}"

- Reboot drone
.. code-block:: bash

    ros2 service call /drone_1/reboot std_srvs/srv/Empty "{}"

- Add drone to the server
.. code-block:: bash

    ros2 service call /add_drone robomaster_interface/srv/AddDrone "
    name: <drone_name_str>
    ip: <drone_ip_str>
    "

- Remove drone from the server
.. code-block:: bash

    ros2 service call /remove_drone robomaster_interface/srv/RemoveRobot "
    name: <drone_name_str>
    type: 'rmtt'
    "







Control drones with external positioning system
-----------------------------------------------

Drones have velocity command that can be used to control the drone. However, to use this command, the controller needs to know drone's current position. This can be achieved by using an external positioning system such as Vicon or OptiTrack. The following example shows how to use the external positioning system to control the drone's position with simple PID control.

More complex examples are included in the `robomaster_examples` package.

.. code-block:: python

    from std_srvs.srv import Trigger
    import rclpy
    from rclpy.node import Node
    import numpy as np
    import os

    from tf2_ros.transform_listener import TransformListener, Buffer
    from geometry_msgs.msg import Twist

    KP = [0.6,0.6,0.75] #[0.6,0.6,0.75]
    KI = [0.0,0.0,0.0]#[0.0001,0.0001,0.00001]
    KD = [0.0,0.0,0.0]#[0.0001,0.0001,0.000001]

    class Waypoints(Node):
    # First takeoff the drone before running the script
    def __init__(self):
        super().__init__('waypoints')

        self.world_frame = "world"
        self.tf_frame = "rmtt_1"
        self.frequency = 20
        self.robot_name = "rmtt_1"
        self.waypoints = np.array([[0,0,1],[1,0,1],[1,1,1],[0,1,1],[0,0,1],[0,0,0.5]])

        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)
        
        self.pubCmdVel = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        
        self.land_client = self.create_client(Trigger, f'/{self.robot_name}/land')

        while not (self.land_client.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('land service not available, waiting again...')
        
        self.Iterm = np.zeros(3)
        self.e_prev = np.zeros(3)

        self.land_req = Trigger.Request()
        self._timer = self.create_timer(1.0/self.frequency, self.control_loop)

    def get_transform(self, source_frame, target_frame):
        return self.tfBuffer.lookup_transform(source_frame, target_frame, rclpy.time.Time())
            
    def send_land_request(self):
        self.land_client.call_async(self.land_req)
        self.get_logger().info('Land request sent!')

    def control_loop(self):
        dt = self._timer.time_since_last_call() / 1e9
        if dt == 0: return # Avoid division by zero on the first call

        try:
            transform = self.get_transform(self.world_frame, self.tf_frame)
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')
            return

        current_pos = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        
        target_pos = self.waypoints[0]
        error_pos = target_pos - current_pos
        
        # --- PID Calculation for each axis ---
        # Proportional
        p_term = KP * error_pos
        
        # Integral
        self.Iterm += error_pos * dt
        i_term = KI * self.Iterm
        
        # Derivative
        d_term = KD * (error_pos - self.e_prev) / dt
        
        # Update previous error
        self.e_prev = error_pos
        
        # --- Calculate velocity command ---
        pid_result = p_term + i_term + d_term
        
        cmd = Twist()
        # Clamp the output velocity to a safe range
        cmd.linear.x = np.clip(pid_result[0], -0.8, 0.8)
        cmd.linear.y = np.clip(pid_result[1], -0.8, 0.8)
        cmd.linear.z = np.clip(pid_result[2], -0.8, 0.8)
        
        self.pubCmdVel.publish(cmd)

        # --- Waypoint switching logic ---
        dist_to_target = np.linalg.norm(error_pos)
        if dist_to_target < 0.2: 
            self.get_logger().info(f'Reached waypoint {target_pos}, distance: {dist_to_target:.2f}m')
            self.waypoints = np.delete(self.waypoints, 0, 0)
            
            # Reset PID state to prevent integral windup from affecting the next waypoint
            self.Iterm = np.zeros(3)
            self.e_prev = np.zeros(3)

            if len(self.waypoints) == 0:
                self.get_logger().info('All waypoints reached. Landing...')
                self.send_land_request()
                self._timer.destroy()

    def main(args=None):
        rclpy.init(args=args)
        waypoints = Waypoints()
        try:
            rclpy.spin(waypoints)
        except KeyboardInterrupt:
            waypoints.send_land_request()
            pass
        waypoints.destroy_node()
        rclpy.shutdown()

