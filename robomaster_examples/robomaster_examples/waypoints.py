from ament_index_python import get_package_share_directory
from std_srvs.srv import Trigger
from robomaster_interface.srv import Takeoff
import time
import rclpy
import rclpy.time
from rclpy.node import Node
import numpy as np
import os

from tf2_geometry_msgs import PoseStamped
from tf2_ros.transform_listener import TransformListener, Buffer
from geometry_msgs.msg import Twist

# PID constants change according to the need
KP = [0.6,0.6,0.75] #[0.6,0.6,0.75]
KI = [0.0,0.0,0.0] #[0.0001,0.0001,0.00001]
KD = [0.0,0.0,0.0] #[0.0001,0.0001,0.000001]

# Error Thresholds
E = [0.2,0.2] # [xy,z] error thresholds

class Waypoints(Node):
      
    def __init__(self):
        super().__init__('waypoints')
        self.declare_parameters(
        namespace='',
        parameters=[
            ('world_frame', 'world'),
            ('tf_frame', "rmtt_1"),
            ('frequency', 20 ),
        ])
        self.declare_parameter('robot_name', 'rmtt_1')
        self.declare_parameter('takeoff_height', 1.0)
        self.declare_parameter('time_to_takeoff', 0.0)
        self.declare_parameter('waypoints','drone_1.wps')

        self.world_frame = self.get_parameter('world_frame').value
        self.tf_frame = self.get_parameter('tf_frame').value
        self.frequency = self.get_parameter('frequency').value
        self.robot_name = self.get_parameter('robot_name').value
        self.waypoints = self.get_parameter('waypoints').value
        if self.waypoints.endswith('.wps'):
            self.waypoints = get_package_share_directory('robomaster_examples')+'/waypoints/'+self.waypoints
            if os.path.isfile(self.waypoints):
                    self.waypoints = np.loadtxt(self.waypoints, delimiter=',', dtype=float)
            else:
                    self.get_logger().error('waypoints file does not exist')
        else:
            self.get_logger().error('waypoints file must be a .wps file with comma separated values')
        self.get_logger().info(f'waypoints: {self.waypoints}')
        self.tfBuffer = Buffer()
        self.tfListnet = TransformListener(self.tfBuffer, self)
        
        self.pubCmdVel = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        self.pubPoint = self.create_publisher(PoseStamped, f'/{self.robot_name}/waypoint', 10)
        
        self.land_client = self.create_client(Trigger, f'/{self.robot_name}/land')

        while not (self.land_client.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('land service not available, waiting again...')

        self.Pterm = 0
        self.Iterm = 0
        self.Dterm = 0
        self.e_prev = [0,0,0]

        self.land_req = Trigger.Request()
        self._timer = self.create_timer(1.0/self.frequency, self.control_loop)

    def get_transform(self, source_frame, target_frame):
        return self.tfBuffer.lookup_transform(source_frame, target_frame, rclpy.time.Time())
            
    def send_land_request(self):
        ret = self.land_client.call(self.land_req)
        self.get_logger().info(f'{ret}')
        self.get_logger().info('land request sent!')

    def control_loop(self):
        dt = self._timer.time_since_last_call() / 1e9
        try:
            transform = self.get_transform(self.world_frame, self.tf_frame)
        except:
            transform = None
            return
        if not transform:
            return
        current = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
        error = np.linalg.norm(self.waypoints[0] - current)
        e_x = self.waypoints[0][0] - current[0]
        e_y = self.waypoints[0][1] - current[1]
        e_z = self.waypoints[0][2] - current[2]
        cmd = Twist()
        cmd.linear.x = -1*max(min(self.calc_pid(e_x, 1, dt), 0.8), -0.8)
        cmd.linear.y = -1*max(min(self.calc_pid(e_y, 0, dt), 0.8), -0.8)
        cmd.linear.z = max(min(self.calc_pid(e_z, 2, dt), 0.8), -0.8)
        self.e_prev = [e_x, e_y, e_z]
        self.pubCmdVel.publish(cmd)
        if error < E[0] and abs(e_z) < E[1]: 
            self.get_logger().info(f'Error: {error}')
            self.waypoints = np.delete(self.waypoints, 0, 0)
            if len(self.waypoints) == 0:
                self.send_land_request()
                self._timer.destroy()

    def calc_pid(self, e, i, dt):
        self.Pterm = KP[i] * e
        self.Iterm += e * dt
        self.Dterm = KD[i]*(e - self.e_prev[i]) / dt
        return self.Pterm + KI[i]*self.Iterm + self.Dterm

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
