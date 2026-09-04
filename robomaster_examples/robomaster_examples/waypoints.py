from ament_index_python import get_package_share_directory
from std_srvs.srv import Trigger, Empty
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

KP = [0.6,0.6,0.75] #[0.6,0.6,0.75]
KI = [0.0,0.0,0.0]#[0.0001,0.0001,0.00001]
KD = [0.0,0.0,0.0]#[0.0001,0.0001,0.000001]

positions = []
error_individual = []

error_total = []
cmd_vel = []

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
        
        self.takeoff_client = self.create_client(Takeoff, f'/{self.robot_name}/takeoff')
        self.land_client = self.create_client(Trigger, f'/{self.robot_name}/land')
        # self.takeoff_client = self.create_client(Empty, f'/{self.robot_name}/takeoff')

        # self.land_client = self.create_client(Empty, f'/{self.robot_name}/land')
        while not (self.takeoff_client.wait_for_service(timeout_sec=1.0) and
                    self.land_client.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('takeoff and land service not available, waiting again...')
        
        self.Pterm = 0
        self.Iterm = 0
        self.Dterm = 0
        self.e_prev = [0,0,0]

        self.takeoff_req = Takeoff.Request()
        self.takeoff_req.height = self.get_parameter('takeoff_height').value
        self.land_req = Trigger.Request()
        self.accum_t = 0  
        #if self.robot_name == 'rmtt_2': time.sleep(1.5)
        self._timer = self.create_timer(1.0/self.frequency, self.control_loop)
        self.takeoff=False
        
        self.posePub = self.create_publisher(PoseStamped, f'/{self.robot_name}/timed_pose', 10)
        self.pose = PoseStamped()
        self.sub = self.create_subscription(PoseStamped, f'/{self.robot_name}/pose', self.pose_cb, 10)
        self.sub

    def pose_cb(self, msg:PoseStamped):
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.pose.position.x = msg.pose.position.x
        self.pose.pose.position.y = msg.pose.position.y
        self.pose.pose.position.z = msg.pose.position.z
        self.pose.pose.orientation.x = msg.pose.orientation.x
        self.pose.pose.orientation.y = msg.pose.orientation.y
        self.pose.pose.orientation.z = msg.pose.orientation.z
        self.pose.pose.orientation.w = msg.pose.orientation.w

    def get_transform(self, source_frame, target_frame):
        return self.tfBuffer.lookup_transform(source_frame, target_frame, rclpy.time.Time())

    def send_takeoff_request(self):
        time.sleep(self.get_parameter('time_to_takeoff').value)
        ret = self.takeoff_client.call(self.takeoff_req)
        _future = self.takeoff_client.call_async(self.takeoff_req)#.add_done_callback(self.takeoff_response)
        _future.add_done_callback
        self.get_logger().info(f'{ret}')
        self.get_logger().info('takeoff request sent!')
        if ret.success:
            self.get_logger().info('takeoff success!')
            
    def send_land_request(self):
        ret = self.land_client.call(self.land_req)
        self.get_logger().info(f'{ret}')
        self.get_logger().info('land request sent!')
        if ret.success:
            self.get_logger().info('land success!')

    def control_loop(self):
        dt = self._timer.time_since_last_call() / 1e9
        try:
            transform = self.get_transform(self.world_frame, self.tf_frame)
        except:
            transform = None
            return
        if not transform:
            return
        # if transform and not self.takeoff:
        #     self.send_takeoff_request()
        #     self.takeoff=True
        #     return
        # if transform and self.takeoff and transform.transform.translation.z<0.7:
        #     self.get_logger().info('waiting for takeoff to finish...')
        #     return
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
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.world_frame
        pose.pose.position.x = self.waypoints[0][0]
        pose.pose.position.y = self.waypoints[0][1]
        pose.pose.position.z = self.waypoints[0][2]
        self.pubPoint.publish(pose)
        self.posePub.publish(self.pose)
        positions.append(current)
        error_individual.append(np.array(self.e_prev))
        error_total.append(error)
        cmd_vel.append(np.array([cmd.linear.x, cmd.linear.y, cmd.linear.z]))
        self.get_logger().info(f'VX: {round(cmd.linear.x,2)}\tVY: {round(cmd.linear.y,2)}\tVZ: {round(cmd.linear.z,2)}\terror: {round(error,2)}\e_z: {round(e_z,2)}',throttle_duration_sec=1.0)
        if error < 0.2 and abs(e_z) < 0.2: #abs(e_y) < 0.1 and abs(e_x) < 0.1:
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
