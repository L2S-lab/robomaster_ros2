from ament_index_python import get_package_share_directory
from std_srvs.srv import Empty, Trigger
from robomaster_interface.srv import Takeoff
import time
import rclpy
import rclpy.time
from rclpy.node import Node
import numpy as np
from os.path import join

from geometry_msgs.msg import Twist
from tf2_ros.transform_listener import TransformListener, Buffer

from robomaster_examples.modules.Trajectory import Trajectory

class execute_trajectory(Node):
      
    def __init__(self):
        super().__init__('execute_trajectory')
        self.declare_parameters(
        namespace='',
        parameters=[
            ('trajectory', join(get_package_share_directory('robomaster_examples'), 'waypoints', 'multi_drone_spiral.yml')),
            ('world_frame', 'world'),
            ('frequency', 20 ),
            ('idx', 1),
            ('tf_frame', "rmtt_1"),
            ('robot_name', 'rmtt_1'),
            ('takeoff_height', 0.8),
            ('takeoff_time', 0.0),

        ])
        self.ready = False
        self.trajectory = self.get_parameter('trajectory').value
        self.world_frame = self.get_parameter('world_frame').value
        self.frequency = self.get_parameter('frequency').value
        self.idx = self.get_parameter('idx').value
        self.tf_frame = self.get_parameter('tf_frame').value
        self.robot = self.get_parameter('robot_name').value
        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.takeoff_time = self.get_parameter('takeoff_time').value

        self.pub = self.create_publisher(Twist, f'/{self.robot}/cmd_vel', 10)
        self.takeoff_cli = self.create_client(Takeoff, f'/{self.robot}/takeoff')
        self.hover_cli = self.create_client(Empty, f'/{self.robot}/hover')
        self.land_cli = self.create_client(Trigger, f'/{self.robot}/land')
        if not self.ready:
            while not self.hover_cli.wait_for_service(timeout_sec=1.0) \
                    or not self.land_cli.wait_for_service(timeout_sec=1.0):
                    #or not self.takeoff_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('landing or hover service not available, waiting again...')
        self.get_logger().info('service available')
        self._future = None
        self.ready = True
        # self.takeoff_req = Takeoff.Request()
        # self.takeoff_req.height = self.takeoff_height
        # self.takeoff_req.sync = False
        self.land_req = Trigger.Request()
        self.hover_req = Empty.Request()
            
        self.traj = Trajectory()
        self.traj.load(self.trajectory, i=self.idx)
        self.get_logger().info('Trajectory loaded')
        
        self.tfBuffer = Buffer()
        self.tfListnet = TransformListener(self.tfBuffer, self)
        self.prev_positions = np.zeros((4))

        self.time = 0
        self._timer = self.create_timer(1.0/self.frequency, self.control_loop)
        self.start_time = time.time()

    # def send_takeoff_request(self):
    #     time.sleep(self.takeoff_time)
    #     self._future = self.takeoff_cli.call_async(self.takeoff_req)
    #     self._future.add_done_callback(self.takeoff_response)

    # def takeoff_response(self, future):
    #     response = future.result()
    #     if response.success:
    #         self.get_logger().info('takeoff request sent!')
    #         self.ready = True
    #         self._future = None
    #     else:
    #         self.get_logger().info('takeoff request failed')
    #         self.ready = False
    #         self._future = None

    def send_land_request(self):
        self._future = self.land_cli.call_async(self.land_req)
        self._future.add_done_callback(self.land_response)

    def land_response(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info('land request sent')
            self.ready = False
            self._future = None
        else:
            self.get_logger().info('land request failed')
            self.ready = True
            self._future = None
    
    def send_hover_request(self):
        self.hover_cli.call_async(Empty.Request())
        self.ready = False

    def get_transform(self, source_frame, target_frame):
        return self.tfBuffer.lookup_transform(source_frame, target_frame, rclpy.time.Time())
    
    def control_loop(self):
        # if not self.ready and self._future is None:
        #     self.send_takeoff_request()
        # if not self.ready:
        #     pass
        if self.ready:
            dt = time.time()-self.start_time-self.time
            self.time = time.time()-self.start_time
            try:
                transform = self.get_transform(self.world_frame, self.tf_frame)
            except:
                transform = None
                return
            current = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
            current = np.append(current)
            u, _, _, _ = self.traj.get_control(self.time, dt, current, self.prev_positions)
            msg = Twist()
            try:
                if u is not None and self.ready:  
                    msg.linear.x = -1*max(min(u[0], 1.0), -1.0)
                    msg.linear.y = -1*max(min(u[1], 1.0), -1.0)
                    msg.linear.z = max(min(u[2], 1.0), -1.0)
                    
                self.pub.publish(msg)
                if u is None:
                    if self.ready:
                        self.send_hover_request()
                    time.sleep(0.2)
                    self.send_land_request()
                    self.get_logger().info('drone trajectory done')
                    self.destroy_node() 
            except KeyboardInterrupt:
                if self.ready:
                    self.send_land_request()
            self.prev_positions = current

def main(args=None):
    rclpy.init(args=args)
    trajectory = execute_trajectory()
    rclpy.spin(trajectory)
    trajectory.destroy_node()
    rclpy.shutdown()