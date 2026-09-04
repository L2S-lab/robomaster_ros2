import rclpy
#import rclpy.client
from rclpy.node import Node
#from rclpy.node import Client as rclpy_client
from rclpy.task import Future
from rclpy import Parameter
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from robomaster_interface.srv import AddRobot, RemoveRobot, AddDrone, Takeoff

from robomaster_ros2.modules.drone import Drone, TextClient
from robomaster_ros2.modules.robot import scan_robot_ip_list, Robot, enable_sdk_robot, Client
from robomaster_ros2.modules.common import Connection
from robomaster_ros2.modules import config
from robomaster_ros2.modules import globals
from robomaster_ros2.modules.util import call_get_parameters, enable_sdk_drone


from random import sample
from typing import Dict
import time
import os
import netifaces
from pynput import keyboard
import threading
from functools import wraps

class RobomasterServer(Node):

    def __init__(self, executor):
        super().__init__('robomaster_server',
                        allow_undeclared_parameters=True,
                        start_parameter_services=True,)
        if os.path.exists(os.getcwd()+'/.tmp'):
            os.system('rm -rf '+os.getcwd()+'/.tmp')

        self.declare_parameters(
           namespace='',
           parameters=[
                ('local_ip', Parameter.Type.STRING),
                ('num_of_drones', Parameter.Type.INTEGER),
                ('num_of_eps1', Parameter.Type.INTEGER),
                ('random_assign', Parameter.Type.BOOL),
                ('keyboard_cmd', Parameter.Type.BOOL),
           ])
        self.executor = executor

        self.add_robot_srv = self.create_service(AddRobot, 'add_robot', self.add_robot)
        self.add_drone_srv = self.create_service(AddDrone, 'add_drone', self.add_drone)
        self.rmv_robot_srv = self.create_service(RemoveRobot, 'remove_robot', self.rmv_robot)

        self.num_of_drones = self.get_parameter('num_of_drones').value
        if self.num_of_drones>1:
            self.drone_local_port = sample(range(globals.TELLO_SDK_PORT_MIN, globals.TELLO_SDK_PORT_MAX), self.num_of_drones)
            self.drone_video_port = sample(range(globals.TELLO_VIDEO_PORT_MIN, globals.TELLO_VIDEO_PORT_MAX), self.num_of_drones)
        else:
            self.drone_local_port = [globals.TELLO_SDK_PORT_MIN]
            self.drone_video_port = [globals.TELLO_VIDEO_PORT]
        
        self.num_of_eps1 = self.get_parameter('num_of_eps1').value
        if self.num_of_eps1>1:
            self.robot_local_port = sample(range(globals.ROBOT_SDK_PORT_MIN, globals.ROBOT_SDK_PORT_MAX), self.num_of_eps1)
            #self.robot_video_port = sample(range(globals.ROBOT_VIDEO_PORT_MIN, globals.ROBOT_VIDEO_PORT_MAX), self.num_of_eps1)
        else:
            self.robot_local_port = [globals.ROBOT_SDK_PORT_MIN]
            #self.robot_video_port = [globals.ROBOT_VIDEO_PORT_MIN]

        self.random_assign = self.get_parameter('random_assign').value
        self.local_ip = self.get_parameter('local_ip').value
        self.keyboard_cmd = self.get_parameter('keyboard_cmd').value
        if self.local_ip not in [ip['addr'] for interface in netifaces.interfaces() for ip in netifaces.ifaddresses(interface).get(netifaces.AF_INET, [])]:
            self.get_logger().error(f'Invalid local IP address: {self.local_ip}')
            self.destroy_node()
            rclpy.shutdown()

        if self.num_of_drones > 0:
            self.base_params_drone = call_get_parameters(self,'param_server', 'rmtt',['mled_display_num','external_position',
                                                                    'drone_name_list','pub_front_tof','pub_bottom_tof',
                                                                    'pub_attitude','pub_barometer','pub_imu','pub_mpad',
                                                                    'camera.fps','camera.bitrate',
                                                                    'camera.resolution','camera.publish_fps',
                                                                    'safety.command_timeout','safety.require_arm',
                                                                    'safety.latch_deadman'])
        if self.num_of_eps1 > 0:
            self.base_params_robot = call_get_parameters(self,'param_server', 'rmep',['external_position', 'led_num',
                                                                    'robot_name_list','pub_imu','pub_cam','pub_marker',
                                                                    'pub_armpose','pub_position','pub_gimbal_angle',
                                                                    'camera.resolution',
                                                                    'camera.publish_fps','armor.enabled',
                                                                    'gripper.control_mode','gripper.power',
                                                                    'gripper.full_travel_time',
                                                                    'gripper.feedback_timeout',
                                                                    'arm.completion_timeout','safety.command_timeout',
                                                                    'safety.require_arm','safety.latch_deadman'])
        if self.keyboard_cmd:
            self.keyboard_paused = False
            self.armed = False
            self.hotkeys = keyboard.GlobalHotKeys({
                '<alt>+t': self.on_activate_t,
                '<alt>+e': self.on_activate_e,
                '<alt>+l': self.on_activate_l,
                '<alt>+a': self.on_activate_a,
            })
        self.drones: Dict[str, Node] = {}
        self.all_drones: Dict[str, Drone] = {}
        self.robots: Dict[str, Node] = {}
        self.all_robots: Dict[str, Robot] = {}
        self.takeoff_srv = None
        self.land_srv = None
        if self.num_of_drones>0:
            self.drone_server()
        self._update_drone_fleet_services()
        if self.num_of_eps1>0:
            self.robot_server()
        if self.keyboard_cmd:
            self.keyboard_listen = threading.Thread(
                target=self.run_hotkeys, daemon=True
            )
            self.keyboard_listen.start()
        #self._timer = self.create_timer(0.1, self.timer_callback)

    def drone_server(self):
        if self.keyboard_cmd:
            msg='''
            \n
            Press <alt>+a to arm all drones
            Press <alt>+e to emergency all drones
            Press <alt>+l to land all drones
            Prest <alt>+t to takeoff all drones
            \n
            '''
            self.get_logger().info(msg,once=True)
        params = {"ext_pose": self.base_params_drone['external_position'],
                  "front_tof": self.base_params_drone['pub_front_tof'],
                  "bottom_tof": self.base_params_drone['pub_bottom_tof'],
                  "attitude": self.base_params_drone['pub_attitude'],
                  "baro": self.base_params_drone['pub_barometer'],
                  "imu": self.base_params_drone['pub_imu'],
                  "cam": False,
                  "camera_fps": self.base_params_drone['camera.fps'],
                  "camera_bitrate": self.base_params_drone['camera.bitrate'],
                  "camera_resolution": self.base_params_drone['camera.resolution'],
                  "camera_publish_fps": self.base_params_drone['camera.publish_fps'],
                  "mpad": self.base_params_drone['pub_mpad'],
                  "safety_command_timeout": self.base_params_drone['safety.command_timeout'],
                  "safety_require_arm": self.base_params_drone['safety.require_arm'],
                  "safety_latch_deadman": self.base_params_drone['safety.latch_deadman'],
                  "topic_name": None, "topic_type": None, "cam_direction": 0}
        if self.random_assign:
            conn = Connection()
            host_list = conn.scan_multi_drones(self.num_of_drones, local_ip=self.local_ip)
            drone_ip_list = [host[0] for host in host_list]
            drone_ip_list = sorted(drone_ip_list, key=lambda x: int(x.split('.')[-1]))
            conn.close()
        for i in range(self.num_of_drones):
            drone_name = self.base_params_drone['drone_name_list'][i]
            drone_param = call_get_parameters(self,'param_server','rmtt',[f'{drone_name}.pub_cam',f'{drone_name}.cam_direction',
                                                                    f'{drone_name}.yaw_hold',f'{drone_name}.ext_pose_topic_type',
                                                                    f'{drone_name}.ext_pose_topic_name'])
            if self.random_assign:
                ip = drone_ip_list[i]
            else:
                ip = call_get_parameters(self,'param_server','rmtt',[f'{drone_name}.ip'])[f'{drone_name}.ip']
            
            self.create_drone(ip, drone_name, i, params, drone_param)
    
    def create_drone(self, ip, drone_name, number, params, drone_param=None):
        params = params.copy()
        time.sleep(0.3)
        self.get_logger().info(f"Enabling SDK mode on drone with ip: {ip}")
        if enable_sdk_drone(ip):
            node = rclpy.create_node(drone_name, namespace=drone_name)
            self.drones[drone_name] = node
            te_conf = config.Config(drone_name)
            te_conf.default_robot_addr = (ip, globals.TELLO_DEVICE_PORT)
            te_conf.product = "TelloEdu"
            te_conf.cmd_proto = "text"  
            te_conf.number = number+1
            te_conf.default_sdk_addr = (self.local_ip, self.drone_local_port[number])
            if drone_param is not None:
                params['yaw_hold'] = drone_param[f'{drone_name}.yaw_hold']
                if params['ext_pose']:
                    params['topic_name'] = drone_param[f'{drone_name}.ext_pose_topic_name']
                    params['topic_type'] = drone_param[f'{drone_name}.ext_pose_topic_type']
                if drone_param[f'{drone_name}.pub_cam']:
                    params['cam'] = True
                    params['cam_direction'] = drone_param[f'{drone_name}.cam_direction']
            te_conf.video_stream_addr = (
                self.local_ip, self.drone_video_port[number]
            )
            te_conf.video_stream_proto = "udp"
            sdk_conn = Connection(conf=te_conf)
            cli = TextClient(conn=sdk_conn)
            drone = Drone(node,conf=te_conf, cli=cli, params=params)
            ret = drone._initialize()
            time.sleep(0.3)
            if ret:
                self.get_logger().info(f"Robot initialized: {drone_name} : {drone._get_sn()}")
                time.sleep(0.3)
                if self.base_params_drone['mled_display_num']:
                    if not drone._conf.number >9:
                        drone._set_mled(cmd=f"EXT mled s r {str(drone._conf.number)}")# async")
                if params['cam']:
                    if not drone.camera.set_fps(params['camera_fps']):
                        self.get_logger().warning(
                            f"Failed to set {drone_name} camera FPS to "
                            f"{params['camera_fps']}"
                        )
                    if not drone.camera.set_bitrate(params['camera_bitrate']):
                        self.get_logger().warning(
                            f"Failed to set {drone_name} camera bitrate to "
                            f"{params['camera_bitrate']}"
                        )
                    if not drone.camera.set_resolution(
                            params['camera_resolution']):
                        self.get_logger().warning(
                            f"Failed to set {drone_name} camera resolution to "
                            f"{params['camera_resolution']}"
                        )
                    if params["cam_direction"] == 1:
                        drone.camera.set_down_vision(1)
                    time.sleep(0.1)
                    if not drone.start_video(
                            publish_fps=params['camera_publish_fps']):
                        self.get_logger().error(
                            f"Failed to start camera stream for {drone_name}"
                        )
                self.all_drones[drone_name] = drone
                self.executor.add_node(node)   
                time.sleep(1)
                return True   
            else:
                self.get_logger().error(f"Failed to initialize drone: {drone_name}")
                return False 
        
    def add_drone(self, request, response):
        _name = request.name
        _ip = request.ip
        _cam = request.cam
        _dir = request.cam_direction
        _ext_pose_name = request.ext_pose_topic_name
        _ext_pose_type = request.ext_pose_topic_type
        if _name in self.drones.keys():
            response.success = False
            response.message = f'{_name} already exists'
            return response
        params = {"ext_pose": self.base_params_drone['external_position'],
                "front_tof": self.base_params_drone['pub_front_tof'],
                "bottom_tof": self.base_params_drone['pub_bottom_tof'],
                "attitude": self.base_params_drone['pub_attitude'],
                "baro": self.base_params_drone['pub_barometer'],
                "imu": self.base_params_drone['pub_imu'],
                "cam": _cam,
                "camera_fps": self.base_params_drone['camera.fps'],
                "camera_bitrate": self.base_params_drone['camera.bitrate'],
                "camera_resolution": self.base_params_drone['camera.resolution'],
                "camera_publish_fps": self.base_params_drone['camera.publish_fps'],
                "mpad": self.base_params_drone['pub_mpad'],
                "safety_command_timeout": self.base_params_drone['safety.command_timeout'],
                "safety_require_arm": self.base_params_drone['safety.require_arm'],
                "safety_latch_deadman": self.base_params_drone['safety.latch_deadman'],
                "topic_name": _ext_pose_name, "topic_type": _ext_pose_type, 
                "cam_direction": _dir}
        self.num_of_drones += 1
        #self.base_params_drone['drone_name_list'].append(_name)
        self.drone_local_port = self.add_port(self.drone_local_port, globals.TELLO_SDK_PORT_MIN, globals.TELLO_SDK_PORT_MAX)
        self.drone_video_port = self.add_port(self.drone_video_port, globals.TELLO_VIDEO_PORT_MIN, globals.TELLO_VIDEO_PORT_MAX)
        response.success= self.create_drone(_ip, _name, self.num_of_drones-1, params)
        response.message = f'Drone {_name} added'
        self._update_drone_fleet_services()
        return response

    def robot_server(self):
        params = {"ext_pose": self.base_params_robot['external_position'],
                  "imu": self.base_params_robot['pub_imu'],
                  "cam": self.base_params_robot['pub_cam'],
                  "marker": self.base_params_robot['pub_marker'],
                  "arm_position": self.base_params_robot['pub_armpose'],
                  "chassis_position": self.base_params_robot['pub_position'],
                  "gimbal_angle": self.base_params_robot['pub_gimbal_angle'],
                  "camera_resolution": self.base_params_robot['camera.resolution'],
                  "camera_publish_fps": self.base_params_robot['camera.publish_fps'],
                  "armor_enabled": self.base_params_robot['armor.enabled'],
                  "gripper_control_mode": self.base_params_robot['gripper.control_mode'],
                  "gripper_power": self.base_params_robot['gripper.power'],
                  "gripper_full_travel_time": self.base_params_robot['gripper.full_travel_time'],
                  "gripper_feedback_timeout": self.base_params_robot['gripper.feedback_timeout'],
                  "arm_completion_timeout": self.base_params_robot['arm.completion_timeout'],
                  "safety_command_timeout": self.base_params_robot['safety.command_timeout'],
                  "safety_require_arm": self.base_params_robot['safety.require_arm'],
                  "safety_latch_deadman": self.base_params_robot['safety.latch_deadman'],}
        if self.random_assign:
            robot_ip_list = scan_robot_ip_list(self.num_of_eps1, timeout=3*self.num_of_eps1)
            robot_ip_list = sorted(robot_ip_list, key=lambda x: int(x.split('.')[-1]))
        for i in range(self.num_of_eps1):
            robot_name = self.base_params_robot['robot_name_list'][i]
            robot_param = call_get_parameters(self,'param_server','rmep',[f'{robot_name}.pub_cam',f'{robot_name}.ext_pose_topic_type',
                                                                   f'{robot_name}.ext_pose_topic_name'])
            if self.random_assign:
                ip = robot_ip_list[i]
            else:
                ip = call_get_parameters(self,'param_server','rmep',[f'{robot_name}.ip'])[f'{robot_name}.ip']
            self.create_robot(ip, robot_name, i, params, robot_param)
    
    def create_robot(self, ip, robot_name, number, params, robot_param):
        params = params.copy()
        local_port = self.robot_local_port[number]
        if enable_sdk_robot(ip,self.local_ip, local_port):
            node = rclpy.create_node(robot_name, namespace=robot_name)
            self.robots[robot_name] = node
            ep_conf = config.Config(robot_name)
            ep_conf.default_robot_addr = (ip, globals.ROBOT_DEVICE_PORT)
            ep_conf.product = "RobomasterEP"
            ep_conf.cmd_proto = "v1"
            ep_conf.number = number+1
            ep_conf.default_sdk_addr = (self.local_ip, local_port)
            ep_conf.video_stream_port = globals.ROBOT_VIDEO_PORT
            ep_conf.video_stream_proto = globals.ROBOT_VIDEO_PROTO
            if params['ext_pose'] and robot_param is not None:
                params['topic_name'] = robot_param[f'{robot_name}.ext_pose_topic_name']
                params['topic_type'] = robot_param[f'{robot_name}.ext_pose_topic_type']
            if robot_param is not None:
                if robot_param[f'{robot_name}.pub_cam']:
                    params['cam'] = True
            sdk_conn = Connection(conf=ep_conf)
            cli = Client(conn=sdk_conn)
            robot = Robot(node,conf=ep_conf, cli=cli, params=params)
            ret = robot._initialize()
            if ret:
                self.get_logger().info(f"Robot initialized: {robot_name} : {robot._get_sn()}")
                time.sleep(0.3)
                if params['cam'] and not robot.start_video(
                        resolution=params['camera_resolution'],
                        publish_fps=params['camera_publish_fps']):
                    self.get_logger().error(
                        f"Failed to start camera stream for {robot_name}"
                    )
                self.all_robots[robot_name] = robot
                self.executor.add_node(node)
                time.sleep(1)
                return True
            else:
                self.get_logger().error(f"Failed to initialize robot: {robot_name}")
            time.sleep(0.3)
            #if ret:
        pass
    
    def add_robot(self, request, response):
        self.get_logger().info(f"This functionality is not implemented yet, please use the config file to add robots")
        _name = request.name
        _ip = request.ip
        _type = request.type
    
    def rmv_robot(self, request, response):
        _name = request.name
        _type = request.type
        if _type == 'rmtt':
            self.all_drones[_name].close()
            self.num_of_drones -= 1
            #self.base_params_drone['drone_name_list'].remove(_name)
            self.drone_local_port.remove(self.all_drones[_name]._conf.default_sdk_addr[-1])
            self.drone_video_port.remove(self.all_drones[_name]._conf._video_stream_addr[-1])
            self.executor.remove_node(self.drones[_name])
            self.drones[_name].destroy_node()
            self.drones.pop(_name)
            self.all_drones.pop(_name)
            self._update_drone_fleet_services()
            response.success = True
            response.message = f'Drone {_name} removed'
            return response
    
    def add_port(self,port_list,min,max):
        port_list = set(port_list)
        not_assigned = set(sample(range(min,max), max-min)) - port_list
        port_list = list(port_list)
        port_list.append(not_assigned.pop())
        return port_list

    def _update_drone_fleet_services(self):
        if self.all_drones and self.takeoff_srv is None:
            self.takeoff_srv = self.create_service(
                Trigger, 'takeoff_all', self.takeoff_all
            )
            self.land_srv = self.create_service(
                Trigger, 'land_all', self.land_all
            )
        elif not self.all_drones and self.takeoff_srv is not None:
            self.destroy_service(self.takeoff_srv)
            self.destroy_service(self.land_srv)
            self.takeoff_srv = None
            self.land_srv = None


    def send_takeoff_request(self, req:Takeoff.Request, drone_name):
        self.get_logger().info(f"Sending takeoff request to {drone_name}")
        self.takeoff_future[drone_name] = self.takeoff_cli[drone_name].call_async(req)
        self.takeoff_future[drone_name].add_done_callback(self.done_callback)

    def send_land_request(self, req:Trigger.Request, drone_name):
        self.get_logger().info(f"Sending land request to {drone_name}")
        self.land_future[drone_name] = self.land_cli[drone_name].call_async(req)
        self.land_future[drone_name].add_done_callback(self.done_callback)

    def done_callback(self, future:Future):
        response = future.result()
        if response.success:
            drone_name = None
            for name, fut in self.takeoff_future.items():
                if future == fut:
                    drone_name = name
                    break
            if drone_name in self.remaining:
                self.remaining.remove(drone_name)
            
    def takeoff_all(self, request, response: Trigger.Response):
        threads = []
        for drone in self.all_drones.values():
            t = threading.Thread(target=drone._takeoff, kwargs={'async_flag': True, 'retry': 5})
            t.start()
            threads.append(t)
        for t in threads:
            t.join()
        response.success = bool(threads)
        response.message = (
            f'Takeoff requested for {len(threads)} drone(s).'
            if threads else 'No drones are connected.'
        )
        return response
    
    def land_all(self, request, response:Trigger.Response):
        threads = []
        for drone in self.all_drones.values():
            t = threading.Thread(target=drone._land, kwargs={'async_flag': True, 'retry': 5})
            t.start()
            threads.append(t)
        for t in threads:
            t.join()
        response.success = bool(threads)
        response.message = (
            f'Landing requested for {len(threads)} drone(s).'
            if threads else 'No drones are connected.'
        )
        return response
    
    def close_all(self):
        for drone in self.all_drones.values():
            drone.close()
        for robot in self.all_robots.values():
            robot._close_robot()

    @staticmethod         
    def hotkey_callback(func):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            if not self.keyboard_paused:
                return func(self, *args, **kwargs)
            else:
                self.get_logger().info("Hotkey callback ignored some commands are already executing")
        return wrapper
    
    @hotkey_callback
    def on_activate_t(self):
        if self.armed:
            self.pause_keyboard()
            threads = []
            for drone in self.all_drones.values():
                t = threading.Thread(target=drone._takeoff, kwargs={'async_flag': True, 'retry': 3})
                t.start()
                threads.append(t)
            for t in threads:
                t.join()
            self.resume_keyboard()
        else:
            self.get_logger().info("Drone is not armed, please press 'alt+a' to arm")
    @hotkey_callback
    def on_activate_l(self):
        self.pause_keyboard()
        threads = []
        for drone in self.all_drones.values():
            drone.stop(retry=1)
            t = threading.Thread(target=drone._land, kwargs={'async_flag': True, 'retry': 3})
            t.start()
            threads.append(t)
        for t in threads:
            t.join()
        self.armed = False
        self.resume_keyboard()
    @hotkey_callback
    def on_activate_e(self):
        self.pause_keyboard()
        self.armed = False
        threads = []
        for drone in self.all_drones.values():
            drone.stop(retry=1)
            t = threading.Thread(target=drone._emergency, kwargs={'async_flag': True, 'retry': 5})
            t.start()
            threads.append(t)
        for t in threads:
            t.join()
        self.resume_keyboard()
    @hotkey_callback
    def on_activate_a(self):
        self.get_logger().info("Drones armed, Accepting commands")
        for drone in self.all_drones.values():
            drone.set_motion_armed(True)
        self.armed = True

    def pause_keyboard(self):
        self.keyboard_paused = True
        self.get_logger().info("Keyboard hotkeys paused")
    def resume_keyboard(self):
        self.keyboard_paused = False
        self.get_logger().info("Keyboard hotkeys resumed")

    def run_hotkeys(self):
        self.hotkeys.start()
        self.hotkeys.join()

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = RobomasterServer(executor)
    node.executor.add_node(node)
    print(node.executor.get_nodes())
    print(node.drones)
    try:
        node.executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.close_all()
        except Exception as exc:
            node.get_logger().error(f'Failed to close all robots: {exc}')
        finally:
            node.executor.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
