#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_share_directory

from robomaster_ros2.modules.drone import Connection as drone_connection
from robomaster_ros2.modules.network_manager import interface_selector
from robomaster_ros2.modules.globals import *

import yaml
import time
import netifaces
import socket

class robot_info(Node):

    def __init__(self):
        super().__init__('retrive_robot_info', allow_undeclared_parameters=True)
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('RMTT', Parameter.Type.INTEGER),
                ('EP', Parameter.Type.INTEGER),
            ])
        interface = interface_selector()
        self.local_ip =  interface[1]
        self.nb_rmtt = self.get_parameter('RMTT').value
        self.nb_ep = self.get_parameter('EP').value
        self._socket = None
        self.num_of_robots = 0
        self.SN_IP_drone = []
        self.SN_IP_robot = []
        if not self.local_ip or \
            self.local_ip not in [ip['addr'] for interface in netifaces.interfaces()\
            for ip in netifaces.ifaddresses(interface).get(netifaces.AF_INET, [])]:
            self.get_logger().error(f"[{self.local_ip}] is not a valid IP address")
            self.destroy_node()
            return
        self.get_drone_IP_SN()
        self.get_robot_IP_SN()
        self.generate_params()

    def get_drone_IP_SN(self):
        ip_list = self._scan_multi_drone(self.nb_rmtt)
        list_failed = []
        self._socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((self.local_ip, TELLO_SDK_PORT))
        self._socket.settimeout(1)
        for i, ip in enumerate(ip_list):
            cmd = 'command'
            retry = 3
            enabled = False
            while retry>0:
                if not enabled:
                    self._socket.sendto(cmd.encode('utf-8'), (ip, TELLO_DEVICE_PORT))
                    time.sleep(0.1)
                    try:
                        resp, _ = self._socket.recvfrom(1024)
                    except socket.timeout:
                        if retry==0:
                            list_failed.append(ip)
                            self.get_logger().error(f"[Connection] [{ip}] enable_sdk failed.")
                        retry -= 1
                        self.get_logger().error(f"[Connection] [{ip}] enable_sdk failed retrying...")
                        continue
                    self.get_logger().debug(f"[Connection] [{ip}] enable_sdk, recv msg: {resp}")
                    if resp == b'ok':
                        enabled = True
                if enabled:
                    cmd = 'sn?'
                    self._socket.sendto(cmd.encode('utf-8'), (ip, TELLO_DEVICE_PORT))
                    time.sleep(0.1)
                    try:
                        resp, _ = self._socket.recvfrom(1024)
                    except socket.timeout:
                        if retry==0:
                            list_failed.append(ip)
                            self.get_logger().error(f"[Connection] [{ip}] get_sn failed.")
                        retry -= 1
                        self.get_logger().error(f"[Connection] [{ip}] get_sn failed retrying...")
                        continue
                    self.get_logger().debug(f"[Connection] [{ip}] get_sn, recv msg: {resp}")
                    if len(resp.decode('utf-8')) != ROBOT_SN_LEN:
                        retry -= 1
                        self.get_logger().error(f"[Connection] [{ip}] get_sn failed retrying...")
                        if retry==0:
                            list_failed.append(ip)
                            self.get_logger().error(f"[Connection] [{ip}] get_sn failed.")
                        continue
                    else:
                        SN = resp.decode('utf-8')
                        self.SN_IP_drone.append((SN, ip))
                        self.get_logger().info(f"[Connection] [{ip}] enable_sdk success")
                        break
        self._socket.close()
        self.get_logger().info(f"Found {len(self.SN_IP_drone)} drones")
        
    def _scan_multi_drone(self, drone_number=1,):
        conn = drone_connection(sock=self._socket)
        ip_list = [] 
        host_list = conn.scan_multi_drones(drone_number, local_ip=self.local_ip)
        ip_list = [host[0] for host in host_list]
        conn.close()
        return ip_list
    
    def get_robot_IP_SN(self, timeout=5):
        self._socket = None
        ip_list = []
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(('0.0.0.0', ROBOT_BROADCAST_PORT))
        self._socket.settimeout(1)
        start = time.time()
        while time.time()-start > timeout:
            try:
                data, ip = self._socket.recvfrom(1024)
            except socket.timeout:
                self.get_logger().error(f"scan_robot_ip_list: socket recv timeout")
                continue
            if ip[0] not in ip_list and len(ip[0].split('.')) == 4:
                ip_list.append(ip[0])
                sn = data.decode(encoding='utf-8').split(b'\x00')[0]
                self.SN_IP_robot.append((sn, ip[0]))
                if len(ip_list) == self.nb_ep:
                    break
        self._socket.close()
        self.get_logger().info(f"Found {len(ip_list)} robots")

    def generate_params(self):
        # Fill the configuration file with the SN and IP of each drone and robot
        config_filename = 'sn_to_name.yaml'
        self.num_of_robots = self.nb_rmtt + self.nb_ep
        # Declare the beginning of the dynamic parameters
        params = {
            #'param_server':{
            #    'ros__parameters':{
                    'nb_robots': self.num_of_robots,
                    'robot_list':{}
                    }
            #}}

        # For each robot, store the SN, IP, name and type (TT or EP or S1)
        for i in range(self.num_of_robots):
            object_key = f'ROBOT{i+1}'
            if i < len(self.SN_IP_drone):
                _name = f'rmtt_{i+1}' 
                _type = 'TT'
                _SN = self.SN_IP_drone[i][0]
                _IP = self.SN_IP_drone[i][1]
            else:
                _name = f'rmep_{i+1-len(self.SN_IP_drone)}'
                _type = 'EP_S1'
                _SN = self.SN_IP_robot[i-len(self.SN_IP_drone)][0]
                _IP = self.SN_IP_robot[i-len(self.SN_IP_drone)][1]

            #params['param_server']['ros__parameters']['robot_list'][object_key] = {
            params['robot_list'][object_key] = {
                'name': _name,
                'SN': _SN,
                'type': _type,
                'IP': _IP
            }
        
            # Write the parameters to the configuration file
        self.write_config_file(params, config_filename)
        self.get_logger().info("Configuration file 'sn_to_name.yaml' written")

    # Function to write parameters to a YAML file
    def write_config_file(self, params, filename):
        file_path = get_package_share_directory('robomaster_ros2') + '/config_ros/' + filename
        with open(file_path, 'w') as file:
            yaml.dump(params, file)

    # Function to read parameters from a YAML file
    def read_config_file(self, filename):
        file_path = get_package_share_directory('robomaster_ros2') + '/config_ros/' + filename
        with open(file_path, 'r') as file:
            params = yaml.safe_load(file)
        return params



def main(args=None):
    rclpy.init(args=args)

    robot_info = robot_info()

    rclpy.spin(robot_info)

    robot_info.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
