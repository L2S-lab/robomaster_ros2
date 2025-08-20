# pip3 install myqr

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from robomaster_ros2.modules import network_manager
from ament_index_python.packages import get_package_share_directory

try:
    from robomaster_ros2.modules import algo
except ImportError as e:
    raise(f"Import Error: {e}")


import time
import os
from MyQR import myqr
from PIL import Image
import socket
import getpass
import random
import base64

class Setup_wifi(Node):

    def __init__(self):
        super().__init__('setup_wifi',
                         allow_undeclared_parameters=True)
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('SSID', Parameter.Type.STRING),
                ('PSK', Parameter.Type.STRING),
                ('UPDATE', Parameter.Type.INTEGER_ARRAY),
                ('drone_list', Parameter.Type.STRING_ARRAY),
                ('EP', Parameter.Type.BOOL),
            ])
        self.SSID = None
        self.PSK = None
        self.connection()

    def connection(self):

        self.get_logger().info("Turn on all the Tello drones if there is any to update")

        #Turn on the wifi of the pc
        network_manager.wifi_on()
        time.sleep(5)
        self.get_logger().info("Wifi on")

        catch = self.getIDPW()

        if catch != 0: # catch = -1 if something went wrong in getIDPW
            self.get_logger().error("Error in getting SSID and Password and conencting to the router, destroying the node")
            self.destroy_node()
            
        else:
            #UPDATE is the list of drones we want to use
            UPDATE = self.get_parameter('UPDATE').value
            if UPDATE == None or UPDATE == [] or UPDATE == [0]:
                self.get_logger().info("No drones to update")
            else:
                self.declare_parameter('drone_list.pwd', Parameter.Type.STRING)
                drone_pwd = self.get_parameter('drone_list.pwd').value
                if drone_pwd == 'None' or drone_pwd == None :
                    drone_pwd = None
                for i in UPDATE:
                    self.declare_parameter('drone_list.n'+str(i)+'.ssid', Parameter.Type.STRING)
                    drone_ssid = self.get_parameter('drone_list.n'+str(i)+'.ssid').value
                    if drone_pwd=='':
                        self.declare_parameter('drone_list.n'+str(i)+'.pwd', Parameter.Type.STRING)
                        drone_pwd = self.get_parameter('drone_list.n'+str(i)+'.pwd').value

                    self.get_logger().info(f"drone_ssid: {drone_ssid}, drone_pwd: {drone_pwd}")
                    #WRITE ID and PWD of the router on the drone
                    self.get_logger().info(f"Connecting to drone {drone_ssid} ...")
                    self.writeIDPW_drone(drone_ssid, drone_pwd)
                    time.sleep(1)

                msg = '''\nTurn OFF all the Tello drones\nSwitch the drones to router mode\nTurn ON all the Tello drones\n
                Wait for the propellers to spin. Around 60 seconds\nYou can now flip the drone to 90° to stop the propellers and place drones on the ground\n
                The drones are now connected to the router and ready to fly\n
                '''        
                self.get_logger().info(msg)

        # EP and S1
        EP = self.get_parameter('EP').value
        if EP is True:
            try:
                #self.get_logger().info(f"{self.SSID=} {self.PSK=}")
                appID = str(random.randint(10000, 20000))
                CC = "CN"

                ssid_len = len(self.SSID)
                pwd_len = len(self.PSK)
                buf = bytearray(2 + 8 + 2 + ssid_len + pwd_len)
                buf[0] = ssid_len | (pwd_len & 0x3) << 6
                buf[1] = (pwd_len >> 2) | (0 << 3)
                buf[2:10] = appID.encode(encoding="utf-8")
                buf[10:12] = CC.encode(encoding="utf-8")
                buf[12:12 + ssid_len] = self.SSID.encode(encoding="utf-8")
                buf[12 + ssid_len:12 + ssid_len + pwd_len] = self.PSK.encode(encoding="utf-8")
                buf = algo.simple_encrypt(buf)
                info = bytes.decode(base64.b64encode(buf), encoding='utf-8')

                QRCODE_NAME = self.SSID+'.png'
                DIR = os.path.join(get_package_share_directory('robomaster_ros2'),'..','..','..','..','wifi_qr')
                if not os.path.exists(DIR):
                    os.mkdir(DIR)

                myqr.run(words=info, save_name=QRCODE_NAME, save_dir=DIR)
                self.get_logger().info("QR code created")
                self.get_logger().info("Opening the QR code...")
                self.get_logger().info("Scan the qrcode with the robot camera to connect it to the router")
                time.sleep(1)
                img = Image.open(DIR+'/'+QRCODE_NAME)
                img.show()

            except Exception as e:
                self.get_logger().error(f"Error in creating QR code: {e}")
        self.get_logger().info("PROCESS FINISHED...")


    def getIDPW(self):
        try:
            self.SSID = self.get_parameter('SSID').value #ID of the router
            if self.SSID == None:
                self.get_logger().info("SSID name is necessary. enter it below and try again")
                time.sleep(0.2)
                while True:
                    self.SSID = input("Enter wifi SSID: ")
                    if self.SSID != '':
                        break
            self.get_logger().info(f"SSID: {self.SSID}")
            try:
                self.PSK = self.get_parameter('PSK').value #Password of the router
                if self.PSK == 'None':
                    self.PSK=''
                    pass

                elif self.PSK == '-1':
                    try:
                        self.PSK = network_manager.get_psk(self.SSID)
                    except Exception as e:
                        self.PSK == None
                        self.get_logger().error(f"Unable to get stored password.")

                if self.PSK == None:
                    time.sleep(0.2)
                    while True:
                        self.PSK = getpass.getpass("Enter wifi password: ")
                        if self.PSK != '':
                            break
                    
            except Exception as e:
                self.PSK = None
                self.get_logger().error(f"Error : {e}")
                self.get_logger().error("Password retriving failed")
                return -1
        except Exception as e:
            self.SSID = None
            self.get_logger().error(f"Error : {e}")
            self.get_logger().error("SSID retriving failed")
            return -1
        try:
            #Connect to the router
            self.get_logger().info("In case if it ask for the password to connect with the main router please entrer it")
            network_manager.connect(self.SSID, self.PSK)
        except Exception as e:
            self.get_logger().error(f"Error connecting to the router with provided SSID and password: {e}")
            self.get_logger().error("But you can still continue with the process and connect manuaaly later")
            return 0
        return 0
    
    def writeIDPW_drone(self, drone_ssid, drone_pwd):
        #connect to the drone
        network_manager.connect(drone_ssid, drone_pwd)
        try:
            self.setup_drone(drone_ssid)
            return 0
        except Exception as e:
            return -1

    def setup_drone(self, drone_ssid):
        #host_addr = ('192.168.10.2', 8890)
        target_addr = ('192.168.10.1', 8889)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        cmd = b'command'
        sock.sendto(cmd, target_addr)
        time.sleep(0.1)
        data,_ = sock.recvfrom(1024)
        if data == b'ok':
            cmd = f'ap {self.SSID} {self.PSK}'.encode('utf-8')
            sock.sendto(cmd, target_addr)
            time.sleep(0.1)
            data,_ = sock.recvfrom(1024)
            if 'OK' in data.decode('utf-8'):
                self.get_logger().info(f"Drone {drone_ssid} connected to the router")
            else:
                self.get_logger().error(f"Drone {drone_ssid} data send failed")
        else:
            self.get_logger().error(f"Drone {drone_ssid} data send failed")
        sock.close()


def main(args=None):
    rclpy.init(args=args)

    setup_wifi = Setup_wifi()

    rclpy.spin(setup_wifi)

    setup_wifi.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
