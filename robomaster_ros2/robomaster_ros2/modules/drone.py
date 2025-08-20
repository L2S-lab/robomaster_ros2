import socket, threading, collections
import time
from math import radians

from numpy import array, pi

from rclpy.node import Node
#from std_msgs.msg import String
from std_srvs.srv import Trigger, Empty
from robomaster_interface.srv import Takeoff, GoTo, Move, SetSpeed, TelloLED, TelloMled 
from robomaster_interface.msg import TelloMpad
from geometry_msgs.msg import Twist, TwistStamped
from geometry_msgs.msg import PoseStamped, Pose, Point, PointStamped
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Image
from tf_transformations import quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


from rclpy.logging import get_logger
logger = get_logger('drone')

try:
    from . import protocol, config, dds
    from . import media
    from . custom_char import get_custom_char, valid_custom_char
    from .common import *
    from .globals import *
except ImportError as e:
    import protocol, config, dds
    import media
    from custom_char import get_custom_char, valid_custom_char
    from common import *
    from globals import *
except Exception as e:
    logger.error(f"Import Error pass two: {e}")

class TelloStatusSubject(dds.Subject):
    name = dds.DDS_TELLO_ALL

    def __init__(self):
        self._freq = protocol.TelloDdsProto.DDS_FREQ
        self._pad_mid = 0
        self._pad_x = 0
        self._pad_y = 0
        self._pad_z = 0
        self._pad_mpry = []
        self._pad_mpry_num = 3
        self._pitch = 0
        self._roll = 0
        self._yaw = 0
        self._vgx = 0
        self._vgy = 0
        self._vgz = 0
        self._templ = 0
        self._temph = 0
        self._tof = 0
        self._high = 0
        self._bat = 0
        self._baro = 0
        self._motor_time = 0
        self._agx = 0
        self._agy = 0
        self._agz = 0
        self._dds_proto = protocol.TelloDdsProto
        self._status_dict = {self._dds_proto.DDS_PAD_MID_FLAG: self._pad_mid,
                             self._dds_proto.DDS_PAD_X_FLAG: self._pad_x,
                             self._dds_proto.DDS_PAD_Y_FLAG: self._pad_y,
                             self._dds_proto.DDS_PAD_Z_FLAG: self._pad_z,
                             self._dds_proto.DDS_PAD_MPRY_FLAG: self._pad_mpry,
                             self._dds_proto.DDS_PITCH_FLAG: self._pitch,
                             self._dds_proto.DDS_ROLL_FLAG: self._roll,
                             self._dds_proto.DDS_YAW_FLAG: self._yaw,
                             self._dds_proto.DDS_VGX_FLAG: self._vgx,
                             self._dds_proto.DDS_VGY_FLAG: self._vgy,
                             self._dds_proto.DDS_VGZ_FLAG: self._vgz,
                             self._dds_proto.DDS_TEMP_L_FLAG: self._templ,
                             self._dds_proto.DDS_TEMP_H_FLAG: self._temph,
                             self._dds_proto.DDS_TOF_FLAG: self._tof,
                             self._dds_proto.DDS_HIGH_FLAG: self._high,
                             self._dds_proto.DDS_BATTERY_FLAG: self._bat,
                             self._dds_proto.DDS_BARO_FLAG: self._baro,
                             self._dds_proto.DDS_MOTOR_TIME_FLAG: self._motor_time,
                             self._dds_proto.DDS_AGX_FLAG: self._agx,
                             self._dds_proto.DDS_AGY_FLAG: self._agy,
                             self._dds_proto.DDS_AGZ_FLAG: self._agz}

    def decode(self, buf):
        if dds.IS_AI_FLAG not in buf:
            push_data_list = buf.split(';')
            for info in push_data_list:
                if ':' not in info:
                    continue
                name, data = info.split(':')
                if name == self._dds_proto.DDS_PAD_MPRY_FLAG:
                    pad_mpry_info = data.split(',')
                    self._status_dict[name].clear()
                    for i in range(self._pad_mpry_num):
                        self._status_dict[name].append(float(pad_mpry_info[i]))
                else:
                    self._status_dict[name] = float(data)

    @property
    def freq(self):
        return self._freq

    @freq.setter
    def freq(self, in_freq):
        if in_freq == 1 or in_freq == 5 or in_freq == 10:
            self._freq = in_freq

    def pad_position(self):
        return self._status_dict[self._dds_proto.DDS_PAD_X_FLAG], \
               self._status_dict[self._dds_proto.DDS_PAD_Y_FLAG], \
               self._status_dict[self._dds_proto.DDS_PAD_Z_FLAG]

    def get_status(self, name):
        return self._status_dict[name]

    def get_all_status(self):
        return self._status_dict

############################################################################################################
class BottomTofInfoSubject(dds.Subject):
    name = dds.DDS_TELLO_TOF

    def __init__(self):
        self._tof = 0
        self._info_num = 1
        self._freq = protocol.TelloDdsProto.DDS_FREQ

    def tof_info(self):
        return self._tof

    def data_info(self):
        return self._tof

    def decode(self, buf):
        push_info = buf.split(';')
        found_info_num = 0
        for info in push_info:
            if protocol.TelloDdsProto.DDS_TOF_FLAG in info:
                tof_str = info.split(':')[1]
                self._tof = int(tof_str)
                found_info_num += 1
        if found_info_num == self._info_num:
            return True
        else:
            logger.warning(f"BottomTofInfoSubject: decode, found_info_num {found_info_num} is not match self._info_num {self._info_num}")
            return False

class TelloAttiInfoSubject(dds.Subject):
    name = dds.DDS_TELLO_ATTITUDE

    def __init__(self):
        self._yaw = 0
        self._pitch = 0
        self._roll = 0
        self._info_num = 3
        self._freq = protocol.TelloDdsProto.DDS_FREQ

    def atti_info(self):
        return self._yaw, self._pitch, self._roll

    def data_info(self):
        return self._yaw, self._pitch, self._roll

    def decode(self, buf):
        push_info = buf.split(';')
        found_info_num = 0
        for info in push_info:
            if protocol.TelloDdsProto.DDS_YAW_FLAG in info:
                yaw_info = info.split(':')[1]
                self._yaw = int(yaw_info)
                found_info_num += 1
            elif protocol.TelloDdsProto.DDS_PITCH_FLAG in info:
                pitch_info = info.split(':')[1]
                self._pitch = int(pitch_info)
                found_info_num += 1
            elif protocol.TelloDdsProto.DDS_ROLL_FLAG in info:
                roll_info = info.split(':')[1]
                self._roll = int(roll_info)
                found_info_num += 1
        if found_info_num == self._info_num:
            return True
        else:
            logger.warning(f"TelloAttiInfoSubject: decode, found_info_num {found_info_num} is not match self._info_num {self._info_num}")
            return False

    @property
    def freq(self):
        return self._freq

    @freq.setter
    def freq(self, in_freq):
        if in_freq == 1 or in_freq == 5 or in_freq == 10:
            self._freq = in_freq

class TelloImuInfoSubject(dds.Subject):
    name = dds.DDS_TELLO_IMU

    def __init__(self):
        self._vgx = 0
        self._vgy = 0
        self._vgz = 0
        self._agx = 0
        self._agy = 0
        self._agz = 0
        self._info_num = 6
        self._freq = protocol.TelloDdsProto.DDS_FREQ

    def Imu_info(self):
        return self._vgx, self._vgy, self._vgz, self._agx, self._agy, self._agz

    def data_info(self):
        return self._vgx, self._vgy, self._vgz, self._agx, self._agy, self._agz

    def decode(self, buf):
        push_info = buf.split(';')
        found_info_num = 0
        for info in push_info:
            if protocol.TelloDdsProto.DDS_VGX_FLAG in info:
                vgx_str = info.split(':')[1]
                self._vgx = float(vgx_str)
                found_info_num += 1
            elif protocol.TelloDdsProto.DDS_VGY_FLAG in info:
                vgy_str = info.split(':')[1]
                self._vgy = float(vgy_str)
                found_info_num += 1
            elif protocol.TelloDdsProto.DDS_VGZ_FLAG in info:
                vgz_str = info.split(':')[1]
                self._vgz = float(vgz_str)
                found_info_num += 1
            elif protocol.TelloDdsProto.DDS_AGX_FLAG in info:
                agx_str = info.split(':')[1]
                self._agx = float(agx_str)
                found_info_num += 1
            elif protocol.TelloDdsProto.DDS_AGY_FLAG in info:
                agy_str = info.split(':')[1]
                self._agy = float(agy_str)
                found_info_num += 1
            elif protocol.TelloDdsProto.DDS_AGZ_FLAG in info:
                agz_str = info.split(':')[1]
                self._agz = float(agz_str)
                found_info_num += 1

        if found_info_num == self._info_num:
            return True
        else:
            logger.warning(f"TelloImuInfoSubject: decode, found_info_num {found_info_num} is not match self._info_num {self._info_num}")
            return False

    @property
    def freq(self):
        return self._freq

    @freq.setter
    def freq(self, in_freq):
        if in_freq == 1 or in_freq == 5 or in_freq == 10:
            self._freq = in_freq

class TextClient(object):

    def __init__(self, conn:Connection=None):
        self._conn = conn
        self._thread = threading.Thread(target=self._recv_task, daemon=True)
        self._running = False
        self._event = threading.Event()
        self._dispatcher = Dispatcher()
        self._has_cmd_wait_ack = False
        self._has_sent = 0
        self._has_recv = 0
        self._wait_ack_mutex = threading.Lock()

    def initialize(self):
        try:
            self._conn.create()
        except Exception as e:
            raise e
            
    def start(self):
        self.initialize()
        self._thread.start()

    def stop(self):
        self._running = False
        self._thread.join()
        logger.warning(f"[{self._conn._conf._name}] running thread stopped")
        self._conn._sock.close()

    def check_is_dds_msg(self, msg):
        return protocol.TextMsg.IS_DDS_FLAG in msg.get_buf()

    def _recv_task(self):
        self._running = True
        while self._running:
            resp = self._conn.recv()
            if not self._running:
                break
            if resp is None:
                logger.warning(f"[Client] [{self._conn._conf._name}] _recv_task, recv resp is None, skip.")
                continue
            self._wait_ack_mutex.acquire()
            if self._has_cmd_wait_ack and not self.check_is_dds_msg(resp):
                logger.debug(f"[Client] [{self._conn._conf._name}] call send_sync dispatcher: {resp}")
                self._dispatch_to_send_sync(resp)
            self._wait_ack_mutex.release()
            if self._dispatcher:
                self._dispatcher.dispatch(resp)
        logger.info(f"[Client] [{self._conn._conf._name}] _recv_task thread: quit.") 

    def send(self, text):
        try:
            self._conn.send(text.encode('utf-8'))
        except Exception as e:
            logger.warning(f"[Client] [{self._conn._conf._name}] send_async_text, exception {e}")
            return False
        return True

    def send_sync_msg(self, msg, callback=None, timeout=2):
        if not self._running:
            logger.error(f"[Client] [{self._conn._conf._name}] send_sync_msg, client rescv_task is not running")
        self._wait_ack_mutex.acquire()
        self._has_cmd_wait_ack = True
        self.send_msg(msg)
        self._wait_ack_mutex.release()
        self._event.wait(timeout)
        if self._event.is_set():
            self._event.clear()
            self._wait_ack_mutex.acquire()
            self._has_cmd_wait_ack = False
            self._wait_ack_mutex.release()
            return self._resp
        else:
            logger.warning(f"[Client] [{self._conn._conf._name}] send_sync_text, failed, timeout.")
            return None

    def send_async_msg(self, msg):
        if not self._running:
            logger.error(f"[Client] [{self._conn._conf._name}] send_async_msg, client recv_task is not running.")
            return None
        return self.send_msg(msg)

    def send_msg(self, msg):
        data = msg.pack()
        self.send(data)
        self._has_sent += 1

    def add_handler(self, obj, name, f):
        self._dispatcher.add_handler(obj, name, f)

    def remove_handler(self, name):
        self._dispatcher.remove_handler(name)

    def _dispatch_to_send_sync(self, msg):
        logger.debug(f"[Client] [{self._conn._conf._name}] _dispatch_to_send_sync, msg {msg}")
        self._resp = msg
        self._event.set()

    def _make_ack_identify(self, msg):
        return msg

class Drone():
    def __init__(self, node:Node=None, conf=config.te_conf, cli:TextClient=None, params:dict=None):
        super().__init__()
        self._conf = conf
        self._client = cli
        self._video_conn = None
        
        self.dds = TelloSubscriber(self)
        self._sock_vdo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.me_cbg = MutuallyExclusiveCallbackGroup()
        self.rnt_cbg = ReentrantCallbackGroup()

        self.state = State.Idle
        self.ext_tof_pub = None     # publisher sensor_msgs/Range
        self.tof_data = None        # data float 
        self.tof_pub = None         # publisher sensor_msgs/Range
        self.attitude_data =None    # data [pitch, roll, yaw]
        self.attitude_pub = None    # publisher geometry_msgs/QuaternionStamped
        self.imu_data = None        # data [vgx, vgy, vgz, agx, agy, agz]
        self.imu_pub = None         # publisher sensor_msgs/Imu
        self.baro_data = None       # data float
        self.baro_pub = None        # publisher sensor_msgs/FluidPressure
        self.mpad_pub = None        # publisher robomaster_msgs/TelloMpad
        self.mpad_data = {}         # {id,x,y,z,p,r,y}
        self.init_baro = None       # Initial barometer reading
        self.init_imu = None        # Initial IMU reading
        self.init_attitude = None   # Initial attitude reading
        self.img_pub = None         # publisher sensor_msgs/Image
        self.node = node            # rclpy.node.Node
        self.exp_pose_sub = None    # rclpy.subscription.Subscription
        self.ext_pose = None        # data geometry_msgs/Pose
        self.pose_pub = None        # TODO
        self.bat = None             # data float
        self.vel_factor = 1.0       # default speed set to 1.0 m/s
        self.yaw_hold = False       # hold initial yaw angle using imu and mission pad data
        
        if self.node is not None:
            # ROS services
            self.get_sn_srv = self.node.create_service(Trigger, 'get_sn', self.get_sn)
            self.get_bat_srv = self.node.create_service(Trigger, 'get_battery', self.get_battery)
            self.set_speed_srv = self.node.create_service(SetSpeed, 'set_speed', self.set_speed)
            self.takeoff_srv = self.node.create_service(Takeoff, 'takeoff', self.takeoff,callback_group=self.me_cbg)
            self.land_srv = self.node.create_service(Trigger, 'land', self.land,callback_group=self.rnt_cbg)
            self.hover_srv = self.node.create_service(Empty, 'hover', self.stop_srv,callback_group=self.rnt_cbg)
            self.safe_emergency_srv = self.node.create_service(Trigger, 'soft_emergency', self.soft_emergency,callback_group=self.rnt_cbg)
            self.emergency_srv = self.node.create_service(Empty, 'emergency', self.emergency,callback_group=self.rnt_cbg)
            self.set_led_srv = self.node.create_service(TelloLED, 'set_led', self.set_led)
            self.set_mled_srv = self.node.create_service(TelloMled, 'set_mled', self.set_mled)
            self.reboot_srv = self.node.create_service(Empty, 'reboot', self.reboot)
            self.status_srv =  self.node.create_service(Trigger, 'status', self.get_status)

            # TODO 
            self.yaw_hold = params["yaw_hold"]
            
            self.cmd_vel_sub = self.node.create_subscription(Twist, 'cmd_vel', self.cmd_vel, 10,callback_group=self.me_cbg)
            #self.cmd_vel_sub = self.node.create_subscription(TwistStamped, 'cmd_vel', self.cmd_vel, 10,callback_group=self.me_cbg)

            if params["ext_pose"]:
                self.ext_pose = Pose()
                self.pose_pub = self.node.create_publisher(Pose, 'rmtt_server/'+'pose', 10,callback_group=self.me_cbg)
                topic_name = params["topic_name"]
                topic_type = params["topic_type"]
                is_valid = False
                if topic_type == 'PoseStamped':
                    self.ext_pose_sub = self.node.create_subscription(PoseStamped, topic_name, 
                                                                      self.ext_pose_callback,10, 
                                                                      callback_group=self.me_cbg)
                    is_valid = True
                elif topic_type == 'PointStamped':
                    self.ext_pose_sub = self.node.create_subscription(PointStamped, topic_name, 
                                                                      self.ext_pose_callback,10,
                                                                      callback_group=self.me_cbg)
                    is_valid = True
                elif topic_type == 'Pose':
                    self.ext_pose_sub = self.node.create_subscription(Pose, topic_name, 
                                                                    self.ext_pose_callback,10,
                                                                    callback_group=self.me_cbg)
                    is_valid = True
                elif topic_type == 'Point':
                    self.ext_pose_sub = self.node.create_subscription(Point, topic_name, 
                                                                    self.ext_pose_callback,10,
                                                                    callback_group=self.me_cbg)
                    is_valid = True
                else:
                    logger.error(f"[Drone] [{self._conf._name}] Invalid topic type {topic_type}")
                if is_valid:
                    # TODO: Implement move and goto services
                    logger.warning(f"[Drone] [{self._conf._name}] move and goto services are not implemented yet.")
                    self.goto_srv = self.node.create_service(GoTo, 'goto', self.goto)
                    self.move_srv = self.node.create_service(Move, 'move', self.move)
            if params["front_tof"]:
                self.ext_tof_pub = self.node.create_publisher(Range, 'front_tof', 10)
                logger.warning(f"[Drone] [{self._conf._name}] front_tof publishing slows down other communication.")
                self.ext_tof_timer = self.node.create_timer(0.5, self.get_ext_tof,callback_group=self.me_cbg)
                self.ext_tof_msg = Range()
                self.ext_tof_msg.min_range = 0.01
                self.ext_tof_msg.max_range = 1.25
                self.ext_tof_msg.field_of_view = 0.0
                self.ext_tof_msg.radiation_type = 1
            if params["bottom_tof"]:
                self.tof_pub = self.node.create_publisher(Range, 'bottom_tof', 10)
                self.tof_msg = Range()
                self.tof_msg.header.frame_id = self._conf._name
                self.tof_msg.min_range = 0.1
                self.tof_msg.max_range = 7.5
                self.tof_msg.field_of_view = 0.0
                self.tof_msg.radiation_type = 1
            if params["attitude"]:
                self.attitude_pub = self.node.create_publisher(QuaternionStamped, 'attitude', 10)
                self.attitude_msg = QuaternionStamped()
                self.attitude_msg.header.frame_id = self._conf._name
            if params["baro"]:
                # TODO: Chech the unit of original data (seem like kPa)
                self.baro_pub = self.node.create_publisher(FluidPressure, 'baro', 10)
                self.baro_msg = FluidPressure()
                self.baro_msg.header.frame_id = self._conf._name
                self.baro_msg.fluid_pressure = 0.0
                self.baro_msg.variance = 0.0
                pass
            if params["imu"]:
                # TODO Check the unit of original data 
                self.imu_pub = self.node.create_publisher(Imu, 'imu', 10)
                self.imu_msg = Imu()
                self.imu_msg.header.frame_id = self._conf._name
            if params["mpad"]:
                self.mpad_pub = self.node.create_publisher(TelloMpad, 'mpad', 10)
                self.mpad_msg = TelloMpad()
                self.mpad_msg.header.frame_id = self._conf._name
            if params["cam"]:
                self._sock_vdo.bind(self._conf._video_stream_addr)
                self._video_conn = media.StreamConnection(sock=self._sock_vdo, robot_ip = self._conf.default_robot_addr[0])
                self.camera = TelloCamera(self)
                self.img_pub = self.node.create_publisher(Image, 'image', 10)
                self.img_msg = Image()
                self.img_timer = None
                self.pub_strategy = "newest"
                self.bridge = CvBridge()
            #self.status_pub = self.node.create_publisher(String, 'status', 10)
            #self.sub_callback_group
            #self.ext_pose_sub
    
    def publisher_callback(self):
        ''' 
        drone_info:  {'mid': -2.0, 'x': -200.0, 'y': -200.0, 'z': -200.0, 'mpry': [0.0, 0.0, 0.0], 'pitch': -59.0, 
                        'roll': 32.0, 'yaw': -27.0, 'vgx': 1.0, 'vgy': -2.0, 'vgz': 0.0, 'templ': 82.0, 'temph': 84.0, 
                        'tof': 30.0, 'h': 0.0, 'bat': 78.0, 'baro': 112.62, 'time': 0.0, 'agx': -745.0, 'agy': -121.0, 
                        'agz': -485.0}
        for mission pad detection
        :id: target object ID
        :x: coordinate of the target image x in cm
        :y: coordinate of the target image y in cm
        :z: coordinate of the target image z in cm
        :mr: target image rotation in degrees
        :mp: target image pitch in degrees
        :my: target image yaw in degrees
        :pitch: pitch angle in degrees
        :roll: roll angle in degrees
        :yaw: yaw angle in degrees
        :vgx: velocity in x direction in dm/s
        :vgy: velocity in y direction in dm/s
        :vgz: velocity in z direction in dm/s
        :templ: low temperature in degree Celsius
        :temph: high temperature in degree Celsius
        :tof: distance from the bottom of the drone to the ground in cm
        :bat: battery percentage
        :baro: barometer measurement is height detectec by barometer in m
        :time: time of motor running in seconds
        agx: acceleration in x direction in cm/s^2
        agy: acceleration in y direction in cm/s^2
        agz: acceleration in z direction in cm/s^2
        '''

        data = self.status_sub.get_all_status()
        try:
            logger.debug(f"[Drone] [{self._conf._name}] publisher_callback, data: {data}")

            self.bat = data['bat']
            self.baro_data = data['baro']
            self.mpad_data = {'id': data['mid'], 'x': data['x'], 'y': data['y'], 'z': data['z'], 'pitch': data['mpry'][0], 'roll': data['mpry'][1], 'yaw': data['mpry'][2]}

            if self.baro_pub:
                self.baro_msg.header.stamp = self.node.get_clock().now().to_msg()
                self.baro_msg.fluid_pressure = self.baro_data*1000
                self.baro_pub.publish(self.baro_msg)

            if self.mpad_pub:
                self.mpad_msg.header.stamp = self.node.get_clock().now().to_msg()
                if self.mpad_data['id'] <0:
                    self.mpad_msg.id = 0
                self.mpad_msg.id = int(self.mpad_data['id'])
                self.mpad_msg.x = self.mpad_data['x']/100
                self.mpad_msg.y = self.mpad_data['y']/100
                self.mpad_msg.z = self.mpad_data['z']/100
                self.mpad_msg.pitch = self.mpad_data['pitch']*pi/180
                self.mpad_msg.roll = self.mpad_data['roll']*pi/180
                self.mpad_msg.yaw = self.mpad_data['yaw']*pi/180
                self.mpad_pub.publish(self.mpad_msg)
            
            if not self.ext_pose:
                # TODO: implement position based on tof and imu data and missionpad detection
                pass
        except Exception as e:
            logger.error(f"[Drone] [{self._conf._name}] publisher_callback, exception: {e}")
        return data

    def start_video(self, strategy="newest"):
        '''
        strategy: "newest" or "pipeline"
        return: bool
        '''
        self.pub_strategy = strategy
        if self._video_conn:
            ret = self.camera.start_video_stream(display=False) 
        if ret:
            self.img_timer = self.node.create_timer(0.05, self.img_publisher_callback, callback_group=self.me_cbg)
            return True
        else:
            return False

    def img_publisher_callback(self):
        frame = self.camera.get_cv2_frame(strategy=self.pub_strategy)
        if frame is not None:
            logger.debug(f'frame: {frame}',once=True)
            try:
                self.img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                if frame is not None:
                    self.img_msg.header.frame_id = self._conf._name
                    self.img_msg.header.stamp = self.node.get_clock().now().to_msg()
                    self.img_pub.publish(self.img_msg)
            except CvBridgeError as e:
                logger.error(f"[Drone] [{self._conf._name}] img_publisher_callback, CvBridgeError: {e}")

    def ext_pose_callback(self, msg):
        if type(msg) == PoseStamped:
            self.ext_pose = msg.pose
        elif type(msg) == PointStamped:
            self.ext_pose.position = msg.point
        elif type(msg) == Pose:
            self.ext_pose = msg
        elif type(msg) == Point:
            self.ext_pose.position = msg
        self.pose_pub.publish(self.ext_pose)

    def _initialize(self, retry=5):
        try:
            self._client.start()
            ret = self._set_port()
            if not ret:
                logger.error(f"[{self._conf._name}] Initialization failed.") 
                self._client.stop()
                return False
            
            self.dds.start()
            self.status_sub = TelloStatusSubject()
            self.status_sub.freq = protocol.TelloDdsProto.DDS_FREQ
            self.dds.add_subject_info(self.status_sub, None, None, None)   
            if self.node:
                if self.tof_pub:
                    self.tof_timer = self.node.create_timer(0.05, self.sub_tof)
                if self.attitude_pub:
                    # This is workin
                    self.attitude_timer = self.node.create_timer(0.05, self.sub_attitude)
                    # Trying something new
                    # self.sub_attitude()
                if self.imu_pub:
                    self.imu_timer = self.node.create_timer(0.05, self.sub_imu)
                self.pub_timer = self.node.create_timer(0.05, self.publisher_callback,callback_group=self.rnt_cbg)
            return True
        except Exception as e:
            logger.error(f"[{self._conf._name}] Connection Create Failed with exception {e}")
            return False

    def close(self):
        """ Stop the drone object """
        self._set_mled(cmd="EXT mled sc")
        self.dds.del_subject_info(self.status_sub)
        self.dds.stop()
        self.stop()
        
        if self.node:
            self.pub_timer.destroy()
            if self.tof_pub:
                self.unsub_tof()
                self.tof_timer.destroy()
            if self.attitude_pub:
                self.unsub_attitude()
                self.attitude_timer.destroy()
            if self.imu_pub:
                self.unsub_imu()
                self.imu_timer.destroy()
            if self.img_pub:
                self.img_timer.destroy()
            # TODO
            #if self.pose_pub:
            #    self.ext_pose_sub.destroy()
        self._sock_vdo.close()
        self._client.stop()

    def _set_port(self, retry=5):
        if self.img_pub:
            cmd = f'port {self._conf.default_sdk_addr[-1]} {self._conf.video_stream_addr[-1]}'
        else:
            cmd = f'port {self._conf.default_sdk_addr[-1]} 11111'
        proto = protocol.TextProtoDrone()
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)
        while retry > 0:
            try:
                resp_msg = self._client.send_sync_msg(msg, timeout=1)
                if resp_msg and resp_msg.get_proto() and resp_msg.get_proto().resp == "ok":
                    logger.info(f"[{self._conf._name}] [set_port], resp == {resp_msg.get_proto().resp}")
                    return True
                else:
                    logger.error(f"[{self._conf._name}] [set_port], failed. retrying...")
                # If we reach here, it means the response was not "ok" or proto was None
                retry -= 1
                time.sleep(0.1)
            # not sure if need to catch exception 
            except Exception as e:
                logger.warning(f"[{self._conf._name}] [set_port], send_sync_msg exception {e}")
                return None
        return False

    def _get_sn(self, retry=5):
        cmd = "sn?"
        proto = protocol.TextProtoDrone()
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)
        while retry > 0:
            try:
                resp_msg = self._client.send_sync_msg(msg, timeout=1)
                if resp_msg and (proto := resp_msg.get_proto()):
                    logger.info(f"[{self._conf._name}] [get_sn], resp == {proto.resp}")
                    return proto.resp
                else:
                    logger.error(f"[{self._conf._name}] [get_sn], failed. retrying...")
                # If we reach here, it means the response was not "ok" or proto was None
                retry -= 1
                time.sleep(0.1)
            # not sure if need to catch exception 
            except Exception as e:
                logger.warning(f"[{self._conf._name}] [get_sn], send_sync_msg exception {e}")
                return None
        return False

    def get_sn(self, request, response:Trigger.Response):
        sn = self._get_sn()
        if sn!=None:
            response.message = sn
            response.success = True
        else:
            response.message = "get_sn failed"
            response.success = False
        return response
    
    def _takeoff(self, async_flag=True, retry=4):
        if self.state == State.Idle:
            self.init_baro = self.baro_data
            self.init_imu = self.imu_data
            self.init_attitude = self.attitude_data
            cmd = "takeoff"
            proto = protocol.TextProtoDrone()
            proto.text_cmd = cmd
            msg = protocol.TextMsg(proto)
            while retry > 0 and self.tof_data <0.3:
                logger.info(f"[{self._conf._name}] takeoff {retry}, tof_data: {self.tof_data}")
                if self.tof_data > 0.3:
                        return True
                if async_flag:  
                    self._client.send_async_msg(msg)
                    time.sleep(1)
                    retry -= 1
                else:
                    try:
                        resp_msg = self._client.send_sync_msg(msg, timeout=1)
                    except:
                        resp_msg = None
                    time.sleep(0.5)
                    if resp_msg and (proto := resp_msg.get_proto()):
                        logger.info(f"[{self._conf._name}] takeoff, ret: {proto.resp}")
                        if proto.resp == 'ok': return True 
                    logger.warning(f"[{self._conf._name}] [takeoff] failed. retrying...")
                    retry -= 2
            if self.tof_data > 0.3:
                self.stop()
                return True
            return False
        else:
            logger.warning(f"[{self._conf._name}] drone is not in idle state. Can not takeoff current state: {self.state}")
        
    def takeoff(self, request:Takeoff.Request, response:Takeoff.Response):
        max_height = 5 # change it here if needed. (in meters)
        response.success = False
        self.takeoff_h = abs(request.height)
        self.takeoff_h = abs(request.height)
        if self.takeoff_h < 0.8 or self.takeoff_h > max_height:
            logger.warning(f"[{self._conf._name}] requested takeoff height is <=0.8m or >5m. doing takeoff at 0.8m")
        ret = self._takeoff(async_flag=request.sync)
        if ret or self.tof_data > 0.5:
            self.state = State.Automatic
        if self.takeoff_h<=0.8 or self.takeoff_h>max_height:
            response.success = True
            time.sleep(0.1)
            self._set_led(r=0,g=255,b=0, async_flag=True)
            return response
        elif self.takeoff_h<max_height:
            while True:
                e = None
                #logger.info(f"{self.tof_data=}")
                if not self.ext_pose:
                    e = self.takeoff_h - self.tof_data
                    if abs(e) < 0.2:
                        self.stop()
                        response.success = True
                        time.sleep(0.1)
                        self._set_led(r=0,g=255,b=0, async_flag=True)
                        break
                elif self.ext_pose:
                    #logger.info(f"{self.ext_pose.position.z=}")
                    e = self.takeoff_h - self.ext_pose.position.z
                    if abs(e) < 0.15:
                        self.stop()
                        response.success = True
                        time.sleep(0.1)
                        self._set_led(r=0,g=255,b=0, async_flag=True)
                        break
                if e:
                    self.rc(z=e*75)
                time.sleep(0.1)
            return response  
        response.success = ret
        return response

    def _land(self,async_flag=True, retry=10):
        cmd = "land"
        proto = protocol.TextProtoDrone()
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)
        while retry > 0 and self.tof_data > 0.2:
            logger.info(f"[{self._conf._name}] land, retry: {retry}, tof_data: {self.tof_data}")
            if async_flag:
                try:  
                    self._client.send_async_msg(msg)
                    time.sleep(0.5)
                except:
                    pass
                retry -= 1
            else:
                try:
                    resp_msg = self._client.send_sync_msg(msg, timeout=1)
                except:
                    resp_msg = None
                if resp_msg and (proto := resp_msg.get_proto()):
                    logger.info(f"[{self._conf._name}] land, ret: {proto.resp}")
                    if proto.resp == 'ok': return True 
                logger.warning(f"[{self._conf._name}] [land] failed. retrying...")
                retry -= 2
                time.sleep(0.1)
        if self.tof_data < 0.2:
            self.state = State.Idle
            return True
        return False
    
    def land(self, request, response:Trigger.Response):
        self.state = State.Landing
        response.message = "land failed."
        response.success = False
        response.success = self._land(retry=3, async_flag=True)
        time.sleep(0.1)
        self._set_led(r=0,g=0,b=0, async_flag=True)
        if response.success or self.tof_data < 0.2: 
            self.state = State.Idle
            response.message = "landed."
        return response
    
    def rc(self, x=0, y=0, z=0, w=0):
        """ Move drone with velocity control
        :param x: float:[-100, 100] roll
        :param y: float:[-100, 100] pitch
        :param z: float:[-100, 100] throttle
        :param w: float:[-100, 100] yaw
        """
        cmd = "rc {0} {1} {2} {3}".format(x, y, z, w)
        proto = protocol.TextProtoDrone()
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)
        try:
            self._client.send_async_msg(msg)
        except Exception as e:
            logger.warning(f"Drone: set rc, send_async_msg exception {e}")

    def cmd_vel(self, data:Twist|TwistStamped):
        data = data.twist if isinstance(data, TwistStamped) else data
        if self.state == State.Idle:
            logger.warning(f"[{self._conf._name}] cmd_vel, drone is in idle state. Takeoff first.", throttle_duration_sec=2)
            return
        self.state = State.Automatic
        if abs(data.linear.x) > 1.0:
            data.linear.x = data.linear.x/abs(data.linear.x)
            logger.warning(f"[{self._conf._name}] cmd_vel, x velocity is out of range [-1,1] resetting.", throttle_duration_sec=2)
        if abs(data.linear.y) > 1.0:
            data.linear.y = data.linear.y/abs(data.linear.y)
            logger.warning(f"[{self._conf._name}] cmd_vel, y velocity is out of range [-1,1] resetting.", throttle_duration_sec=2)
        if abs(data.linear.z) > 1.0:
            data.linear.z = data.linear.z/abs(data.linear.z)
            logger.warning(f"[{self._conf._name}] cmd_vel, z velocity is out of range [-1,1] resetting.", throttle_duration_sec=2)
        if abs(data.angular.z) > 1.0:
            data.angular.z = data.angular.z/abs(data.angular.z)
            logger.warning(f"[{self._conf._name}] cmd_vel, z angular velocity is out of range [-1,1] resetting.", throttle_duration_sec=2)
        # if self.yaw_hold:
        #     e = self.init_attitude[2] - self.attitude_data[2]
        #     data.angular.z = 0.02 * e

        x = -1*int(data.linear.y*100)
        y = int(data.linear.x*100)
        z = int(data.linear.z*100)
        w = int(data.angular.z*100)

        self.rc(x, y, z, w)
    
    def stop(self, retry=5):
        cmd = "stop"
        proto = protocol.TextProtoDrone()
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)
        while retry > 0:
            try:
                resp_msg = self._client.send_sync_msg(msg, timeout=1)
                if resp_msg and (proto := resp_msg.get_proto()):
                    logger.info(f"[{self._conf._name}] stop, ret: {proto.resp}")
                    if proto.resp == 'ok': 
                        self.state = State.Hover 
                        logger.info(f"[{self._conf._name}] Hovering.")
                        return True
                logger.warning(f"[{self._conf._name}] [stop] failed. retrying...")
                retry -= 1
                time.sleep(0.1)
            except Exception as e:
                logger.warning(f"[{self._conf._name}] [stop], send_sync_msg exception {e}")
                return False
        return False

    def stop_srv(self, request, response:Empty.Response):
        self.stop()
        return response

    def motor_off(self):
        self.state = State.Landing
        cmd = "motoroff"
        proto = protocol.TextProtoDrone()
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)

    def _emergency(self, async_flag=True, retry=5):
        cmd = "emergency"
        proto = protocol.TextProtoDrone()
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)
        while retry > 0:
            try:
                if async_flag:
                    self._client.send_async_msg(msg)
                    time.sleep(0.1)
                else:
                    resp_msg = self._client.send_sync_msg(msg, timeout=1)
                    if resp_msg and (proto := resp_msg.get_proto()):
                        logger.info(f"[{self._conf._name}] emergency, ret: {proto.resp}")
                        if proto.resp == 'ok': return True 
                logger.warning(f"[{self._conf._name}] [emergency] failed. retrying...")
                retry -= 1
                time.sleep(0.1)
            except Exception as e:
                logger.warning(f"[{self._conf._name}] [emergency], send_sync_msg exception {e}")
                return False
        return False    

    def emergency(self, request, response:Empty.Response):
        self._emergency(async_flag=True, retry=10)
        self.motor_off()
        self.state = State.Emergency
        return response
    
    def soft_emergency(self, request, response:Trigger.Response):
        response.message = "Trying to land safely."
        response.success = False
        response.success = self._land(async_flag=False)
        if not response.success:
            self._emergency(async_flag=True, retry=10)
        self.state = State.Emergency
        return response

    # TODO: Implement move service
    def _move(self, direction="forward", distance=0):
        pass
    def move(self, request:Move.Request, response:Move.Response):
        pass

    # TODO: Implement goto service
    def _goto(self, target:Point):
        pass
    def goto(self, request:GoTo.Request, response:GoTo.Response):
        pass

    def _set_led(self, r:int=None, g:int=None, b:int=None, breath:float=None, blink:float=None, 
                 r2:int=None, g2:int=None, b2:int=None, cmd:str=None, retry=3, async_flag=False):
        '''
        :param r: int:[0, 255] red color
        :param g: int:[0, 255] green color
        :param b: int:[0, 255] blue color
        :param breath: float:[0.1, 2.5] breath interval
        :param blink: float:[0.1, 10] blink interval
        :param r2: int:[0, 255] red color for second led
        :param g2: int:[0, 255] green color for second led
        :param b2: int:[0, 255] blue color for second led
        :param cmd: str: command string (used for ROS service)
        '''
        proto = protocol.TextProtoDrone()
        if cmd == None:
            if breath==None and blink==None:
                cmd = "EXT led {0} {1} {2}".format(r, g, b)
            elif breath!=None and blink!=None:
                print("Invalid input. Only one of breath or blink can be set.")
                return False
            elif breath!=None and blink==None:
                cmd = "EXT led br {0} {1} {2} {3}".format(breath, r, g, b)
            elif breath==None and blink!=None:
                cmd = "EXT led bl {0} {1} {2} {3} {4} {5}".format(blink, r, g, b, r2, g2, b2)
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)
        if async_flag:
            self._client.send_async_msg(msg)
            time.sleep(0.1)
        else:
            while retry > 0:
                try:
                    resp_msg = self._client.send_sync_msg(msg, timeout=0.2)
                    if resp_msg and resp_msg.get_proto() and resp_msg.get_proto().resp=="led ok":
                        return True
                    retry -= 1
                    time.sleep(0.1)
                except Exception as e:
                    logger.warning(f"[{self._conf._name}] [set_led], send_sync_msg exception {e}")
                    return False
        return False

    def set_led(self, request:TelloLED.Request, response:TelloLED.Response):
        response.success = False
        response.message = "set_led failed."
        if request.breath==0 and request.blink==0:
            if request.led.r>=0 and request.led.r<=255 and request.led.g>=0 and request.led.g<=255 and request.led.b>=0 and request.led.b<=255:
                cmd = "EXT led {0} {1} {2}".format(request.led.r, request.led.g, request.led.b)
                if self._set_led(cmd=cmd):
                    response.success = True
                    response.message = "set_led success."
        elif request.breath>0 and round(request.breath,1)<2.6:
            if request.led.r>=0 and request.led.r<=255 and request.led.g>=0 and request.led.g<=255 and request.led.b>=0 and request.led.b<=255:
                cmd = "EXT led br {0} {1} {2} {3}".format(round(request.breath,1), request.led.r, request.led.g, request.led.b)
                if self._set_led(cmd=cmd):
                    response.success = True
                    response.message = "set_led success."
        elif request.blink>0 and round(request.blink,1)<10.1:
            if (request.led.r>=0 and request.led.r<=255 and request.led.g>=0 and request.led.g<=255 and request.led.b>=0 and request.led.b<=255 and
                request.led2.r>=0 and request.led2.r<=255 and request.led2.g>=0 and request.led2.g<=255 and request.led2.b>=0 and request.led2.b<=255):
                cmd = "EXT led bl {0} {1} {2} {3} {4} {5} {6}".format(round(request.blink,1), request.led.r, request.led.g, request.led.b,
                                                                      request.led2.r, request.led2.g, request.led2.b)
                if self._set_led(cmd=cmd):
                    response.success = True
                    response.message = "set_led success."
        else:
            response.message = "set_led failed. Invalid input."
        return response

    def _get_battery(self, retry=3):
        cmd = "battery?"
        proto = protocol.TextProtoDrone()
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)
        while retry > 0:
            try:
                resp_msg = self._client.send_sync_msg(msg, timeout=0.2)
                if resp_msg and (proto := resp_msg.get_proto()):
                    if proto.resp:
                        return int(proto.resp)
                    else:
                        return None
                else:
                    return None
            except Exception as e:
                return None
        
    def get_battery(self, request, response:Trigger.Response):
        if self.bat!=None:
            response.message = str(self.bat)+"%"
            response.success = True
        else:
            try:
                response.message = str(self._get_battery())+"%"
                response.success = True
            except:
                response.message = "get_battery failed."
                response.success = False
        return response
    
    def _set_speed(self, speed=0):
        cmd = "speed {0}".format(speed)
        proto = protocol.TextProtoDrone()
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)
        try:
            resp_msg = self._client.send_sync_msg(msg)
            if resp_msg:
                proto = resp_msg.get_proto()
                if proto:
                    if proto.resp == "ok":
                        return True
                    else:
                        logger.warning(f"[{self._conf._name}] [set_speed], resp {proto.resp}")
                        return False
                else:
                    return False
            else:
                logger.warning(f"[{self._conf._name}] [set_speed] failed.")
        except Exception as e:
            logger.warning(f"[{self._conf._name}] [set_speed], send_sync_msg exception {e}")
            return False
        
    def set_speed(self, request:SetSpeed.Request, response:SetSpeed.Response):
        response.success = False
        response.message = "set_speed failed."
        if request.speed>=0.1 and request.speed<=1.0:
            if self._set_speed(int(request.speed*100)):
                response.success = True
                response.message = "set_speed success."
        else:
            response.message = "set_speed failed. Invalid input. Speed should be in [0.1, 1.0]"
            response.success = False
        return response

    def _set_mled(self, brightness:int=None, clr:str=None, mled_char:str=None, graph:str=None, scroll_dir=None, freq=None, cmd=None, retry=3):
        '''
        :param brightness: int:[0, 255] brightness of mled
        :param clr: str:[r, b, p] color of mled 
        :param mled_char:[a-z, A-Z, 0-9] str: mled character 
        :param graph: str: graph for mled '64 bit string with 0, r, b, p'
        :param scroll_dir: str:[l, r, u, d] scroll direction 
        :param freq: float:[0.1, 2.5] frequency of mled
        :param cmd: str: command string (used for ROS service)
        '''
        proto = protocol.TextProtoDrone()
        async_flag = False
        if cmd == None:
            # TODO freq and scroll_dir
            if brightness:
                cmd = "EXT mled sl {0}".format(brightness)
            elif mled_char and graph:
                print("Invalid input. Only one of mled_char or graph can be set.")
                return False
            elif graph and len(graph)==64 and graph.isalnum() and set(graph).issubset(set('0rbp')):
                cmd = "EXT mled g {0}".format(graph)
            elif mled_char:
                cmd = "EXT mled s {0} {1}".format(clr, mled_char)
        if cmd.split()[-1] == 'async':
            cmd = cmd[:-6]
            async_flag = True
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)
        if async_flag:
            try:
                self._client.send_async_msg(msg)
                return True
            except Exception as e:
                logger.warning(f"[{self._conf._name}] set ext mled char, send_async_msg exception {e}")
                return False
        else:
            while retry > 0:
                resp_msg = self._client.send_sync_msg(msg,timeout=0.2)
                if resp_msg and resp_msg.get_proto() and resp_msg.get_proto().resp == "matrix ok":
                    return True
                retry -= 1
                time.sleep(0.1)
            return False

    def set_mled(self, request:TelloMled.Request, response:TelloMled.Response):
        freq = None
        clr = None
        scroll_dir = None
        response.message = ""
        response.success = False
        if request.brightness>0 and request.brightness<=255:
            cmd = "EXT mled sl {0}".format(request.brightness)
            self._set_mled(cmd=cmd)
            response.message = response.message + "mled brightness set to {0}".format(request.brightness)
        if request.freq>0 and round(request.freq,1)<2.6:
            freq = round(request.freq,1)
        if request.color in ['r', 'b', 'p']:
            clr = request.color
        if request.scroll_dir in ['l', 'r', 'u', 'd']:
            scroll_dir = request.scroll_dir
        cmd = None
        logger.debug(f"{scroll_dir=}, {freq=}, {clr=}, {request.mled_char=}")
        if request.mled_char== 'custom' or request.mled_char== '':
            graph = request.graph
            if len(graph)==64 and graph.isalnum() and set(graph).issubset(set('0rbp')):
                if scroll_dir and freq:
                    cmd = "EXT mled {0} {1} {2} {3} ".format(scroll_dir, "g", freq, graph)
                else:
                    cmd = "EXT mled g {0}".format(graph)
            else:
                response.message = "Invalid input"
                response.success = False
                return response
        elif len(request.mled_char)==1 and request.mled_char.isalnum():
            if scroll_dir and freq:
                cmd = "EXT mled {0} {1} {2} {3} ".format(scroll_dir, clr, freq, request.mled_char)
            else:
                cmd = "EXT mled s {0} {1}".format(clr, request.mled_char)
        elif len(request.mled_char)>1 and request.mled_char.isalnum():
            if valid_custom_char(request.mled_char):
                graph = get_custom_char(request.mled_char,clr)
                logger.debug(f"{graph=}")
                if scroll_dir and freq:
                    cmd = "EXT mled {0} {1} {2} {3} ".format(scroll_dir, "g", freq, graph)
                else:
                    cmd = "EXT mled g {0}".format(graph)
            else:
                graph =request.mled_char
                if scroll_dir and freq:
                    cmd = "EXT mled {0} {1} {2} {3} ".format(scroll_dir, clr, freq, request.mled_char)
        else:
            response.message = "Invalid input"
            response.success = False
            return response
        
        if self._set_mled(cmd=cmd):
            logger.debug(f"cmd: {cmd}")
            response.success = True
            response.message = response.message + " mled char set to {0}".format(request.mled_char)
        else:
            response.message = "set_mled failed."
            response.success = False            
        return response

    def _reboot(self, retry=5):
        cmd = "reboot"
        proto = protocol.TextProtoDrone()
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)
        while retry > 0:
            try:
                self._client.send_async_msg(msg)
                time.sleep(0.2)
                retry -= 1
            except Exception as e:
                break
        
    def reboot(self, request, response):
        self._reboot()
        return response

    def get_status(self, request, response:Trigger.Response):
        response.message = self.state.name
        response.success = True
        return response

###############################################################
    def _sub_tof(self, freq=10, callback=None, *args, **kw):
        sub = self.dds
        subject = BottomTofInfoSubject()
        subject.freq = freq
        return sub.add_subject_info(subject, callback, args, kw)

    def unsub_tof(self):
        sub_dds = self.dds
        return sub_dds.del_subject_info(dds.DDS_TELLO_TOF)
    
    def get_tof_callback(self, data):
        self.tof_data = data/100
        if self.tof_pub:
            self.tof_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tof_msg.range = data/100
            self.tof_pub.publish(self.tof_msg)
        #logger.info(f"[Drone] [{self._conf._name}] tof data: {self.tof_data}")
    
    def sub_tof(self):
        self._sub_tof(10, self.get_tof_callback)

    def _sub_attitude(self, freq=10, callback=None, *args, **kw):
        sub = self.dds
        subject = TelloAttiInfoSubject()
        subject.freq = freq
        return sub.add_subject_info(subject, callback, args, kw)

    def unsub_attitude(self):
        sub_dds = self.dds
        return sub_dds.del_subject_info(dds.DDS_TELLO_ATTITUDE)
    
    def get_attitude_callback(self, data):
        self.attitude_data = [data[2], data[1], data[0]]
        #logger.info(f"[Drone] [{self._conf._name}] attitude data: {self.attitude_data}")
        if self.attitude_pub:
            quaternion = quaternion_from_euler(radians(self.attitude_data[0]), radians(self.attitude_data[1]), radians(self.attitude_data[2]))
            self.attitude_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.attitude_msg.quaternion.x = quaternion[0]
            self.attitude_msg.quaternion.y = quaternion[1]
            self.attitude_msg.quaternion.z = quaternion[2]
            self.attitude_msg.quaternion.w = quaternion[3]
            self.attitude_pub.publish(self.attitude_msg)
            
    def sub_attitude(self):
        self._sub_attitude(10, self.get_attitude_callback)

    def _sub_imu(self, freq=10, callback=None, *args, **kw):
        sub = self.dds
        subject = TelloImuInfoSubject()
        subject.freq = freq
        return sub.add_subject_info(subject, callback, args, kw)

    def unsub_imu(self):
        sub_dds = self.dds
        return sub_dds.del_subject_info(dds.DDS_TELLO_IMU)
    
    def get_imu_callback(self, data):
        self.imu_data = list(data)
        if self.imu_pub:
            self.imu_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.imu_msg.angular_velocity.x = self.imu_data[0]
            self.imu_msg.angular_velocity.y = self.imu_data[1]
            self.imu_msg.angular_velocity.z = self.imu_data[2]
            self.imu_msg.linear_acceleration.x = self.imu_data[3]
            self.imu_msg.linear_acceleration.y = self.imu_data[4]   
            self.imu_msg.linear_acceleration.z = self.imu_data[5]
            self.imu_pub.publish(self.imu_msg)

    def sub_imu(self):
        self._sub_imu(10, self.get_imu_callback)

###############################################################
    def get_ext_tof(self):
        cmd = "EXT tof?".format()
        proto = protocol.TextProtoDrone()
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)
        try:
            resp_msg = self._client.send_sync_msg(msg,timeout=0.1)
            if resp_msg:
                proto = resp_msg.get_proto()
                if proto:
                    try:
                        self.ext_tof_msg.header.stamp = self.node.get_clock().now().to_msg()
                        self.ext_tof_msg.range = float(proto.resp[4:])/1000
                        self.ext_tof_pub.publish(self.ext_tof_msg)
                        return float(proto.resp[4:])
                    except:
                        return None
            return None
        except Exception as e:
            logger.warning(f"Drone: get ext tof, send_sync_msg exception {e}")
            return None
    
class TelloSubscriber(object):

    def __init__(self, drone:Drone):
        self._drone = drone
        self._publisher = collections.defaultdict(list)
        self._dispatcher_running = False
        self._dispatcher_thread = None
        self._client = self._drone._client
        self._msg = None
        self._freq = protocol.TelloDdsProto.DDS_FREQ

    def __del__(self):
        self.stop()

    def start(self):
        self._client.add_handler(self, "TelloSubscriber", self._msg_recv)
        self._dispatcher_thread = threading.Thread(target=self._dispatch_task)
        self._dispatcher_thread.start()

    def stop(self):
        self._dispatcher_running = False
        if self._dispatcher_thread:
            self._dispatcher_thread.join()
            self._dispatcher_thread = None

    @classmethod
    def _msg_recv(cls, self, msg):
        if protocol.TextMsg.IS_DDS_FLAG in msg.get_proto().resp or IS_AI_FLAG in msg.get_proto().resp:
            self._msg = msg

    def _dispatch_task(self):
        self._dispatcher_running = True
        logger.debug(f"[TelloSubscriber] [{self._drone._conf._name}] dispatcher_task is running...")
        interval = 1 / protocol.TelloDdsProto.DDS_FREQ
        time_count = 0

        while self._dispatcher_running:
            msg = self._msg
            if msg is None:
                if not self._dispatcher_running:
                    break
                continue
            proto = msg.get_proto()
            if proto is None:
                logger.warning(f"[TelloSubscriber] [{self._drone._conf._name}] _publist, msg.get_proto None, msg: {msg}")
                continue
            for name in self._publisher:
                handler = self._publisher[name]
                need_time = protocol.TelloDdsProto.DDS_FREQ / handler.subject.freq
                if time_count % need_time == 0:
                    if handler.subject.decode(proto.resp):
                        handler.subject.exec()
            if time_count > TELLO_DDS_TIME_MAX:
                time_count = 0
            else:
                time_count += 1
            time.sleep(interval)

    def add_subject_info(self, subject, callback=None, *args):
        """ Request Data Subscription Underlying Interface

        :param subject: Data subscription corresponds to subject
        :param callback: The parsing function corresponding to the subscription data
        :return: bool: Data subscription results
        """
        # add handler to publisher.
        subject.set_callback(callback, args[0], args[1])
        handler = dds.SubHandler(self, subject, callback)
        self._publisher[subject.name] = handler
        logger.debug(f"[TelloSubscriber] [{self._drone._conf._name}] add_subject_info, add sub sucessfully")

    def del_subject_info(self, subject_name):
        """ Delete data subscription messages

        :param subject_name: The subscription subject to be deleted
        :return: bool: Delete data subscription results
        """
        logger.debug(f"[TelloSubscriber] [{self._drone._conf._name}] del_subject_info: name:{subject_name}, self._publisher:{self._publisher}")
        if subject_name in self._publisher:
            del self._publisher[subject_name]
            logger.debug(f"[TelloSubscriber] [{self._drone._conf._name}] del_subject_info, del sub sucessfully")
            return True
        else:
            logger.warning(f"[TelloSubscriber] [{self._drone._conf._name}] fail to del_subject_info {subject_name}")
            return False

    @property
    def freq(self):
        return self._freq

    @freq.setter
    def freq(self, in_freq):
        if in_freq <= 0:
            self._freq = 0
        elif in_freq > protocol.TelloDdsProto.DDS_FREQ:
            self._freq = protocol.TelloDdsProto.DDS_FREQ
        else:
            self._freq = in_freq

class TelloCamera(object):
    def __init__(self, drone:Drone):
        super().__init__()
        self._drone = drone
        self._video_enable = False
        if self._drone._video_conn is None:
            logger.error(f"[{self._drone._conf._name}] video connection is None.")
        self._liveview = media.LiveView(self._drone._video_conn, self._drone._conf._name)

    def __del__(self):
        self.stop()

    def start_video_stream(self, display=False):
        """ 
        param display: bool (default False, use rviz to display video)
        return: bool
        """
        self._video_stream(1)
        self._video_enable = True
        return self._liveview.start_video_stream(display)

    def stop_video_stream(self):
        flag = self._liveview.stop_video_stream()
        self._video_stream(0)
        self._video_enable = False
        return flag

    def _video_stream(self, on_off=1):
        cmd = ""
        if on_off == 1:
            cmd = "streamon"
        elif on_off == 0:
            cmd = "streamoff"
        else:
            logger.warning(f"[Video][{self._drone._conf._name}] video_stream error")
            return False
        proto = protocol.TextProtoDrone()
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)
        try:
            resp_msg = self._drone._client.send_sync_msg(msg)
            if resp_msg:
                proto = resp_msg.get_proto()
                if proto:
                    return proto.resp
                else:
                    return False
            else:
                logger.warning(f"[Video][{self._drone._conf._name}] get_wifi failed.")
        except Exception as e:
            logger.warning(f"[Video][{self._drone._conf._name}] get_wifi, send_sync_msg exception {e}")
            return False

    def stop(self):
        if self._video_enable:
            self.stop_video_stream()
        if self._liveview:
            self._liveview.stop()

    def set_fps(self, fps='high', async_flag=False):
        """
        param fps: [high, middle, low]
        return: bool
        """
        cmd = "setfps {0}".format(fps)
        proto = protocol.TextProtoDrone()
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)
        if async_flag:
            try:
                self._drone._client.send_async_msg(msg)
                return True
            except Exception as e:
                logger.warning(f"[Video][{self._drone._conf._name}] set_fps async, send_async_msg exception {e}")
                return False
        else:
            try:
                resp_msg = self._drone._client.send_sync_msg(msg)
                if resp_msg:
                    proto = resp_msg.get_proto()
                    if proto:
                        if proto.resp.lower().startswith(protocol.TextProtoData.SUCCESSFUL_RESP_FLAG):
                            return True
                        else:
                            logger.warning(f"[Video][{self._drone._conf._name}] set_fps resp {proto.resp}")
                    logger.warning(f"[Video][{self._drone._conf._name}] set_fps failed")
                return False
            except Exception as e:
                logger.warning(f"[Video][{self._drone._conf._name}] set_fps, send_sync_msg exception {e}")
                return False

    def set_bitrate(self, bitrate=3, async_flag=False):
        """ 
        :param bitrate: [0, 5]
        :return: bool
        """
        cmd = "setbitrate {0}".format(bitrate)
        proto = protocol.TextProtoDrone()
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)
        if async_flag:
            try:
                self._drone._client.send_async_msg(msg)
                return True
            except Exception as e:
                logger.warning(f"[Video][{self._drone._conf._name}] set_bitrate async, send_async_msg exception {e}")
                return False
        else:
            try:
                resp_msg = self._drone._client.send_sync_msg(msg)
                if resp_msg:
                    proto = resp_msg.get_proto()
                    if proto:
                        if proto.resp.lower().startswith(protocol.TextProtoData.SUCCESSFUL_RESP_FLAG):
                            return True
                        else:
                            logger.warning(f"[Video][{self._drone._conf._name}] resp {proto.resp}")
                    logger.warning(f"[Video][{self._drone._conf._name}] set_bitrate failed")
                return False
            except Exception as e:
                logger.warning(f"[Video][{self._drone._conf._name}] set_bitrate, send_sync_msg exception {e}")
                return False

    def set_resolution(self, resolution='high', async_flag=False):
        """
        TODO issue with low resolution, numpy running out of memory
        :param resolution [high, low]
        :return: bool: 
        """
        cmd = "setresolution {0}".format(resolution)
        proto = protocol.TextProtoDrone()
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)
        if async_flag:
            try:
                self._drone._client.send_async_msg(msg)
                return True
            except Exception as e:
                logger.warning(f"[Video][{self._drone._conf._name}] set_resolution async, send_async_msg exception {e}")
                return False
        else:
            try:
                resp_msg = self._drone._client.send_sync_msg(msg)
                if resp_msg:
                    proto = resp_msg.get_proto()
                    if proto:
                        if proto.resp.lower().startswith(protocol.TextProtoData.SUCCESSFUL_RESP_FLAG):
                            return True
                        else:
                            logger.warning(f"[Video][{self._drone._conf._name}] set_resolution resp {proto.resp}")
                    logger.warning(f"[Video][{self._drone._conf._name}] set_resilution failed")
                return False
            except Exception as e:
                logger.warning(f"[Video][{self._drone._conf._name}] set_resilution, send_sync_msg exception {e}")
                return False

    def set_down_vision(self, setting):
        """
        param direction: [1, 0]
        return: bool
        """
        cmd = "downvision {0}".format(setting)
        print("cmd", cmd)
        proto = protocol.TextProtoDrone()
        proto.text_cmd = cmd
        msg = protocol.TextMsg(proto)
        try:
            resp_msg = self._drone._client.send_sync_msg(msg)
            if resp_msg:
                proto = resp_msg.get_proto()
                if proto:
                    if proto.resp.lower().startswith(protocol.TextProtoData.SUCCESSFUL_RESP_FLAG):
                        return True
                    else:
                        logger.warning(f"[Video][{self._drone._conf._name}] set_down_vision resp {proto.resp}")
                logger.error(f"[Video][{self._drone._conf._name}] set_down_vision failed")
            return False
        except Exception as e:
            logger.error(f"[Video][{self._drone._conf._name}] set_down_vision, send_sync_msg exception {e}")
            return False

    def get_cv2_frame(self, timeout=1, strategy="newest"):
        frame = self._liveview.read_video_frame(timeout, strategy)
        if frame is None:
            return None
        img = array(frame)
        return img
 
