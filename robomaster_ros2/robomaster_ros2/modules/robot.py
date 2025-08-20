import socket, threading
import time
import random, binascii
import struct
import numpy as np

from rclpy.node import Node
from std_srvs.srv import Empty
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from robomaster_interface.srv import MoveChassis 
from robomaster_interface.srv import MoveArm 
from robomaster_interface.srv import MoveGimbal, Fire
from robomaster_interface.srv import Gripper 
from robomaster_interface.srv import RobotLED
from robomaster_interface.msg import GimbalAngle , GimbalVel

from rclpy.logging import get_logger
logger = get_logger('robot')

try:
    from . import protocol, config, dds
    from .action import ActionDispatcher, Action
    from .action import ACTION_RUNNING, ACTION_SUCCEEDED, ACTION_FAILED, ACTION_STARTED
    from . import util
    from .common import *
    from .globals import *
except ImportError as e:
    import protocol, config, dds
    from action import ActionDispatcher, Action
    from action import ACTION_RUNNING, ACTION_SUCCEEDED, ACTION_FAILED, ACTION_STARTED
    import util
    from common import *
    from globals import *
except Exception as e:
    logger.error(f"Import Error pass two: {e}")


class GimbalMoveAction(Action):
    _action_proto_cls = protocol.ProtoGimbalRotate
    _push_proto_cls = protocol.ProtoGimbalActionPush
    _target = protocol.host2byte(4, 0)

    def __init__(self, pitch=0, yaw=0, pitch_speed=30, yaw_speed=30, coord=COORDINATE_YCPN, **kw):
        super().__init__(**kw)
        self._pitch = pitch
        self._yaw = yaw
        self._roll = 0
        self._pitch_speed = pitch_speed
        self._yaw_speed = yaw_speed
        self._coordinate = coord

    def __repr__(self):
        return "action_id:{0}, state:{1}, percent:{2}, pitch:{3}, yaw:{4}, roll:{5}, pitch_speed:{6}, yaw_speed:{7}, " \
               "coord:{8}".format(self._action_id, self._state, self._percent, self._pitch, self._yaw, self._roll,
                                  self._pitch_speed, self._yaw_speed, self._coordinate)

    def encode(self):
        proto = protocol.ProtoGimbalRotate()
        proto._pitch = int(self._pitch)
        proto._yaw = int(self._yaw)
        proto._pitch_speed = int(util.GIMBAL_PITCH_MOVE_SPEED_SET_CHECKER.val2proto(self._pitch_speed))
        proto._yaw_speed = int(util.GIMBAL_YAW_MOVE_SPEED_SET_CHECKER.val2proto(self._yaw_speed))
        proto._coordinate = self._coordinate
        return proto

    def update_from_push(self, proto):
        """ Push message to update Action status """
        if proto.__class__ is not self._push_proto_cls:
            return

        self._percent = proto._percent
        if proto._action_state == 0:
            self._changeto_state(ACTION_RUNNING)
        elif proto._action_state == 1:
            self._changeto_state(ACTION_SUCCEEDED)
        elif proto._action_state == 2:
            self._changeto_state(ACTION_FAILED)
        elif proto._action_state == 3:
            self._changeto_state(ACTION_STARTED)
        else:
            logger.warning("GimbalMoveAction: update_from_push, unsupported state {0}".format(proto._action_state))
            return

        self._yaw = float(proto._yaw) / 10.0
        self._roll = float(proto._roll) / 10.0
        self._pitch = float(proto._pitch) / 10.0
        logger.debug("{0}: update_from_push, {1}".format(self.__class__.__name__, self))

class ChassisMoveAction(Action):
    _action_proto_cls = protocol.ProtoPositionMove
    _push_proto_cls = protocol.ProtoPositionPush
    _target = protocol.host2byte(3, 6)

    def __init__(self, x=0., y=0., z=0., spd_xy=0., spd_z=0., **kw):
        super().__init__(**kw)
        self._x = x
        self._y = -y
        self._z = z
        self._spd_xy = spd_xy
        self._spd_z = spd_z

    def __repr__(self):
        return "action_id:{0}, state:{1}, percent:{2}, x:{3}, y:{4}, z:{5}, xy_speed:{6}, z_speed:{7}".format(
            self._action_id, self._state, self._percent, self._x, self._y, self._z, self._spd_xy, self._spd_z)

    def encode(self):
        proto = protocol.ProtoPositionMove()
        proto._pos_x = int(util.CHASSIS_POS_X_SET_CHECKER.val2proto(self._x))
        proto._pos_y = int(util.CHASSIS_POS_Y_SET_CHECKER.val2proto(self._y))
        proto._pos_z = int(util.CHASSIS_POS_Z_SET_CHECKER.val2proto(self._z))
        # The spd_xy limit to [0.5, 2.0]
        if self._spd_xy < 0.5:
            self._spd_xy = 0.5
            logger.warning("spd_xy: below limit and is set to 0.5")
        if self._spd_xy > 2.0:
            self._spd_xy = 2.0
            logger.warning("spd_xy: over limit and is set to 2.0")
        proto._vel_xy_max = int(160 * self._spd_xy - 70)
        # The spd_z limit to [10, 540]
        if self._spd_z < 10:
            self._spd_z = 10
            logger.warning("spd_z: below limit and is set to 10")
        if self._spd_z > 540:
            self._spd_z = 540
            logger.warning("spd_z: over limit and is set to 540")
        proto._agl_omg_max = int(self._spd_z * 10)
        return proto

    def update_from_push(self, proto):
        if proto.__class__ is not self._push_proto_cls:
            return

        self._percent = proto._percent
        self._update_action_state(proto._action_state)

        self._pos_x = util.CHASSIS_POS_X_SET_CHECKER.proto2val(proto._pos_x)
        self._pos_y = util.CHASSIS_POS_Y_SET_CHECKER.proto2val(proto._pos_y)
        self._pos_z = util.CHASSIS_POS_Z_SET_CHECKER.proto2val(proto._pos_z)
        logger.debug("{0} update_from_push: {1}".format(self.__class__.__name__, self))

class RoboticArmMoveAction(Action):
    _action_proto_cls = protocol.ProtoRoboticArmMoveCtrl
    _push_proto_cls = protocol.ProtoRoboticArmMovePush
    _target = protocol.host2byte(3, 6)

    def __init__(self, x=0., y=0., z=0., mode=0, **kw):
        super().__init__(**kw)
        self._x = x
        self._y = z # y and z are reversed because the robot has a different coordinate system
        self._z = y # For the robot arm, the y-axis is the vertical direction
        self._mode = mode

    def __repr__(self):
        return "action_id:{0}, state:{1}, percent:{2}, x:{3}, y:{4}, z:{5}".format(
            self._action_id, self._state, self._percent, self._x, self._y, self._z)

    def encode(self):
        proto = protocol.ProtoRoboticArmMoveCtrl()
        proto._x = int(util.ROBOTIC_ARM_POS_CHECK.val2proto(self._x))
        proto._y = int(util.ROBOTIC_ARM_POS_CHECK.val2proto(self._y))
        proto._z = int(util.ROBOTIC_ARM_POS_CHECK.val2proto(self._z))
        proto._mode = self._mode
        proto._mask = 0x03
        return proto

    def update_from_push(self, proto):
        if proto.__class__ is not self._push_proto_cls:
            return

        self._percent = proto._percent
        self._update_action_state(proto._action_state)

        self._x = proto._x
        self._y = proto._y
        self._z = proto._z
        logger.debug("{0} update_from_push: {1}".format(self.__class__.__name__, self))

class PositionSubject(dds.Subject):
    name = dds.DDS_POSITION
    uid = dds.SUB_UID_MAP[name]
    type = dds.DDS_SUB_TYPE_PERIOD

    def __init__(self, cs):
        self._position_x = 0
        self._position_y = 0
        self._position_z = 0
        self._cs = cs
        self._offset_x = 0
        self._offset_y = 0
        self._offset_z = 0
        self._first_flag = True

    def position(self):
        return self._position_x, self._position_y, self._position_z

    def data_info(self):
        ''' cs=0 to use the current position as the origin, \
            otherwise use the position of the robot at the moment of power-up as the origin.'''
        if self._cs == 0:
            if self._first_flag:
                self._offset_x = self._position_x
                self._offset_y = self._position_y
                self._offset_z = self._position_z
                self._first_flag = False
            self._position_x = self._position_x - self._offset_x
            self._position_y = self._position_y - self._offset_y
            self._position_z = self._position_z - self._offset_z
        self._position_x = util.CHASSIS_POS_X_SUB_CHECKER.proto2val(self._position_x)
        self._position_y = -(util.CHASSIS_POS_Y_SUB_CHECKER.proto2val(self._position_y))
        self._position_z = util.CHASSIS_POS_Z_SUB_CHECKER.proto2val(self._position_z)
        return self._position_x, self._position_y, self._position_z

    def decode(self, buf):
        self._position_x, self._position_y, self._position_z = struct.unpack('<fff', buf)

class GimbalPosSubject(dds.Subject):
    name = dds.DDS_GIMBAL_POS
    uid = dds.SUB_UID_MAP[name]
    type = dds.DDS_SUB_TYPE_PERIOD

    def __init__(self):
        self._yaw_angle = 0
        self._pitch_angle = 0
        self._yaw_ground_angle = 0
        self._pitch_ground_angle = 0
        self._option_mode = 0
        self._return_center = 0
        self._res = 0

    @property
    def angle(self):
        return self._pitch_angle, self._yaw_angle, self._pitch_ground_angle, self._yaw_ground_angle

    def data_info(self):
        return self._pitch_angle, self._yaw_angle, self._pitch_ground_angle, self._yaw_ground_angle

    def decode(self, buf):
        [self._yaw_ground_angle, self._pitch_ground_angle, self._yaw_angle,
         self._pitch_angle, self._res] = struct.unpack('<hhhhB', buf)
        self._return_center = (self._res >> 2) & 0x01
        self._option_mode = (self._res & 0x2)
        self._pitch_angle = util.GIMBAL_ATTI_PITCH_CHECKER.proto2val(self._pitch_angle)
        self._yaw_angle = util.GIMBAL_ATTI_YAW_CHECKER.proto2val(self._yaw_angle)
        self._pitch_ground_angle = util.GIMBAL_ATTI_PITCH_CHECKER.proto2val(self._pitch_ground_angle)
        self._yaw_ground_angle = util.GIMBAL_ATTI_YAW_CHECKER.proto2val(self._yaw_ground_angle)

class ArmSubject(dds.Subject):
    name = dds.DDS_ARM
    uid = dds.SUB_UID_MAP[name]
    type = dds.DDS_SUB_TYPE_PERIOD

    def __init__(self):
        self._x_limit = 0
        self._y_limit = 0
        self._main_servo_lock = 0
        self._sub_servo_lock = 0
        self._pos_x = 0
        self._pos_y = 0

    def arm_data(self):
        """ Get information about the robot arm

        :return: tuple: (x, y), coordinates of the robot arm
        """
        return self._pos_x, self._pos_y

    def data_info(self):
        return self._pos_x, self._pos_y

    def decode(self, buf):
        self._pos_x, self._pos_y = struct.unpack('<II', buf[1:])


def scan_robot_ip_list(nb_robots:int,timeout=None):

    ip_list = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("0.0.0.0", config.ROBOT_BROADCAST_PORT))
    except Exception as e:
        logger.warning("scan_robot_ip_list: exception {0}".format(e))
        return ip_list

    start = time.time()
    while True:
        if timeout!= None and time.time()-start > timeout:
            break
        if len(ip_list) >= nb_robots:
            break
        s.settimeout(0.5)
        try:
            data, ip = s.recvfrom(1024)
        except Exception as e:
            logger.warning("scan_robot_ip_list: socket recv, {0}".format(e))
            continue
        logger.debug("conn: scan_robot_ip_list, data:{0}, ip:{1}".format(data[:-1].decode(encoding='utf-8'), ip))
        if ip[0] not in ip_list:
            ip_list.append(ip[0])
            logger.debug("conn: scan_robot_ip_list, ip_list:{0}".format(ip_list))
            logger.debug("find robot sn:{0}, ip:{1}".format(str(data[:-1].decode(encoding='utf-8')), ip[0]))
    s.close()
    return ip_list

def enable_sdk_robot(robot_ip, local_ip, local_port, timeout=1, retry=3):
    _sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    _sock.settimeout(timeout)
    remote_addr = (robot_ip, ROBOT_PROXY_PORT)
    proto = protocol.ProtoSetSdkConnection()
    proto._connection = 1
    proto._host = protocol.host2byte(9, 6)
    proto._port = local_port
    proto._ip = local_ip
    msg = protocol.Msg(ROBOT_DEFAULT_HOST, protocol.host2byte(9, 0), proto)
    buf = msg.pack()
    while retry >0:
        try:
            _sock.sendto(buf, remote_addr)
            time.sleep(0.1)
            data, addr = _sock.recvfrom(1024)
        except:
            retry -= 1
            continue
        resp_msg, data = protocol.decode_msg(data)
        resp_msg.unpack_protocol() 
        if resp_msg:
            prot = resp_msg.get_proto()
            if prot._retcode == 0: 
                if prot._state == 0: 
                    logger.debug("enable_sdk_robot: accept connection.")
                    return True
                if prot._state == 1: 
                    logger.error("enable_sdk_robot: reject connection, service is busy!")
                    return False
                if prot._state == 2: 
                    logger.debug("enable_sdk_robot: got host ip:{0}".format(prot._config_ip)) 
                    return True 
        retry -= 1
    logger.error(f"enable_sdk_robot: {robot_ip} failed.")
    _sock.close()
    return False

class EventIdentify(object):
    def __init__(self):
        self._valid = False
        self._ident = None
        self._event = threading.Event()

class MsgHandler:
    def __init__(self, proto_data=None, req_cb=None, ack_cb=None):
        self._proto_data = proto_data

    @staticmethod
    def make_dict_key(cmd_set, cmd_id):
        return cmd_set * 256 + cmd_id

class Client(object):
    def __init__(self, conn:Connection=None, host=9, index=6):
        self._host = host
        self._index = index
        self._conn = conn

        self._has_sent = 0
        self._has_recv = 0
        self._unpack_failed = 0
        self._dispatcher = Dispatcher()

        self._handler_dict = {}

        self._wait_ack_list = {}
        self._wait_ack_mutex = threading.Lock()
        self._event_list = []

        self._thread = None
        self._running = False

    def __del__(self):
        self.stop()

    def add_handler(self, obj, name, f):
        self._dispatcher.add_handler(obj, name, f)

    def initialize(self):
        if not self._conn:
            logger.warning("Client: initialize, no connections, init connections first.")
            return False
        for i in range(0, CLIENT_MAX_EVENT_NUM):
            ident = EventIdentify()
            self._event_list.append(ident)
        try:
            self._conn.create()
        except Exception as e:
            raise e
        return True

    @property
    def hostbyte(self):
        return protocol.host2byte(self._host, self._index)

    def start(self):
        try:
            result = self.initialize()
            if not result:
                return False
            self._thread = threading.Thread(target=self._recv_task)
            self._thread.start()
        except Exception as e:
            raise e

    def stop(self):
        if self._thread.is_alive(): 
            self._running = False
            proto = protocol.ProtoGetVersion()
            msg = protocol.Msg(self.hostbyte, self.hostbyte, proto)
            self._conn.send_self(msg.pack()) 
            self._thread.join() 
        if self._conn:
            self._conn.close()

    def send_msg(self, msg):
        data = msg.pack()
        logger.debug("Client: send_msg, msg {0} {1}".format(self._has_sent, msg))

        logger.debug("Client: send_msg, cmset:{0:2x}, cmdid:{1:2x}, {2}".format(msg.cmdset, msg.cmdid,
                                                                                binascii.hexlify(data)))

        self._has_sent += 1
        self.send(data)

    def send_sync_msg(self, msg, callback=None, timeout=1.0):
        if not self._running:
            logger.error("Client: send_sync_msg, client recv_task is not running.")
            return None
        if msg._need_ack > 0:
            evt = self._ack_register_identify(msg)
            if evt is None:
                logger.error("Client: send_sync_msg, ack_register failed.")
                return None
            self.send_msg(msg)
            evt._event.wait(timeout)
            if not evt._event.is_set():
                logger.error("Client: send_sync_msg wait msg receiver:{0}, cmdset:0x{1:02x}, cmdid:0x{2:02x} \
timeout!".format(msg.receiver, msg.cmdset, msg.cmdid))
                evt._valid = False
                return None
            resp_msg = self._ack_unregister_identify(evt._ident)
            evt._valid = False
            if resp_msg is None:
                logger.error("Client, send_sync_msg, get resp msg failed.")
            else:
                if isinstance(resp_msg, protocol.Msg):
                    try:
                        resp_msg.unpack_protocol()
                        if callback:
                            callback(resp_msg)
                    except Exception as e:
                        self._unpack_failed += 1
                        logger.warning("Client: send_sync_msg, resp_msg {0:d} cmdset:0x{1:02x}, cmdid:0x{2:02x}, "
                                       "e {3}".format(self._has_sent, resp_msg.cmdset, resp_msg.cmdid, format(e)))
                        return None
                else:
                    logger.warning("Client: send_sync_msg, has_sent:{0} resp_msg:{1}.".format(
                        self._has_sent, resp_msg))
                    return None

            return resp_msg
        else:
            self.send_msg(msg)

    def send(self, data):
        try:
            self._conn.send(data) 
        except Exception as e:
            logger.warning("Client: send, exception {0}".format(str(e)))

    def send_async_msg(self, msg):
        if not self._running:
            logger.error("Client: send_async_msg, client recv_task is not running.")
            return None
        msg._need_ack = 0
        return self.send_msg(msg)

    def _recv_task(self):
        self._running = True
        logger.debug("Client: recv_task, Start to Recving data...")
        while self._running:
            msg = self._conn.recv() 
            if not self._running:
                break
            if msg is None:
                logger.debug("Client: _recv_task, recv msg is None, skip.")
                continue
            self._has_recv += 1
            self._dispatch_to_send_sync(msg)
            self._dispatch_to_callback(msg)
            if self._dispatcher:
                self._dispatcher.dispatch(msg)
        self._running = False

    def _dispatch_to_send_sync(self, msg):
        if msg.is_ack:
            logger.debug("Client: dispatch_to_send_sync, {0} cmdset:{1} cmdid:{2}".format(
                self._has_recv, hex(msg._cmdset), hex(msg._cmdid)))
            ident = self._make_ack_identify(msg)
            self._wait_ack_mutex.acquire()
            if ident in self._wait_ack_list.keys():
                for i, evt in enumerate(self._event_list):
                    if evt._ident == ident and evt._valid:
                        self._wait_ack_list[ident] = msg
                        evt._event.set()
            else:
                logger.debug("Client: dispatch_to_send_sync, ident:{0} is not in wait_ack_list {1}".format(
                    ident, self._wait_ack_list))
            self._wait_ack_mutex.release()

    def _dispatch_to_callback(self, msg):
        if msg._is_ack:
            key = MsgHandler.make_dict_key(msg.cmdset, msg.cmdid)
            if key in self._handler_dict.keys():
                self._handler_dict[key]._ack_cb(self, msg)
            else:
                logger.debug("Client: dispatch_to_callback, msg cmdset:{0:2x}, cmdid:{1:2x} is not define ack \
handler".format(msg.cmdset, msg.cmdid))
        else:
            key = MsgHandler.make_dict_key(msg.cmdset, msg.cmdid)
            if key in self._handler_dict.keys():
                self._handler_dict[key]._req_cb(self, msg)
            else:
                logger.debug("Client: _dispatch_to_callback, cmdset:{0}, cmdid:{1} is not define req handler".format(
                    hex(msg.cmdset), hex(msg.cmdid)))

    @staticmethod
    def _make_ack_identify(msg):
        if msg.is_ack:
            return str(msg._sender) + str(hex(msg.cmdset)) + str(hex(msg.cmdid)) + str(msg._seq_id)
        else:
            return str(msg._receiver) + str(hex(msg.cmdset)) + str(hex(msg.cmdid)) + str(msg._seq_id)

    def _ack_register_identify(self, msg):
        self._wait_ack_mutex.acquire()
        ident = self._make_ack_identify(msg)
        self._wait_ack_list[ident] = 1
        self._wait_ack_mutex.release()
        evt = None

        for i, evt_ident in enumerate(self._event_list):
            if not evt_ident._valid:
                evt = evt_ident
                break
        if evt is None:
            logger.error("Client: event list is run out.")
            return None
        evt._valid = True
        evt._ident = ident
        evt._event.clear()
        return evt

    def _ack_unregister_identify(self, identify):
        try:
            self._wait_ack_mutex.acquire()
            if identify in self._wait_ack_list.keys():
                return self._wait_ack_list.pop(identify)
            else:
                logger.warning("can not find ident:{0} in wait_ack_list.".format(identify))
                return None
        finally:
            self._wait_ack_mutex.release()

class Robot():
    def __init__(self, node:Node=None, conf=config.ep_conf, cli:Client=None, params:dict=None): 
        super().__init__()
        self._conf = conf
        self._action_dispatcher = None
        self._send_heart_beat_timer = None
        self._running = False
        self._initialized = False
        self._audio_id = 0
        self._client = cli
        self._auto_timer = None
        self.dds = dds.Subscriber(self)
        self.dds.start()
        self.distance_pub = None
        self.position_data = None
        self.position_pub = None
        self.gimbal_angle_data = None
        self.gimbal_angle_msg = None
        self.gimbal_angle_pub = None
        self.arm_position_data = None
        self.arm_position_pub = None
        self.arm_position_msg = None
        self.gripper_position = None
        self.gimbal_sub = None
        self.node = node
        if node is not None:
            self.get_sn_srv = node.create_service(Trigger, 'get_sn', self.get_sn)
            self.set_speed_chassis_sub = node.create_subscription(Twist, 'cmd_vel', self.drive_speed, 10)
            #self.move_chassis_srv = node.create_service(MoveChassis, 'move_chassis', self.move_chassis)
            self.close_robot_srv = node.create_service(Empty, 'close_robot', self.close_robot)
            self.led_srv = node.create_service(RobotLED, 'led_chassis', self.set_led)
            # if params["chassis_position"]:
            #     self.position_pub = node.create_publisher(PointStamped, 'position', 10)
            #     self.position_timer = node.create_timer(0.1, self.pub_position)
            #     self.position_msg = PointStamped()
            #     self.position_msg.header.frame_id = self._conf._name
            #if params["gimbal_angle"]:
            #     self.gimbal_angle_pub = node.create_publisher(GimbalAngle, 'gimbal_angle', 10)
            #     self.gimbal_angle_timer = node.create_timer(0.1, self.pub_gimbal_angle)
            #      self.gimbal_angle_msg = GimbalAngle()
            if params["arm_position"]:
                self.arm_position_msg = PointStamped()
                self.arm_position_msg.header.frame_id = self._conf._name

    def __del__(self):
        self._close_robot()

    def _initialize(self):
        try:
            self._client.start()
            if not self._client:
                return False
            self._action_dispatcher = ActionDispatcher(self._client)
            self._action_dispatcher.initialize()
            time.sleep(0.1)
            ret = self._enable_sdk(1)
            ret = ret & self._sub_node_reset()

            self._running = True
            self._start_heart_beat_timer()
            self._initialized = True
            if not ret:
                logger.error("Robot: initialize failed.")
                self._close_robot() 
            try:
                s1 = self._move_gimbal(0, 0, async_flag=False)
                s1 = s1.wait_for_completed(timeout=3)
            except:
                pass
            if s1:
                ret = ret & self.set_robot_mode(mode=CHASSIS_LEAD)
                self.move_gimbal_srv = self.node.create_service(MoveGimbal, 'move_gimbal', self.move_gimbal)
                self.shoot_sub_srv = self.node.create_service(Fire, 'fire', self.fire)
                self.gimbal_sub = self.node.create_subscription(GimbalVel, 'gimbal_cmd_vel', self.gimbal_cmd_vel, 60)
                # TODO
                # self.led_gimbal_srv = self.node.create_service(GimbalLED, 'led_gimbal', self.led.set_led)
            else:
                ret = ret & self.set_robot_mode(mode=FREE)
                self.move_arm_srv = self.node.create_service(MoveArm, 'move_arm', self.move_arm)
                self.gripper_srv = self.node.create_service(Gripper, 'gripper', self.gripper)
                if self.arm_position_msg:
                    self.sub_arm_position()
                    self.arm_position_pub = self.node.create_publisher(PointStamped, 'arm_position', 10)
                    # TODO verify
                    #self.arm_position_timer = self.node.create_timer(0.1, self.sub_arm_position)
                    logger.info(f"{self._conf._name} arm_position_pub created.")
            return ret
        except Exception as e:
            logger.error(f"Robot: initialize, exception {e}")
            return False

    def _close_robot(self):
        #ep_led = Led(self)
        #ep_led.set_led(comp=COMP_ALL, r=255, g=255, b=255, effect=EFFECT_ON)
        if self._initialized:
            time.sleep(0.1)
            self._enable_sdk(0)
            self._stop_heart_beat_timer()
        if self._client:
            self._client.stop()
        self._initialized = False
        logger.info("Robot close")

    def close_robot(self, request, response:Empty.Response):
        self._close_robot()
        return response

    def _get_sn(self):
        proto = protocol.ProtoGetSn()
        msg = protocol.Msg(self._client.hostbyte, protocol.host2byte(8, 1), proto)
        try:
            resp_msg = self._client.send_sync_msg(msg)
            if resp_msg:
                proto = resp_msg.get_proto()
                if proto:
                    return proto._sn
                else:
                    return None
            else:
                logger.warning("Robot: get_sn failed.")
                return None
        except Exception as e:
            logger.warning("Robot: get_sn, send_sync_msg exception {0}".format(str(e)))
            return None
        
    def get_sn(self, request, response:Trigger.Response):
        sn = self._get_sn()
        if sn!=None:
            response.success = True
            response.message = sn
        else:
            response.success = False
            response.message = "get_sn failed."
        return response
    
    '''
    def _move_chassis(self, x=0., y=0., z=0., xy_speed=0.5, z_speed=30):
        chassis_action = ChassisMoveAction(x, y, z, xy_speed, z_speed)
        self._action_dispatcher.send_async_action(chassis_action) 
        return chassis_action
    
    def move_chassis(self, request, response:MoveChassis.Response):
        x = request.x
        y = request.y
        w = request.w
        xy_speed = request.xy_speed
        w_speed = request.w_speed
        fa = self._move_chassis(x,y,w,xy_speed,w_speed)
        if fa:
            response.success = True
        else:
            response.success = False
        return response
    '''

    def _gripper(self, position=1, async_flag=True, retry=3):
        proto = protocol.ProtoGripperCtrl()
        if position == 1 or position == 2:
            proto._control = position
        else:
            proto._control = 0 # Default pause
        power = 50
        proto._power = int(util.GRIPPER_POWER_CHECK.val2proto(power))
        if async_flag:
            self._send_async_proto(proto, protocol.host2byte(3, 6))
        else:
            while retry > 0:
                if self._send_sync_proto(proto, protocol.host2byte(3, 6)):
                    logger.info(f"Gripper position set to {position}")
                    return True
                time.sleep(0.1)
                retry -= 1
            return False
        return True
            
    def gripper(self, request, response:Gripper.Response):
        distance = request.distance

        time_to_sleep, ret, dir = self.calculate_gripper_movement(distance)
        if not ret:
            response.success = False
            return response
        if time_to_sleep == 0:
            response.success = True
            return response

        fa = self._gripper(dir, async_flag=False)

        time.sleep(time_to_sleep)

        fa = self._gripper(0) #pause
        if fa:
            self.gripper_position = distance
            response.success = True
        else:
            response.success = False
        return response

    def calculate_gripper_movement(self, distance):
        time_to_sleep = 0
        if distance not in [0, 25, 50, 75, 100]:  # Check for valid distance early
            logger.warning("Invalid distance for gripper")
            return time_to_sleep, False, 0

        to_move = distance - self.gripper_position
        if to_move == 0:
            logger.info("Gripper already at position")
            return time_to_sleep, True, 0
        
        # Define the time increments for each movement
        time_increments = {
            25: 0.32,
            50: 0.63,
            75: 0.84,
            100: 1.25
        }
        time_to_sleep = time_increments[abs(to_move)]
        return time_to_sleep, True, 1 if to_move > 0 else 2

    def _move_arm(self, x=0., z=0., async_flag=True):
        """ Relative positional movement of the robot arm
        """
        x = x * 1000 # Convert from m to mm
        z = z * 1000
        arm_action = RoboticArmMoveAction(x=x, y=0, z=z, mode=0)
        if async_flag:
            self._action_dispatcher.send_async_action(arm_action)
        else:
            self._action_dispatcher.send_action(arm_action)
        return arm_action
    
    def move_arm(self, request, response:MoveArm.Response):
        x = request.x
        z = request.z
        if abs(x)>0.22 or abs(z)>0.15:
            logger.warning("Invalid arm position, will modify accordingly")
        x = -0.22 if x < -0.22 else x
        x = 0.22 if x > 0.22 else x
        z = -0.15 if z < -0.15 else z
        z = 0.15 if z > 0.15 else z
        self._move_arm(x,z)
        start_time = time.time()
        while time.time() - start_time < 2:
            logger.info(f"{self.arm_position_data}")
            time.sleep(0.1)
        response.success = True
        return response
    
    def _move_gimbal(self, pitch=0, yaw=0, pitch_speed=30, yaw_speed=30, async_flag=True):  
        """ Controls the head to move to the specified position, the origin of the axis is the current position.

        :param pitch: float: [-55, 55], pitch axis angle in °.
        :param yaw: float: [-500, 500], angle of the yaw axis in °.
        :param pitch_speed: float: [0, 540], pitch axis motion speed in °/s
        :param yaw_speed: float: [0, 540], yaw axis speed in °/s
        :return: Returns the action object
        """
        pitch = int(util.GIMBAL_PITCH_MOVE_CHECKER.val2proto(pitch))
        yaw = int(util.GIMBAL_YAW_MOVE_CHECKER.val2proto(yaw))
        gimble_action = GimbalMoveAction(pitch, yaw, pitch_speed, yaw_speed, COORDINATE_CUR)
        if async_flag:
            self._action_dispatcher.send_async_action(gimble_action)
        else:
            self._action_dispatcher.send_action(gimble_action)
        return gimble_action
    
    def move_gimbal(self, request, response:MoveGimbal.Response):
        #Convert from rad to deg
        pitch = request.pitch 
        yaw = request.yaw 
        pitch_speed = request.pitch_speed
        yaw_speed = request.yaw_speed 
        fa = self._move_gimbal(pitch, yaw, pitch_speed, yaw_speed)
        if fa:
            response.success = True
        else:
            response.success = False
        return response
    
    def gimbal_cmd_vel(self, data:GimbalVel):
        proto = protocol.ProtoGimbalRotate()
        proto._pitch = int(np.sign(data.pitch_speed)*3)
        proto._yaw = int(np.sign(data.yaw_speed)*3)
        proto._pitch_speed = int(util.GIMBAL_PITCH_MOVE_SPEED_SET_CHECKER.val2proto(abs(data.pitch_speed)))
        proto._yaw_speed = int(util.GIMBAL_YAW_MOVE_SPEED_SET_CHECKER.val2proto(abs(data.yaw_speed)))
        proto._coordinate = COORDINATE_CUR
        logger.info(f"gimbal_cmd_vel: {proto._pitch=} {proto._yaw=}")
        self._send_async_proto(proto, protocol.host2byte(4, 0))

    def test_arm(self):
        arm_action = RoboticArmMoveAction(x=0, y=0, z=0, mode=0)
        self._action_dispatcher.send_action(arm_action) 
        return arm_action
    
    def test_gimbal(self):
        action1 = GimbalMoveAction(0, 0, 30, 30, COORDINATE_CUR)
        self._action_dispatcher.send_action(action1) 
        return action1
    
    def _fire(self, type='ir', times=1, async_flag=True):
        """ Fire the robot

        :param type: str: 'ir' or 'bids', fire type
        :param times: int: number of shots
        :param async_flag: bool: whether to send asynchronously
        """
        proto = protocol.ProtoBlasterFire()
        if type == 'bids':
            proto._type = 0
        elif type == 'ir':
            proto._type = 1
        else:
            proto._type = 1
        proto._times = util.FIRE_TIMES_CHECKER.val2proto(times)
        if async_flag:
            self._send_async_proto(proto, protocol.host2byte(23, 0))
        else:
            self._send_sync_proto(proto, protocol.host2byte(23, 0))
    
    def fire(self, request, response:Fire.Response):
        type = request.type
        times = request.times
        self._fire(type, times)
        return response

    def _drive_speed(self, x=0.0, y=0.0, z=0.0, timeout=None):
        """ Set the chassis speed with immediate effect

        :param x: float:[-3.5,3.5], x-axis speed, i.e. forward speed, in m/s
        :param y: float:[-3.5,3.5], y-axis speed, i.e. traverse speed, in m/s
        :param z: float:[-600,600], z-axis speed, i.e. rotation speed, in °/s
        :param timeout: float:(0,inf), the robot stops after the specified time has 
        elapsed without receiving a command to rotate the wheels, in s
        """
        host = protocol.host2byte(3, 6)
        proto = protocol.ProtoChassisSpeedMode()
        z = z * 180 / 3.1415926 #Convert from rad/s to deg/s
        proto._x_spd = util.CHASSIS_SPD_X_CHECKER.val2proto(x)
        proto._y_spd = util.CHASSIS_SPD_Y_CHECKER.val2proto(y)
        proto._z_spd = util.CHASSIS_SPD_Z_CHECKER.val2proto(z)
        logger.debug("x_spd:{0:f}, y_spd:{1:f}, z_spd:{2:f}".format(proto._x_spd, proto._y_spd, proto._z_spd))
        if timeout:
            if self._auto_timer:
                if self._auto_timer.is_alive():
                    self._auto_timer.cancel()
            self._auto_timer = threading.Timer(timeout, self._auto_stop_timer, args=("drive_speed",))
            self._auto_timer.start()
            return self._send_async_proto(proto, host)
        return self._send_async_proto(proto, host)

    def drive_speed(self, data):
        x = data.linear.x
        y = data.linear.y
        z = data.angular.z
        self._drive_speed(x, y, z)
    
    def _auto_stop_timer(self, api="drive_speed"):
        if api == "drive_speed":
            logger.debug("Chassis: drive_speed timeout, auto stop!")
            self._drive_speed(0, 0, 0)
        else:
            logger.warning("Chassis: unsupported api:{0}".format(api))
    
    def _send_sync_proto(self, proto, host, timeout=1.0, target=None):
        if not self._client:
            return False

        if target:
            msg = protocol.Msg(self._client.hostbyte, target, proto)
        else:
            msg = protocol.Msg(self._client.hostbyte, host, proto)
        try:
            resp_msg = self._client.send_sync_msg(msg, timeout=timeout)
            if resp_msg:
                proto = resp_msg.get_proto()
                if proto._retcode == 0: 
                    return True
                else:
                    logger.warning("{0}: send_sync_proto, proto:{1}, retcode:{2} ".format(self.__class__.__name__,
                                                                                          proto,
                                                                                          proto._retcode)) 
                    return False
            else:
                logger.warning("{0}: send_sync_proto, proto:{1} resp_msg is None.".format(
                    self.__class__.__name__, proto))
                return False
        except Exception as e:
            logger.warning("{0}: send_sync_proto, proto:{1}, exception:{2}".format(self.__class__.__name__, proto, e))
            return False

    def _send_async_proto(self, proto, target:protocol.host2byte=None):
        if not self._client:
            return False

        msg = protocol.Msg(self._client.hostbyte, target, proto)
        try:
            return self._client.send_async_msg(msg)
        except Exception as e:
            logger.error("{0}: _send_async_proto, proto:{1}, exception:{2}".format(self.__class__.__name__, proto, e))
            return False

    def _enable_sdk(self, enable=1, retry=3):   
        """ Entering and exiting SDK mode

        :param enable: enter or exit SDK mode, 1 is to enter SDK mode, 0 is to exit SDK mode
        """
        proto = protocol.ProtoSetSdkMode()
        proto._enable = enable
        msg = protocol.Msg(self._client.hostbyte, protocol.host2byte(9, 0), proto)
        while retry > 0:
            try:
                resp_msg = self._client.send_sync_msg(msg, timeout=1)
                if resp_msg:
                    return True
                else:
                    logger.warning(f"{self._conf._name} enable_sdk retrying...")
                    retry -= 1
                    continue
            except Exception as e:
                logger.warning(f"{self._conf._name} enable_sdk, send_sync_msg exception {e}")
                retry -= 1
                continue
        logger.error(f"{self._conf._name} enable_sdk failed.")
        return False

    def _sub_node_reset(self, retry=3):
        proto = protocol.ProtoSubNodeReset()
        proto._node_id = self._client.hostbyte
        msg = protocol.Msg(self._client.hostbyte, protocol.host2byte(9, 0), proto)
        while retry > 0:
            try:
                resp_msg = self._client.send_sync_msg(msg)
                if resp_msg:
                    return True
                else:
                    logger.warning(f"{self._conf._name} sub_node_reset retrying...")
                    retry -= 1
                    continue
            except Exception as e:
                logger.warning(f"{self._conf._name} sub_node_reset, send_sync_msg exception {e}")
                retry -= 1
                continue
        logger.error(f"{self._conf._name} sub_node_reset failed.")
        return False
    
    def set_robot_mode(self, mode=CHASSIS_LEAD, retry=3):
        """ Setting the robot working mode

        :param mode: robot working mode: free means free mode; 
        chassis_lead means chassis follow chassis mode; gimbal_lead means chassis follow gimbal mode \
        :return: bool: call result
        """
        proto = protocol.ProtoSetRobotMode()
        if mode == FREE:
            proto._mode = 0
        elif mode == GIMBAL_LEAD:
            proto._mode = 1
            self.reset_robot_mode()
        elif mode == CHASSIS_LEAD:
            proto._mode = 2
            self.reset_robot_mode()
        else:
            logger.error(f"{self._conf._name} set_robot_mode, mode {mode} is not supported.")
            return False    
        msg = protocol.Msg(self._client.hostbyte, protocol.host2byte(9, 0), proto)
        while retry > 0:
            try:
                resp_msg = self._client.send_sync_msg(msg)
                if resp_msg:
                    return True
                else:
                    logger.warning(f"{self._conf._name} set_robot_mode retrying...")
                    retry -= 1
                    continue
            except Exception as e:
                logger.warning(f"{self._conf._name} set_robot_mode, send_sync_msg exception {e}")
                retry -= 1
                continue
        logger.error(f"{self._conf._name} set_robot_mode failed.")
        return False
        
    def reset_robot_mode(self, retry=3):
        proto = protocol.ProtoSetRobotMode()
        proto._mode = 0
        msg = protocol.Msg(self._client.hostbyte, protocol.host2byte(9, 0), proto)
        while retry > 0:
            try:
                resp_msg = self._client.send_sync_msg(msg)
                if resp_msg:
                    return True
                else:
                    logger.warning(f"{self._conf._name} reset_robot_mode retrying...")
                    retry -= 1
                    continue
            except Exception as e:
                logger.warning(f"{self._conf._name} reset_robot_mode, send_sync_msg exception {e}")
                retry -= 1
                continue
        logger.error(f"{self._conf._name} reset_robot_mode failed.")
        return False

    def _set_led(self, cmd=None, r=0, g=0, b=0, effect=None, freq=None, retry=3):
        """ Set the LED light of the robot

        :param comp: int: [0, 1, 2, 3], set the component to be controlled \
            0 chassis, 1 gimbal, 2 arm, 3 all
        :param r: int: [0, 255], red light brightness
        :param g: int: [0, 255], green light brightness
        :param b: int: [0, 255], blue light brightness
        :param effect: int: [0, 1, 2, 3], set the light effect \
            0 always on, 1 breathing light, 2 flashing light, 3 off
        :return: bool: call result
        """
        proto = protocol.ProtoSetSystemLed()
        proto._ctrl_mode = 7
        proto._comp_mask = cmd
        proto._r = int(util.COLOR_VALUE_CHECKER.val2proto(r))
        proto._g = int(util.COLOR_VALUE_CHECKER.val2proto(g))
        proto._b = int(util.COLOR_VALUE_CHECKER.val2proto(b))
        proto._effect_mode = effect
        proto._t1 = freq
        proto._t2 = freq
        msg = protocol.Msg(self._client.hostbyte, protocol.host2byte(24, 0), proto)

        while retry > 0:
            try:
                resp_msg = self._client.send_sync_msg(msg, timeout=0.5)
                if resp_msg:
                    return True
                else:
                    logger.warning(f"{self._conf._name} set_led retrying...")
                    retry -= 1
                    continue
            except Exception as e:
                logger.warning(f"{self._conf._name} set_led, send_sync_msg exception {e}")
                retry -= 1
                continue
        logger.error(f"{self._conf._name} set_led failed.")
        return False
    
    def set_led(self, request:RobotLED.Request, response:RobotLED.Response):
        response.success = False
        CMD_MAP = {
            "all": ARMOR_BOTTOM_ALL,
            "front": ARMOR_BOTTOM_FRONT,
            "back": ARMOR_BOTTOM_BACK,
            "left": ARMOR_BOTTOM_LEFT,
            "right": ARMOR_BOTTOM_RIGHT
        }
        effect = 0
        freq = 0
        if request.on:
            effect = 1
            _eff = request.effect.casefold()
            if _eff == "flash":
                effect = 3
                freq = int(500 / max(request.freq, 1))
            elif _eff == "breath":
                effect = 2
                freq = 1000

        cmd = CMD_MAP.get(request.which, None)
        if cmd is None:
            response.message = f"{self._conf._name} set_led, invalid led component {request.which}"
            return response
        
        ret = self._set_led(cmd=cmd, r=request.led.r, g=request.led.g, b=request.led.b, effect=effect, freq=freq)
        response.success = ret
        response.message = "LED set successfully"

    def _start_heart_beat_timer(self):
        if self._running:
            self._send_heart_beat_msg()

    def _stop_heart_beat_timer(self):
        if self._send_heart_beat_timer:
            self._send_heart_beat_timer.cancel()
            self._send_heart_beat_timer = None
    
    def _send_heart_beat_msg(self):
        proto = protocol.ProtoSdkHeartBeat()
        msg = protocol.Msg(self._client.hostbyte, protocol.host2byte(9, 0), proto)
        try:
            self._client.send_msg(msg)
        except Exception as e:
            logger.warning("Robot: send heart beat msg failed, exception {0}".format(e))
        if self._running:
            self._send_heart_beat_timer = threading.Timer(1, self._send_heart_beat_msg)
            self._send_heart_beat_timer.start()

    def _sub_position(self, cs=0, freq=5, callback=None, *args, **kw):
        """ Subscribe to chassis position information

        :param cs: int: [0,1] set the coordinate system of the chassis position, \
            0 the current position of the robot, 1 the robot power-up position
        :param freq: enum: (1, 5, 10, 20, 50) Set the frequency of pushing the data subscription data, in Hz
        :param callback: callback function, return data (x, y, z).

                        :x: distance in x-axis, in m
                        :y: distance in y-axis, in m
                        :z: distance in the z-direction, in m

        :param args: variable parameter
        :param kw: keyword parameter
        :return: bool: data subscription result
        """
        sub = self.dds
        subject = PositionSubject(cs)
        subject.freq = freq
        return sub.add_subject_info(subject, callback, args, kw)
    
    def unsub_position(self):
        """ Unsubscribe from chassis location information

        :return: bool: unsubscribe data result
        """
        sub_dds = self.dds
        return sub_dds.del_subject_info(dds.DDS_POSITION)
    
    def get_position_callback(self, data):
        self.position_data = list(data)

    def sub_position(self):
        self._sub_position(freq=10, callback=self.get_position_callback)

    def pub_position(self):
        if self.position_data and self.position_pub:
            self.position_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.position_msg.point.x = self.position_data[0]
            self.position_msg.point.y = self.position_data[1]
            self.position_msg.point.z = self.position_data[2]
            self.position_pub.publish(self.position_msg)
        else:
            logger.info("Position data not available or is None")

    def _sub_gimbal_angle(self, freq=5, callback=None, *args, **kw):
        """ Subscribe to gimbal attitude angle information

        :param freq: enum: (1, 5, 10, 20, 50) sets the frequency of push for the data subscription, in Hz
        :param callback: callback function that returns data (pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle).

                        :pitch_angle: angle of the pitch axis relative to the chassis
                        :yaw_angle: angle of yaw axis relative to the chassis
                        :pitch_ground_angle: angle of pitch axis at the time of power-up
                        :yaw_ground_angle: yaw axis angle at power-up time

        :param args: variable parameter
        :param kw: keyword parameter
        :return: bool: data subscription result
        """
        sub = self.dds
        subject = GimbalPosSubject()
        subject.freq = freq
        return sub.add_subject_info(subject, callback, args, kw)

    def unsub_gimbal_angle(self):
        """ Cancel Gimbal Attitude Angle Subscription

        :return: bool: Result of canceling data subscription
        """
        sub_dds = self.dds
        return sub_dds.del_subject_info(dds.DDS_GIMBAL_POS)
    
    def get_gimbal_angle_callback(self, data):
        self.gimbal_angle_data = list(data)
    
    def sub_gimbal_angle(self):
        self._sub_gimbal_angle(freq=10, callback=self.get_gimbal_angle_callback)

    def pub_gimbal_angle(self):
        if self.gimbal_angle_data and self.gimbal_angle_pub:
            self.gimbal_angle_msg.pitch_angle = self.gimbal_angle_data[0]
            self.gimbal_angle_msg.yaw_angle = self.gimbal_angle_data[1]
            self.gimbal_angle_msg.pitch_ground_angle = self.gimbal_angle_data[2]
            self.gimbal_angle_msg.yaw_ground_angle = self.gimbal_angle_data[3]
            self.gimbal_angle_pub.publish(self.gimbal_angle_msg)
        else:
            logger.info("Gimbal angle data not available or is None")

    def _sub_arm_position(self, freq=5, callback=None, *args, **kw):
        """ Subscribe to the position information of the robotic arm

        :param freq: enum:(1,5,10,20,50) sets the push frequency of the data subscription data in Hz
        :param callback: callback function, return data (pos_x, pos_y):

                        :pos_x: x-axis position information of the robot arm
                        :pos_y: position of the arm in y-axis

        :param args: variable parameter
        :param kw: keyword parameter
        :return: bool: Data subscription result
        """
        sub = self.dds
        subject = ArmSubject()
        subject.freq = freq
        return sub.add_subject_info(subject, callback, args, kw)

    def unsub_arm_position(self):
        """ Unsubscribe from robotic arm position information

        :return: bool: unsubscribe result
        """
        sub_dds = self.dds
        return sub_dds.del_subject_info(dds.DDS_ARM)
    
    def get_arm_position_callback(self, data):
        self.arm_position_data = [data[0], data[1] if data[1] < 1000 else data[1]-4294967295]
        #logger.info(f"Arm position data: {self.arm_position_data}") 
        if self.arm_position_data and self.arm_position_pub:
            self.arm_position_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.arm_position_msg.point.x = float(self.arm_position_data[0])
            self.arm_position_msg.point.z = float(self.arm_position_data[1])
            self.arm_position_pub.publish(self.arm_position_msg)

    def sub_arm_position(self):
        self._sub_arm_position(freq=10, callback=self.get_arm_position_callback)

class Led():
    """ EP Armor Light Module """

    _host = protocol.host2byte(24, 0)

    def __init__(self, robot:Robot):
        super().__init__()
        self.client = robot._client

    def set_led(self, comp=COMP_ALL, r=0, g=0, b=0, effect=EFFECT_ON, freq=1):
        """ Setting the whole armor light effect

        :param comp: enum: ("all", "top_all", "top_right", "top_left", "bottom_all", "bottom_front", \
        "bottom_back", "bottom_left", "bottom_right") light effect parts, all: all armor lights; top_all: all armor lights on the gimbal; \
        top_right: right armor light of the gimbal; top_left: left armor light of the gimbal; bottom_all: all armor lights of the chassis; bottom_front: front armor light; \
        bottom_back: rear armor light; bottom_left: left armor light; bottom_right: right armor light
        :param r: int: [0~255], RGB red color component value
        :param g: int: [0~255], RGB green component value
        :param b: int: [0~255], RGB blue component value
        :param effect: enum: ("on", "off", "flash", "breath", "scrolling") type of light effect, on: always on; off: always off; flash: flashing; \
        breath: breath; scrolling: scrolling (only valid for gimbals)
        :param freq: int: [1, 10], frequency of flashing, valid only for flashing lamps
        :return: bool:call result
        """
        comp_mask = 0x0
        if comp == COMP_ALL:
            comp_mask = ARMOR_ALL
        elif comp == COMP_TOP_ALL:
            comp_mask = ARMOR_TOP_ALL
        elif comp == COMP_TOP_LEFT:
            comp_mask = ARMOR_TOP_LEFT
        elif comp == COMP_TOP_RIGHT:
            comp_mask = ARMOR_TOP_RIGHT
        elif comp == COMP_BOTTOM_ALL:
            comp_mask = ARMOR_BOTTOM_ALL
        elif comp == COMP_BOTTOM_BACK:
            comp_mask = ARMOR_BOTTOM_BACK
        elif comp == COMP_BOTTOM_LEFT:
            comp_mask = ARMOR_BOTTOM_LEFT
        elif comp == COMP_BOTTOM_FRONT:
            comp_mask = ARMOR_BOTTOM_FRONT
        elif comp == COMP_BOTTOM_RIGHT:
            comp_mask = ARMOR_BOTTOM_RIGHT
        else:
            logger.warning("Led: set_led, not support comp:{0}".format(comp))
            return False

        proto = protocol.ProtoSetSystemLed()
        proto._ctrl_mode = 7
        proto._comp_mask = comp_mask
        proto._r = int(util.COLOR_VALUE_CHECKER.val2proto(r))
        proto._g = int(util.COLOR_VALUE_CHECKER.val2proto(g))
        proto._b = int(util.COLOR_VALUE_CHECKER.val2proto(b))
        if effect is EFFECT_OFF:
            proto._effect_mode = 0
        elif effect is EFFECT_ON:
            proto._effect_mode = 1
        elif effect is EFFECT_BREATH:
            proto._effect_mode = 2
            proto._t1 = 1000
            proto._t2 = 1000
        elif effect is EFFECT_FLASH:
            proto._effect_mode = 3
            if freq == 0:
                logger.warning("Led: set_led: freq is zero.")
                freq = 1
            t = int(500/freq)
            proto._t1 = t
            proto._t2 = t
        elif effect is EFFECT_SCROLLING:
            proto._effect_mode = 4
            proto._t1 = 30
            proto._t2 = 40
            proto._led_mask = 0x0f
        else:
            logger.warning("Led: set_led, unsupported effect {0}".format(effect))

        return self._send_sync_proto(proto, protocol.host2byte(9, 0))
    
    def set_gimbal_led(self, comp=COMP_TOP_ALL, r=255, g=255, b=255, led_list=[0, 1, 2, 3], effect=EFFECT_ON):
        """ Setting the gimbal lighting effect

        :param comp: enum: ("top_all", "top_left", "top_right"), head part
        :param r: int: [0, 255], RGB red component value
        :param g: int: [0, 255], RGB green component value
        :param b: int: [0, 255], RGB blue component value
        :param led_list: list [idx0, idx1, ...], idx: int: [0, 255], RGB blue component value , idx: int[0,7] list of led serial numbers.
        :param effect: enum: ("on", "off"), type of light effect
        :return: bool: result of the call
        """
        comp_mask = 0x0
        if comp == COMP_ALL:
            comp_mask = ARMOR_ALL
        elif comp == COMP_TOP_ALL:
            comp_mask = ARMOR_TOP_ALL
        elif comp == COMP_TOP_LEFT:
            comp_mask = ARMOR_TOP_LEFT
        elif comp == COMP_TOP_RIGHT:
            comp_mask = ARMOR_TOP_RIGHT
        else:
            logger.warning("Led: set_gimbal_led, not support comp:{0}".format(comp))
            return False

        proto = protocol.ProtoSetSystemLed()
        proto._ctrl_mode = 7
        proto._comp_mask = comp_mask
        proto._led_mask = 0
        for i in range(0, len(led_list)):
            proto._led_mask += 1 << (led_list[i] % 8)
        proto._r = int(util.COLOR_VALUE_CHECKER.val2proto(r))
        proto._g = int(util.COLOR_VALUE_CHECKER.val2proto(g))
        proto._b = int(util.COLOR_VALUE_CHECKER.val2proto(b))
        if effect is EFFECT_OFF:
            proto._effect_mode = 0
        elif effect is EFFECT_ON:
            proto._effect_mode = 1
        else:
            logger.warning("Led: set_gimbal_led, unsupported effect {0}".format(effect))

        return self._send_sync_proto(proto, protocol.host2byte(9, 0))
    
    def _send_sync_proto(self, proto, target=None):
        if not self.client:
            return False

        if target:
            msg = protocol.Msg(self.client.hostbyte, target, proto)
        else:
            msg = protocol.Msg(self.client.hostbyte, self._host, proto)
        try:
            resp_msg = self.client.send_sync_msg(msg)
            if resp_msg:
                proto = resp_msg.get_proto()
                if proto._retcode == 0: 
                    return True
                else:
                    logger.warning("{0}: send_sync_proto, proto:{1}, retcode:{2} ".format(self.__class__.__name__,
                                                                                          proto,
                                                                                          proto._retcode)) 
                    return False
            else:
                logger.warning("{0}: send_sync_proto, proto:{1} resp_msg is None.".format(
                    self.__class__.__name__, proto))
                return False
        except Exception as e:
            logger.warning("{0}: send_sync_proto, proto:{1}, exception:{2}".format(self.__class__.__name__, proto, e))
            return False

