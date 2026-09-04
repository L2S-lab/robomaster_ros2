import socket, threading
import time
import random, binascii
import struct
from math import radians
import numpy as np

from rclpy.node import Node
from std_srvs.srv import Empty
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tf_transformations import quaternion_from_euler
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from robomaster_interface.srv import MoveChassis 
from robomaster_interface.srv import MoveArm 
from robomaster_interface.srv import (
    BlasterLED,
    Fire,
    GimbalLED,
    GripperCommand,
    MoveGimbal,
    SetArmorSensitivity,
    SetGripperMode,
)
from robomaster_interface.srv import Gripper 
from robomaster_interface.srv import RobotLED
from robomaster_interface.msg import (
    ArmActionStatus,
    ArmorHit,
    GimbalAngle,
    GimbalVel,
    GripperState,
    IrHit,
)

from rclpy.logging import get_logger
logger = get_logger('robot')

CAMERA_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

ARMOR_ID_TO_COMPONENT = {
    1: 'bottom_back',
    2: 'bottom_front',
    3: 'bottom_left',
    4: 'bottom_right',
    5: 'top_left',
    6: 'top_right',
}

# These masks follow DJI's armor module. They intentionally do not reuse the
# LED constants because the SDK's left/right armor masks differ there.
ARMOR_SENSITIVITY_MASKS = {
    'bottom_back': 1 << 0,
    'bottom_front': 1 << 1,
    'bottom_right': 1 << 2,
    'bottom_left': 1 << 3,
    'top_right': 1 << 4,
    'top_left': 1 << 5,
    'bottom_all': 0x0f,
    'top_all': 0x30,
    'all': 0x3f,
}

GRIPPER_POSITIONS = (0, 25, 50, 75, 100)
GRIPPER_TIME_FRACTIONS = {
    25: 0.32 / 1.25,
    50: 0.63 / 1.25,
    75: 0.84 / 1.25,
    100: 1.0,
}

ARM_MAX_COMPLETION_TIMEOUT = 60.0

try:
    from . import protocol, config, dds, camera
    from .action import ActionDispatcher, Action
    from .action import ACTION_RUNNING, ACTION_SUCCEEDED, ACTION_FAILED, ACTION_STARTED
    from . import util
    from .common import *
    from .globals import *
    from .motion_safety import MotionSafety
except ImportError as e:
    import protocol, config, dds, camera
    from action import ActionDispatcher, Action
    from action import ACTION_RUNNING, ACTION_SUCCEEDED, ACTION_FAILED, ACTION_STARTED
    import util
    from common import *
    from globals import *
    from motion_safety import MotionSafety
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
        # DJI's ProtoRoboticArmMovePush contains only x and y. The command's
        # unused third axis remains unchanged instead of reading a missing z.
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


class ChassisAttitudeSubject(dds.Subject):
    name = dds.DDS_ATTITUDE
    uid = dds.SUB_UID_MAP[name]
    type = dds.DDS_SUB_TYPE_PERIOD

    def __init__(self):
        self._yaw = 0.0
        self._pitch = 0.0
        self._roll = 0.0

    def data_info(self):
        return self._yaw, self._pitch, self._roll

    def decode(self, buf):
        self._yaw, self._pitch, self._roll = struct.unpack('<fff', buf)
        self._yaw = util.CHASSIS_YAW_CHECKER.proto2val(self._yaw)
        self._pitch = util.CHASSIS_PITCH_CHECKER.proto2val(self._pitch)
        self._roll = util.CHASSIS_ROLL_CHECKER.proto2val(self._roll)


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


class BatterySubject(dds.Subject):
    """Decode EP battery percentage from DJI's periodic DDS subject."""

    name = dds.DDS_BATTERY
    uid = dds.SUB_UID_MAP[name]
    type = dds.DDS_SUB_TYPE_PERIOD

    def __init__(self):
        self._percent = 0

    def data_info(self):
        return self._percent

    def decode(self, buf):
        _, _, _, self._percent, _ = struct.unpack('<HhiBB', buf)


class GripperSubject(dds.Subject):
    """Decode the endpoint-only state exposed by the EP gripper."""

    name = dds.DDS_GRIPPER
    uid = dds.SUB_UID_MAP[name]
    type = dds.DDS_SUB_TYPE_PERIOD

    def __init__(self):
        self._status = 0

    def data_info(self):
        return {0: 'normal', 1: 'opened', 2: 'closed'}.get(
            self._status, 'unknown'
        )

    def decode(self, buf):
        self._status = buf[0]


class ArmorHitEvent(dds.Subject):
    """Decode water-bead or IR armor impacts."""

    name = 'hit_event'
    cmdset = protocol.ProtoArmorHitEvent._cmdset
    cmdid = protocol.ProtoArmorHitEvent._cmdid
    type = dds.DDS_SUB_TYPE_EVENT

    def __init__(self):
        self._armor_id = 0
        self._hit_type = 0
        self._strength = 0

    def data_info(self):
        return (
            self._armor_id,
            ARMOR_ID_TO_COMPONENT.get(self._armor_id, 'unknown'),
            {0: 'water', 1: 'ir'}.get(self._hit_type, 'unknown'),
            self._strength,
        )

    def decode(self, buf):
        self._armor_id, self._hit_type, self._strength = buf


class IrHitEvent(dds.Subject):
    """Decode DJI IR-hit metadata and maintain a per-connection hit count."""

    name = 'ir_event'
    cmdset = protocol.ProtoIrHitEvent._cmdset
    cmdid = protocol.ProtoIrHitEvent._cmdid
    type = dds.DDS_SUB_TYPE_EVENT

    def __init__(self):
        self._hit_count = 0
        self._skill_id = 0
        self._role_id = 0
        self._receiver_device = 0
        self._receiver_ir_pin = 0

    def data_info(self):
        return (
            self._hit_count,
            self._skill_id,
            self._role_id,
            self._receiver_device,
            self._receiver_ir_pin,
        )

    def decode(self, buf):
        self._hit_count += 1
        (
            self._skill_id,
            self._role_id,
            self._receiver_device,
            self._receiver_ir_pin,
        ) = buf


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
        params = params or {}
        self._conf = conf
        self._action_dispatcher = None
        self._send_heart_beat_timer = None
        self._running = False
        self._initialized = False
        self._audio_id = 0
        self._client = cli
        self._auto_timer = None
        self._motion_lock = threading.RLock()
        self._safety_require_arm = bool(
            params.get("safety_require_arm", False)
        )
        self.motion_safety = MotionSafety(
            default_timeout=params.get("safety_command_timeout", 0.5),
            initially_armed=not self._safety_require_arm,
            latch_deadman=params.get("safety_latch_deadman", False),
        )
        self._safety_timer = None
        self.dds = dds.Subscriber(self)
        self.dds.start()
        self._dds_running = True
        self.distance_pub = None
        self.position_data = None
        self.chassis_attitude_data = None
        self.chassis_yaw_origin = None
        self.position_pub = None
        self.position_msg = None
        self.gimbal_angle_data = [None]*4
        self.gimbal_angle_msg = None
        self.gimbal_angle_pub = None
        self.arm_position_data = None
        self.arm_position_pub = None
        self.arm_position_msg = None
        self.arm_action_pub = None
        self.arm_action_timer = None
        self.current_arm_action = None
        self.current_arm_command = 'idle'
        self._arm_lock = threading.RLock()
        requested_arm_timeout = float(
            params.get('arm_completion_timeout', 10.0)
        )
        if not np.isfinite(requested_arm_timeout):
            logger.warning(
                f'[{self._conf._name}] Non-finite arm completion timeout; '
                'using 10.0 seconds.'
            )
            requested_arm_timeout = 10.0
        self.arm_completion_timeout = min(
            max(requested_arm_timeout, 0.1), ARM_MAX_COMPLETION_TIMEOUT
        )
        self.battery_percent = None
        self.gripper_position = None
        requested_gripper_mode = str(
            params.get('gripper_control_mode', 'timed')
        ).casefold()
        if requested_gripper_mode not in ('timed', 'feedback'):
            logger.warning(
                f"[{self._conf._name}] Unsupported gripper mode "
                f"'{requested_gripper_mode}'; using timed."
            )
            requested_gripper_mode = 'timed'
        self.gripper_control_mode = requested_gripper_mode
        self.gripper_power = min(
            max(int(params.get('gripper_power', 50)), 1), 100
        )
        self.gripper_full_travel_time = max(
            float(params.get('gripper_full_travel_time', 1.25)), 0.1
        )
        self.gripper_feedback_timeout = max(
            float(params.get('gripper_feedback_timeout', 3.0)), 0.1
        )
        self.gripper_status = 'unknown'
        self.gripper_busy = False
        self.gripper_last_command_success = False
        self.gripper_message = 'Waiting for a gripper command.'
        self.gripper_state_pub = None
        self.gripper_state_timer = None
        self._gripper_lock = threading.RLock()
        self._gripper_worker = None
        self._gripper_cancel_event = None
        self._gripper_feedback_event = threading.Event()
        self.armor_enabled = bool(params.get('armor_enabled', True))
        self.armor_hit_pub = None
        self.ir_hit_pub = None
        self.gimbal_sub = None
        self.led_control = None
        self.camera = None
        self.img_pub = None
        self.img_timer = None
        self.camera_resolution = params.get("camera_resolution", STREAM_540P)
        self.camera_publish_fps = float(
            params.get("camera_publish_fps", 20.0)
        )
        self.publish_position = bool(
            params.get("chassis_position", False)
        )
        self.publish_gimbal_angle = bool(
            params.get("gimbal_angle", False)
        )
        self.camera_cbg = MutuallyExclusiveCallbackGroup()
        self.manipulator_cbg = ReentrantCallbackGroup()
        self.node = node
        if node is not None:
            self.get_sn_srv = node.create_service(Trigger, 'get_sn', self.get_sn)
            self.get_battery_srv = node.create_service(
                Trigger, 'get_battery', self.get_battery
            )
            self.motion_arm_srv = node.create_service(
                SetBool, 'set_armed', self.set_armed
            )
            self.set_speed_chassis_sub = node.create_subscription(Twist, 'cmd_vel', self.drive_speed, 10)
            self.move_chassis_srv = node.create_service(MoveChassis, 'move_chassis', self.move_chassis)
            self.close_robot_srv = node.create_service(Empty, 'close_robot', self.close_robot)
            self.led_srv = node.create_service(RobotLED, 'led_chassis', self.set_led)
            if self.armor_enabled:
                self.armor_hit_pub = node.create_publisher(
                    ArmorHit, 'armor_hit', 10
                )
                self.ir_hit_pub = node.create_publisher(IrHit, 'ir_hit', 10)
                self.armor_sensitivity_srv = node.create_service(
                    SetArmorSensitivity,
                    'set_armor_sensitivity',
                    self.set_armor_sensitivity,
                )
            if params.get("arm_position", False):
                self.arm_position_msg = PointStamped()
                self.arm_position_msg.header.frame_id = self._conf._name
            if params.get("cam", False):
                self.camera = camera.EPCamera(self)
                self.img_pub = self.node.create_publisher(
                    Image, 'image', CAMERA_QOS
                )
                self.bridge = CvBridge()
            if self.motion_safety.default_timeout > 0.0:
                watchdog_period = min(
                    max(self.motion_safety.default_timeout / 2.0, 0.02),
                    0.1,
                )
                self._safety_timer = node.create_timer(
                    watchdog_period, self._safety_watchdog
                )

    def start_video(self, resolution=None, publish_fps=None,
                    strategy="newest"):
        """Start the EP video stream and its ROS image publisher."""
        if self.camera is None or self.img_pub is None:
            logger.error(
                f"[Robot] [{self._conf._name}] camera publishing is disabled."
            )
            return False

        resolution = resolution or self.camera_resolution
        publish_fps = (
            self.camera_publish_fps
            if publish_fps is None else float(publish_fps)
        )
        if publish_fps <= 0.0:
            logger.error(
                f"[Robot] [{self._conf._name}] camera publish FPS must be "
                "greater than zero."
            )
            return False
        if not self.camera.start_video_stream(
                display=False, resolution=resolution):
            return False

        self.pub_strategy = strategy
        self.img_timer = self.node.create_timer(
            1.0 / publish_fps,
            self.img_publisher_callback,
            callback_group=self.camera_cbg,
        )
        return True

    def img_publisher_callback(self):
        frame = self.camera.get_cv2_frame(
            timeout=0.0, strategy=self.pub_strategy
        )
        if frame is None:
            return
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            image_msg.header.frame_id = self._conf._name
            image_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.img_pub.publish(image_msg)
        except CvBridgeError as e:
            logger.error(
                f"[Robot] [{self._conf._name}] image conversion failed: {e}"
            )

    def set_motion_armed(self, armed):
        """Set the motion gate arm state without changing existing APIs."""
        with self._motion_lock:
            if armed:
                return self.motion_safety.arm()
            self.motion_safety.disarm()
            self._safe_stop_channels(('chassis', 'gimbal'))
            return True

    def set_armed(self, request, response:SetBool.Response):
        """Handle the per-robot motion arming service."""
        response.success = self.set_motion_armed(request.data)
        if response.success:
            state = 'armed' if request.data else 'disarmed'
            response.message = f'Motion commands {state}.'
        else:
            response.message = 'Motion gate is closed and cannot be armed.'
        return response

    def _safety_watchdog(self):
        with self._motion_lock:
            expired = self.motion_safety.poll_deadman()
            if not expired:
                return
            self._safe_stop_channels(expired)
        suffix = ' Re-arm required.' if not self.motion_safety.armed else ''
        logger.warning(
            f'[{self._conf._name}] Deadman stopped: '
            f'{", ".join(expired)}.{suffix}'
        )

    def _safe_stop_channels(self, channels):
        if not self._initialized:
            return
        if 'chassis' in channels:
            self._drive_speed(0.0, 0.0, 0.0)
        if 'gimbal' in channels and self.gimbal_sub is not None:
            self._send_gimbal_speed(0.0, 0.0)

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
                return False
            s1 = False
            try:
                s1 = self._move_gimbal(0, 0, async_flag=False)
                s1 = s1.wait_for_completed(timeout=3)
            except Exception:
                # EP Core has no gimbal; continue with its arm/gripper setup.
                s1 = False
            if s1:
                ret = ret & self.set_robot_mode(mode=CHASSIS_LEAD)
                self.move_gimbal_srv = self.node.create_service(MoveGimbal, 'move_gimbal', self.move_gimbal)
                self.reset_gimbal_srv = self.node.create_service(Trigger, 'reset_gimbal', self.reset_gimbal)
                self.shoot_sub_srv = self.node.create_service(Fire, 'fire', self.fire)
                self.gimbal_sub = self.node.create_subscription(GimbalVel, 'gimbal_cmd_vel', self.gimbal_cmd_vel, 60)
                self.led_control = Led(self)
                self.led_gimbal_srv = self.node.create_service(
                    GimbalLED, 'led_gimbal', self.set_gimbal_led
                )
                self.led_blaster_srv = self.node.create_service(
                    BlasterLED, 'led_blaster', self.set_blaster_led
                )
                if self.publish_gimbal_angle:
                    self.gimbal_angle_msg = GimbalAngle()
                    self.gimbal_angle_pub = self.node.create_publisher(
                        GimbalAngle, 'gimbal_angle', 10
                    )
                    logger.info(
                        f"{self._conf._name} gimbal_angle publisher created."
                    )
                self.sub_gimbal_angle()
            else:
                ret = ret & self.set_robot_mode(mode=FREE)
                self.move_arm_srv = self.node.create_service(
                    MoveArm, 'move_arm', self.move_arm,
                    callback_group=self.manipulator_cbg,
                )
                self.move_arm_to_srv = self.node.create_service(
                    MoveArm, 'move_arm_to', self.move_arm_to,
                    callback_group=self.manipulator_cbg,
                )
                self.recenter_arm_srv = self.node.create_service(
                    Trigger, 'recenter_arm', self.recenter_arm,
                    callback_group=self.manipulator_cbg,
                )
                self.reset_arm_srv = self.node.create_service(
                    Trigger, 'reset_arm', self.reset_arm,
                    callback_group=self.manipulator_cbg,
                )
                self.cancel_arm_srv = self.node.create_service(
                    Trigger, 'cancel_arm', self.cancel_arm,
                    callback_group=self.manipulator_cbg,
                )
                self.arm_action_pub = self.node.create_publisher(
                    ArmActionStatus, 'arm_action_status', 10
                )
                self.arm_action_timer = self.node.create_timer(
                    0.1,
                    self.publish_arm_action_status,
                    callback_group=self.manipulator_cbg,
                )
                self.gripper_srv = self.node.create_service(
                    Gripper, 'gripper', self.gripper,
                    callback_group=self.manipulator_cbg,
                )
                self.gripper_command_srv = self.node.create_service(
                    GripperCommand, 'gripper_command', self.gripper_command,
                    callback_group=self.manipulator_cbg,
                )
                self.gripper_mode_srv = self.node.create_service(
                    SetGripperMode,
                    'set_gripper_mode',
                    self.set_gripper_mode,
                    callback_group=self.manipulator_cbg,
                )
                self.gripper_state_pub = self.node.create_publisher(
                    GripperState, 'gripper_state', 10
                )
                self.gripper_state_timer = self.node.create_timer(
                    0.2,
                    self.publish_gripper_state,
                    callback_group=self.manipulator_cbg,
                )
                self.sub_gripper_status(freq=10)
                if self.publish_gimbal_angle:
                    logger.warning(
                        f"{self._conf._name} has no gimbal; gimbal_angle "
                        "publishing is unavailable."
                    )
                if self.arm_position_msg:
                    self.sub_arm_position()
                    self.arm_position_pub = self.node.create_publisher(PointStamped, 'arm_position', 10)
                    # TODO verify
                    #self.arm_position_timer = self.node.create_timer(0.1, self.sub_arm_position)
                    logger.info(f"{self._conf._name} arm_position_pub created.")
            self.sub_battery_info(freq=1)
            if self.armor_enabled:
                self.sub_armor_events()
            if self.publish_position:
                self.position_msg = PoseStamped()
                self.position_msg.header.frame_id = (
                    f'{self._conf._name}_odom'
                )
                self.position_pub = self.node.create_publisher(
                    PoseStamped, 'position', 10
                )
                self.sub_position()
                self.sub_chassis_attitude()
                logger.info(
                    f"{self._conf._name} position publisher created."
                )
            return ret
        except Exception as e:
            logger.error(f"Robot: initialize, exception {e}")
            return False

    def _close_robot(self):
        #ep_led = Led(self)
        #ep_led.set_led(comp=COMP_ALL, r=255, g=255, b=255, effect=EFFECT_ON)
        if getattr(self, '_initialized', False):
            if getattr(self, 'gripper_state_pub', None) is not None:
                worker = self._gripper_worker
                self._cancel_gripper_command(send_pause=True)
                if worker is not None and worker is not threading.current_thread():
                    worker.join(timeout=1.0)
            action = getattr(self, 'current_arm_action', None)
            if action is not None and not action.is_completed:
                self._cancel_arm_action(action)
        with self._motion_lock:
            self.motion_safety.close()
            self._safe_stop_channels(('chassis', 'gimbal'))
        if self._safety_timer is not None:
            self._safety_timer.cancel()
        if self.img_timer is not None and self.node is not None:
            self.node.destroy_timer(self.img_timer)
            self.img_timer = None
        if self.arm_action_timer is not None and self.node is not None:
            self.node.destroy_timer(self.arm_action_timer)
            self.arm_action_timer = None
        if self.gripper_state_timer is not None and self.node is not None:
            self.node.destroy_timer(self.gripper_state_timer)
            self.gripper_state_timer = None
        if self.camera is not None:
            self.camera.stop()
        if self._initialized:
            time.sleep(0.1)
            self._enable_sdk(0)
            self._running = False
            self._stop_heart_beat_timer()
        if getattr(self, '_dds_running', False):
            self.dds.stop()
            self._dds_running = False
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
    def _sub_battery_info(self, freq=1, callback=None, *args, **kw):
        subject = BatterySubject()
        subject.freq = freq
        return self.dds.add_subject_info(subject, callback, args, kw)

    def sub_battery_info(self, freq=1):
        """Cache EP battery percentage for the synchronous Trigger service."""
        return self._sub_battery_info(
            freq=freq, callback=self.get_battery_callback
        )

    def get_battery_callback(self, percent):
        percent = int(percent)
        if 0 <= percent <= 100:
            self.battery_percent = percent
        else:
            logger.warning(
                f"[{self._conf._name}] Ignoring invalid battery percentage "
                f"{percent}."
            )

    def get_battery(self, request, response: Trigger.Response):
        """Return the latest EP DDS battery sample in the RMTT service format."""
        if self.battery_percent is None:
            response.success = False
            response.message = (
                'Battery percentage unavailable; wait for the first DDS sample.'
            )
        else:
            response.success = True
            response.message = f'{self.battery_percent}%'
        return response

    def _sub_armor_hit_event(self, callback=None, *args, **kw):
        return self.dds.add_subject_event_info(
            ArmorHitEvent(), callback, args, kw
        )

    def _sub_ir_hit_event(self, callback=None, *args, **kw):
        return self.dds.add_subject_event_info(
            IrHitEvent(), callback, args, kw
        )

    def sub_armor_events(self):
        """Subscribe to both physical/water and IR armor event channels."""
        hit_ok = self._sub_armor_hit_event(
            callback=self.get_armor_hit_callback
        )
        ir_ok = self._sub_ir_hit_event(callback=self.get_ir_hit_callback)
        return bool(hit_ok and ir_ok)

    def get_armor_hit_callback(self, data):
        if self.armor_hit_pub is None:
            return
        armor_id, component, hit_type, strength = data
        msg = ArmorHit()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self._conf._name
        msg.armor_id = int(armor_id)
        msg.component = component
        msg.hit_type = hit_type
        msg.strength = int(strength)
        self.armor_hit_pub.publish(msg)

    def get_ir_hit_callback(self, data):
        if self.ir_hit_pub is None:
            return
        hit_count, skill_id, role_id, receiver_device, receiver_ir_pin = data
        msg = IrHit()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self._conf._name
        msg.hit_count = int(hit_count)
        msg.skill_id = int(skill_id)
        msg.role_id = int(role_id)
        msg.receiver_device = int(receiver_device)
        msg.receiver_ir_pin = int(receiver_ir_pin)
        self.ir_hit_pub.publish(msg)

    def _set_armor_sensitivity(self, component, sensitivity):
        component = component.casefold()
        mask = ARMOR_SENSITIVITY_MASKS.get(component)
        if mask is None or not 0 <= sensitivity <= 10:
            return False

        coefficient = 1.5 - sensitivity / 10.0
        proto = protocol.ProtoSetArmorParam()
        proto._armor_mask = mask
        proto._voice_energy_en = 500
        proto._voice_energy_ex = 300
        proto._voice_len_max = 50
        proto._voice_len_min = 13
        proto._voice_len_silence = 6
        proto._voice_peak_count = 1
        proto._voice_peak_min = int(160 * coefficient)
        proto._voice_peak_ave = int(180 * coefficient)
        proto._voice_peak_final = int(200 * coefficient)
        return self._send_sync_proto(proto, protocol.host2byte(24, 1))

    def set_armor_sensitivity(
            self, request: SetArmorSensitivity.Request,
            response: SetArmorSensitivity.Response):
        component = request.component.casefold()
        if component not in ARMOR_SENSITIVITY_MASKS:
            response.success = False
            response.message = (
                'Invalid armor component. Use all, top_all, bottom_all, '
                'top_left, top_right, bottom_left, bottom_right, '
                'bottom_front, or bottom_back.'
            )
            return response
        if request.sensitivity > 10:
            response.success = False
            response.message = 'Armor sensitivity must be in [0, 10].'
            return response

        response.success = bool(self._set_armor_sensitivity(
            component, int(request.sensitivity)
        ))
        response.message = (
            f'Armor sensitivity for {component} set to '
            f'{request.sensitivity}.'
            if response.success else 'Armor sensitivity command failed.'
        )
        return response

    def _gripper(self, position=1, power=None, async_flag=True, retry=3):
        """Send open (1), close (2), or pause (0) to the EP gripper."""
        control = position if position in (1, 2) else 0
        proto = protocol.ProtoGripperCtrl()
        proto._control = control
        if control == 0:
            proto._power = 0
        else:
            power = self.gripper_power if power is None else int(power)
            if not 1 <= power <= 100:
                return False
            proto._power = int(util.GRIPPER_POWER_CHECK.val2proto(power))
        if async_flag:
            self._send_async_proto(proto, protocol.host2byte(3, 6))
            return True
        while retry > 0:
            if self._send_sync_proto(proto, protocol.host2byte(3, 6)):
                return True
            time.sleep(0.1)
            retry -= 1
        return False

    def calculate_gripper_movement(self, distance):
        """Plan a calibrated percentage move without assuming startup state."""
        if distance not in GRIPPER_POSITIONS:
            return 0.0, False, 0
        if self.gripper_position is None:
            if distance not in (0, 100):
                return 0.0, False, 0
            delta = 100
            direction = 1 if distance == 100 else 2
        else:
            delta = distance - self.gripper_position
            if delta == 0:
                return 0.0, True, 0
            direction = 1 if delta > 0 else 2

        duration = (
            self.gripper_full_travel_time
            * GRIPPER_TIME_FRACTIONS[abs(delta)]
        )
        return duration, True, direction

    def _start_gripper_position_command(self, distance, power=None):
        power = self.gripper_power if power is None else int(power)
        if not 1 <= power <= 100:
            return False, 'Gripper power must be in [1, 100].'

        with self._gripper_lock:
            if self.gripper_busy:
                return False, 'The gripper is already executing a command.'
            duration, valid, direction = self.calculate_gripper_movement(
                distance
            )
            if not valid:
                if distance not in GRIPPER_POSITIONS:
                    return False, 'Gripper distance must be 0, 25, 50, 75, or 100.'
                return False, (
                    'Gripper position is unknown. First command 0 or 100, or '
                    'use gripper_command reset to calibrate an endpoint.'
                )
            if direction == 0:
                self.gripper_last_command_success = True
                self.gripper_message = (
                    f'Gripper is already at estimated position {distance}%.'
                )
                self.publish_gripper_state()
                return True, self.gripper_message

            strategy = self.gripper_control_mode
            fallback = strategy == 'feedback' and distance not in (0, 100)
            if fallback:
                strategy = 'timed'
            cancel_event = threading.Event()
            self._gripper_cancel_event = cancel_event
            self._gripper_feedback_event.clear()
            self.gripper_busy = True
            self.gripper_last_command_success = False
            mode_note = (
                'timed fallback because DJI feedback has no intermediate '
                'percentage' if fallback else strategy
            )
            self.gripper_message = (
                f'Moving gripper to {distance}% using {mode_note} mode.'
            )
            worker = threading.Thread(
                target=self._run_gripper_position_command,
                args=(distance, direction, duration, power, strategy,
                      cancel_event),
                daemon=True,
                name=f'{self._conf._name}-gripper',
            )
            self._gripper_worker = worker
            worker.start()
        self.publish_gripper_state()
        return True, self.gripper_message

    def _run_gripper_position_command(
            self, distance, direction, duration, power, strategy,
            cancel_event):
        command_ok = False
        motion_attempted = False
        if not cancel_event.is_set():
            motion_attempted = True
            command_ok = self._gripper(
                direction, power=power, async_flag=False
            )
        reached = False
        if command_ok and strategy == 'feedback':
            deadline = time.monotonic() + self.gripper_feedback_timeout
            while not cancel_event.is_set() and time.monotonic() < deadline:
                with self._gripper_lock:
                    reached = self.gripper_position == distance
                if reached:
                    break
                remaining = deadline - time.monotonic()
                self._gripper_feedback_event.wait(
                    timeout=min(max(remaining, 0.0), 0.1)
                )
                self._gripper_feedback_event.clear()
        elif command_ok:
            reached = not cancel_event.wait(duration)

        pause_ok = self._gripper(0, power=0, async_flag=False)
        cancelled = cancel_event.is_set()
        success = command_ok and pause_ok and reached and not cancelled
        with self._gripper_lock:
            if success:
                self.gripper_position = distance
                self.gripper_message = (
                    f'Gripper reached {distance}% using {strategy} mode.'
                )
            elif cancelled:
                if motion_attempted:
                    self.gripper_position = None
                self.gripper_message = (
                    'Gripper command was paused; recalibrate an endpoint '
                    'before the next intermediate percentage.'
                    if motion_attempted else
                    'Gripper command was cancelled before motion started; '
                    'the previous estimate is unchanged.'
                )
            elif strategy == 'feedback' and command_ok:
                self.gripper_position = None
                self.gripper_message = (
                    'Gripper feedback timed out; position is unknown and an '
                    'endpoint must be recalibrated.'
                )
            else:
                if motion_attempted:
                    self.gripper_position = None
                self.gripper_message = 'Gripper command failed.'
            self.gripper_last_command_success = success
            self.gripper_busy = False
            if self._gripper_cancel_event is cancel_event:
                self._gripper_cancel_event = None
            self._gripper_worker = None
        self.publish_gripper_state()

    def _cancel_gripper_command(self, send_pause=True):
        with self._gripper_lock:
            cancel_event = self._gripper_cancel_event
            if cancel_event is not None:
                cancel_event.set()
        if send_pause and getattr(self, '_initialized', False):
            return self._gripper(0, power=0, async_flag=False)
        return cancel_event is not None

    def gripper(self, request, response: Gripper.Response):
        response.success, response.message = (
            self._start_gripper_position_command(request.distance)
        )
        return response

    def gripper_command(
            self, request: GripperCommand.Request,
            response: GripperCommand.Response):
        command = request.command.casefold()
        if command == 'pause':
            response.success = self._cancel_gripper_command(send_pause=True)
            response.message = (
                'Gripper pause command sent.'
                if response.success else 'Gripper pause command failed.'
            )
            return response
        if command not in ('open', 'close', 'reset'):
            response.success = False
            response.message = 'Gripper command must be open, close, pause, or reset.'
            return response

        distance = 100 if command == 'open' else 0
        power = None if request.power == 0 else request.power
        response.success, response.message = (
            self._start_gripper_position_command(distance, power)
        )
        if response.success and command == 'reset':
            response.message = 'Gripper endpoint calibration accepted. ' + response.message
        return response

    def set_gripper_mode(
            self, request: SetGripperMode.Request,
            response: SetGripperMode.Response):
        mode = request.mode.casefold()
        if mode not in ('timed', 'feedback'):
            response.success = False
            response.message = 'Gripper mode must be timed or feedback.'
            return response
        with self._gripper_lock:
            if self.gripper_busy:
                response.success = False
                response.message = (
                    'Pause or wait for the current gripper command before '
                    'changing mode.'
                )
                return response
            self.gripper_control_mode = mode
            self.gripper_message = f'Gripper control mode set to {mode}.'
        response.success = True
        response.message = self.gripper_message
        self.publish_gripper_state()
        return response

    def _sub_gripper_status(self, freq=10, callback=None, *args, **kw):
        subject = GripperSubject()
        subject.freq = freq
        return self.dds.add_subject_info(subject, callback, args, kw)

    def sub_gripper_status(self, freq=10):
        return self._sub_gripper_status(
            freq=freq, callback=self.get_gripper_status_callback
        )

    def get_gripper_status_callback(self, status):
        with self._gripper_lock:
            self.gripper_status = status
            if status == 'opened':
                self.gripper_position = 100
                self._gripper_feedback_event.set()
            elif status == 'closed':
                self.gripper_position = 0
                self._gripper_feedback_event.set()
        self.publish_gripper_state()

    def publish_gripper_state(self):
        if self.gripper_state_pub is None:
            return
        with self._gripper_lock:
            msg = GripperState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = self._conf._name
            msg.status = self.gripper_status
            msg.estimated_position = (
                -1 if self.gripper_position is None
                else int(self.gripper_position)
            )
            msg.position_known = self.gripper_position is not None
            msg.control_mode = self.gripper_control_mode
            msg.busy = self.gripper_busy
            msg.last_command_success = self.gripper_last_command_success
            msg.message = self.gripper_message
        self.gripper_state_pub.publish(msg)

    def _move_arm(self, x=0.0, z=0.0, absolute=False):
        """Start a tracked relative or absolute arm action."""
        arm_action = RoboticArmMoveAction(
            x=x * 1000.0,
            y=0,
            z=z * 1000.0,
            mode=1 if absolute else 0,
        )
        self._action_dispatcher.send_action(arm_action)
        return arm_action

    def _start_arm_action(self, x, z, absolute, command):
        with self._arm_lock:
            if (
                self.current_arm_action is not None
                and not self.current_arm_action.is_completed
            ):
                return None, 'The robotic arm is already executing an action.'
            try:
                action = self._move_arm(x=x, z=z, absolute=absolute)
            except Exception as exc:
                logger.error(
                    f'[{self._conf._name}] Failed to start arm action: {exc}'
                )
                return None, f'Failed to start robotic-arm action: {exc}'
            self.current_arm_action = action
            self.current_arm_command = command
        self.publish_arm_action_status()
        return action, (
            f'Robotic-arm {command} accepted as action '
            f'{action._action_id}.'
        )

    @staticmethod
    def _bounded_relative_arm_target(x, z):
        bounded_x = min(max(x, -0.22), 0.22)
        bounded_z = min(max(z, -0.15), 0.15)
        return bounded_x, bounded_z, (bounded_x != x or bounded_z != z)

    def _fill_arm_response(self, response, action, accepted, message):
        response.success = bool(accepted)
        if action is None:
            response.completed = False
            response.action_id = 0
            response.percent = 0
            response.state = 'action_rejected'
        else:
            response.completed = bool(action.is_completed)
            response.action_id = int(action._action_id)
            response.percent = int(action._percent)
            response.state = action.state
        response.message = message
        return response

    def _handle_arm_request(self, request, response, absolute):
        x = float(request.x)
        z = float(request.z)
        if not np.isfinite(x) or not np.isfinite(z):
            return self._fill_arm_response(
                response, None, False, 'Arm coordinates must be finite.'
            )

        timeout = None
        if request.wait_for_completion:
            requested_timeout = float(request.timeout)
            if not np.isfinite(requested_timeout):
                return self._fill_arm_response(
                    response, None, False, 'Arm timeout must be finite.'
                )
            timeout = (
                requested_timeout
                if requested_timeout > 0.0
                else self.arm_completion_timeout
            )
            if timeout > ARM_MAX_COMPLETION_TIMEOUT:
                return self._fill_arm_response(
                    response,
                    None,
                    False,
                    f'Arm timeout must not exceed '
                    f'{ARM_MAX_COMPLETION_TIMEOUT:.1f} seconds.',
                )

        note = ''
        if absolute:
            if abs(x) > 0.22 or abs(z) > 0.15:
                return self._fill_arm_response(
                    response,
                    None,
                    False,
                    'Absolute arm coordinates exceed x +/-0.22 m or '
                    'z +/-0.15 m.',
                )
        else:
            x, z, clamped = self._bounded_relative_arm_target(x, z)
            if clamped:
                note = ' Requested relative motion was clamped to safe bounds.'

        command = 'move_to' if absolute else 'move'
        action, message = self._start_arm_action(
            x=x, z=z, absolute=absolute, command=command
        )
        if action is None:
            return self._fill_arm_response(
                response, None, False, message
            )

        if not request.wait_for_completion:
            return self._fill_arm_response(
                response, action, True, message + note
            )

        finished = action._event.wait(timeout=timeout)
        if not finished:
            return self._fill_arm_response(
                response,
                action,
                False,
                f'Arm action is still running after {timeout:.2f} seconds; '
                'monitor arm_action_status or call cancel_arm.' + note,
            )
        return self._fill_arm_response(
            response,
            action,
            action.has_succeeded,
            (
                'Robotic-arm action completed successfully.'
                if action.has_succeeded else
                f'Robotic-arm action ended with state {action.state}.'
            ) + note,
        )

    def move_arm(self, request, response: MoveArm.Response):
        return self._handle_arm_request(request, response, absolute=False)

    def move_arm_to(self, request, response: MoveArm.Response):
        return self._handle_arm_request(request, response, absolute=True)

    def recenter_arm(self, request, response: Trigger.Response):
        action, message = self._start_arm_action(
            x=0.0, z=0.0, absolute=True, command='recenter'
        )
        response.success = action is not None
        response.message = message
        return response

    def _cancel_arm_action(self, action):
        try:
            success = self._action_dispatcher.cancel_action(action)
        except Exception as exc:
            logger.error(
                f'[{self._conf._name}] Failed to cancel arm action: {exc}'
            )
            return False
        self.publish_arm_action_status()
        return success

    def cancel_arm(self, request, response: Trigger.Response):
        with self._arm_lock:
            action = self.current_arm_action
        if action is None or action.is_completed:
            response.success = False
            response.message = 'There is no active robotic-arm action.'
            return response
        response.success = self._cancel_arm_action(action)
        response.message = (
            f'Robotic-arm action {action._action_id} cancelled.'
            if response.success else
            f'Failed to cancel robotic-arm action {action._action_id}.'
        )
        return response

    def reset_arm(self, request, response: Trigger.Response):
        """Cancel active motion, then recenter; DJI has no separate reset API."""
        with self._arm_lock:
            action = self.current_arm_action
        if action is not None and not action.is_completed:
            if not self._cancel_arm_action(action):
                response.success = False
                response.message = (
                    'Arm reset stopped because the active action could not be '
                    'cancelled.'
                )
                return response
        new_action, message = self._start_arm_action(
            x=0.0, z=0.0, absolute=True, command='reset'
        )
        response.success = new_action is not None
        response.message = message
        return response

    def publish_arm_action_status(self):
        if self.arm_action_pub is None:
            return
        with self._arm_lock:
            action = self.current_arm_action
            msg = ArmActionStatus()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = self._conf._name
            msg.command = self.current_arm_command
            if action is None:
                msg.action_id = 0
                msg.state = 'action_idle'
                msg.percent = 0
                msg.completed = False
                msg.succeeded = False
            else:
                msg.action_id = int(action._action_id)
                msg.state = action.state
                msg.percent = int(action._percent)
                msg.completed = bool(action.is_completed)
                msg.succeeded = bool(action.has_succeeded)
        self.arm_action_pub.publish(msg)

    def move_chassis(self, request:MoveChassis.Request, response:MoveChassis.Response):
        """Move the chassis to a specified position and orientation."""
        x = request.x
        y = request.y
        w = request.w
        xy_speed = request.xy_speed
        w_speed = request.w_speed
        fa = self._move_chassis(x, y, w, xy_speed, w_speed)
        if fa:
            response.success = True
        else:
            response.success = False
        return response

    def _move_chassis(self, x=0., y=0., w=0., xy_speed=0.5, w_speed=30, async_flag=True):
        """
        Controls the chassis to move to the specified position, the origin of the axis is the current position.
        """

        chassis_action = ChassisMoveAction(x, y, w, xy_speed, w_speed)
        if async_flag:
            self._action_dispatcher.send_async_action(chassis_action)
        else:
            self._action_dispatcher.send_action(chassis_action)
        return chassis_action

    # def _clip_chassis_speed_for_limits(self,x xy_speed: float, w_speed: float):
    #     """Prevent commanding further into hard limits; allow motion away from limits."""
    #     x = float(xy_speed)
    #     w = float(w_speed)

    #     xy_min, xy_max = 0.2, 2.0
    #     W_min, w_max = 10.0, 540.0
    #     x = max(xy_min, min(xy_max, x))
    #     w = max(W_min, min(w_max, w))
    #     return x, w

    
    def _move_gimbal(self, pitch=0, yaw=0, pitch_speed=30, yaw_speed=30, async_flag=True):  
        """ Controls the head to move to the specified position, the origin of the axis is the current position.

        :param pitch: float: [-34, 34], pitch axis angle in °.
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
    
    def reset_gimbal(self, request, response:Trigger.Response):
        # Reset the gimbal to its default position
        self._move_gimbal(-self.gimbal_angle_data[0], -self.gimbal_angle_data[1], 120, 120)
        response.success = True
        return response

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

    def _clip_gimbal_speed_for_limits(self, pitch_speed: float, yaw_speed: float):
        """Prevent commanding further into hard limits; allow motion away from limits."""
        p = float(pitch_speed)
        y = float(yaw_speed)

        pmin, pmax = -19.5, 34.5
        ymin, ymax = -249.5, 249.5
        eps = 0.5

        # pitch
        if self.gimbal_angle_data[0] is not None:
            if self.gimbal_angle_data[0] >= (pmax - eps) and p > 0.0:
                logger.warning(
                    f'[{self._conf._name}] Gimbal pitch at upper limit '
                    f'({self.gimbal_angle_data[0]:.2f} deg). Ignoring +pitch command.',
                    throttle_duration_sec=1,
                )
                p = 0.0
            elif self.gimbal_angle_data[0] <= (pmin + eps) and p < 0.0:
                logger.warning(
                    f'[{self._conf._name}] Gimbal pitch at lower limit '
                    f'({self.gimbal_angle_data[0]:.2f} deg). Ignoring -pitch command.',
                    throttle_duration_sec=1,
                )
                p = 0.0
        # yaw
        if self.gimbal_angle_data[1] is not None:
            if self.gimbal_angle_data[1] >= (ymax - eps) and y > 0.0:
                logger.warning(
                    f'[{self._conf._name}] Gimbal yaw at upper limit '
                    f'({self.gimbal_angle_data[1]:.2f} deg). Ignoring +yaw command.',
                    throttle_duration_sec=1,
                )
                y = 0.0
            elif self.gimbal_angle_data[1] <= (ymin + eps) and y < 0.0:
                logger.warning(
                    f'[{self._conf._name}] Gimbal yaw at lower limit '
                    f'({self.gimbal_angle_data[1]:.2f} deg). Ignoring -yaw command.',
                    throttle_duration_sec=1,
                )
                y = 0.0
        return p, y
    
    def gimbal_cmd_vel(self, data:GimbalVel):
        decision = self.motion_safety.authorize(
            'gimbal',
            (data.pitch_speed, data.yaw_speed),
            limits=((-540.0, 540.0), (-540.0, 540.0)),
            source='gimbal_cmd_vel',
            motion_allowed=self._initialized,
        )
        if not decision.accepted:
            logger.warning(
                f'[{self._conf._name}] Rejected gimbal command: {decision.reason}',
                throttle_duration_sec=2,
            )
            return

        pitch_speed, yaw_speed = self._clip_gimbal_speed_for_limits(
            decision.values[0], decision.values[1]
        )

        with self._motion_lock:
            self._send_gimbal_speed(pitch_speed, yaw_speed)

    def _send_gimbal_speed(self, pitch_speed, yaw_speed):
        proto = protocol.ProtoGimbalRotate()
        # Send actual commanded speed (deg/s), clamped by checker.
        # Direction comes from sign of value; magnitude is the speed itself.
        pitch_cmd = float(np.clip(pitch_speed, -540.0, 540.0))
        yaw_cmd = float(np.clip(yaw_speed, -540.0, 540.0))

        proto._pitch = int(pitch_cmd)
        proto._yaw = int(yaw_cmd)
        proto._pitch_speed = int(
            util.GIMBAL_PITCH_MOVE_SPEED_SET_CHECKER.val2proto(abs(pitch_cmd))
        )
        proto._yaw_speed = int(
            util.GIMBAL_YAW_MOVE_SPEED_SET_CHECKER.val2proto(abs(yaw_cmd))
        )
        proto._coordinate = COORDINATE_CUR

        logger.info(f"gimbal_cmd_vel: {proto._pitch=} {proto._yaw=}")
        return self._send_async_proto(proto, protocol.host2byte(4, 0))
        # proto = protocol.ProtoGimbalRotate()
        # proto._pitch = int(np.sign(pitch_speed)*3)
        # proto._yaw = int(np.sign(yaw_speed)*3)
        # proto._pitch_speed = int(
        #     util.GIMBAL_PITCH_MOVE_SPEED_SET_CHECKER.val2proto(
        #         abs(pitch_speed)
        #     )
        # )
        # proto._yaw_speed = int(
        #     util.GIMBAL_YAW_MOVE_SPEED_SET_CHECKER.val2proto(abs(yaw_speed))
        # )
        # proto._coordinate = COORDINATE_CUR
        # logger.info(f"gimbal_cmd_vel: {proto._pitch=} {proto._yaw=}")
        # return self._send_async_proto(proto, protocol.host2byte(4, 0))

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
        max_yaw_rate = 600.0 * np.pi / 180.0
        decision = self.motion_safety.authorize(
            'chassis',
            (data.linear.x, data.linear.y, data.angular.z),
            limits=(
                (-3.5, 3.5),
                (-3.5, 3.5),
                (-max_yaw_rate, max_yaw_rate),
            ),
            source='cmd_vel',
            motion_allowed=self._initialized,
        )
        if not decision.accepted:
            logger.warning(
                f'[{self._conf._name}] Rejected chassis command: '
                f'{decision.reason}',
                throttle_duration_sec=2,
            )
            return
        if decision.clamped:
            logger.warning(
                f'[{self._conf._name}] Chassis command exceeded limits and '
                'was clamped.',
                throttle_duration_sec=2,
            )
        with self._motion_lock:
            self._drive_speed(*decision.values)
    
    def _auto_stop_timer(self, api="drive_speed"):
        if api == "drive_speed":
            logger.debug("Chassis: drive_speed timeout, auto stop!")
            self.motion_safety.mark_safe('chassis')
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
        msg = protocol.Msg(self._client.hostbyte, protocol.host2byte(9, 0), proto)

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
        
        ret = self._set_led(
            cmd=cmd,
            r=request.led.r,
            g=request.led.g,
            b=request.led.b,
            effect=effect,
            freq=freq,
        )
        response.success = ret
        response.message = (
            "LED set successfully" if ret else "LED command failed"
        )
        return response

    def set_gimbal_led(
            self, request: GimbalLED.Request,
            response: GimbalLED.Response):
        """Set selected EP/S1 gimbal armor LEDs."""
        response.success = False
        component_map = {
            COMP_TOP_ALL: COMP_TOP_ALL,
            COMP_TOP_LEFT: COMP_TOP_LEFT,
            COMP_TOP_RIGHT: COMP_TOP_RIGHT,
        }
        component = component_map.get(request.which.casefold())
        if component is None:
            response.message = (
                "Invalid gimbal LED component. Use top_all, top_left, "
                "or top_right."
            )
            return response

        color = (request.led.r, request.led.g, request.led.b)
        if any(not np.isfinite(value) or value < 0 or value > 255
               for value in color):
            response.message = "Gimbal LED RGB values must be in [0, 255]."
            return response

        led_list = list(request.led_list) or [0, 1, 2, 3]
        if any(index > 7 for index in led_list):
            response.message = "Gimbal LED indices must be in [0, 7]."
            return response
        if len(set(led_list)) != len(led_list):
            response.message = "Gimbal LED indices must not be repeated."
            return response
        if self.led_control is None:
            response.message = "Gimbal LED control is unavailable."
            return response

        success = self.led_control.set_gimbal_led(
            comp=component,
            r=color[0],
            g=color[1],
            b=color[2],
            led_list=led_list,
            effect=EFFECT_ON if request.on else EFFECT_OFF,
        )
        response.success = bool(success)
        response.message = (
            "Gimbal LEDs set successfully."
            if response.success else "Gimbal LED command failed."
        )
        return response

    def _set_blaster_led(self, on=True, brightness=255):
        """Send the DJI no-ack blaster LED command."""
        proto = protocol.ProtoBlasterSetLed()
        value = int(brightness)
        proto._r = value
        proto._g = value
        proto._b = value
        proto._effect = 1 if on else 0
        msg = protocol.Msg(
            self._client.hostbyte, protocol.host2byte(23, 0), proto
        )
        try:
            self._client.send_sync_msg(msg)
            return True
        except Exception as exc:
            logger.warning(
                f'[{self._conf._name}] Blaster LED send failed: {exc}'
            )
            return False

    def set_blaster_led(
            self, request: BlasterLED.Request,
            response: BlasterLED.Response):
        """Set the EP/S1 blaster LED brightness or turn it off."""
        response.success = bool(self._set_blaster_led(
            on=request.on,
            brightness=request.brightness,
        ))
        response.message = (
            "Blaster LED set successfully."
            if response.success else "Blaster LED command failed."
        )
        return response

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
                        :z: chassis yaw relative to the origin, in degrees

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
        self.position_data = [data[0], data[1], data[2]]
        self.pub_position()

    def sub_position(self, freq=10):
        return self._sub_position(
            freq=freq, callback=self.get_position_callback
        )

    def _sub_chassis_attitude(self, freq=10, callback=None, *args, **kw):
        subject = ChassisAttitudeSubject()
        subject.freq = freq
        return self.dds.add_subject_info(subject, callback, args, kw)

    def get_chassis_attitude_callback(self, data):
        yaw, pitch, roll = data
        if self.chassis_yaw_origin is None:
            self.chassis_yaw_origin = yaw
        relative_yaw = (
            (yaw - self.chassis_yaw_origin + 180.0) % 360.0 - 180.0
        )
        self.chassis_attitude_data = [roll, pitch, relative_yaw]
        if self.position_data is not None:
            self.pub_position()

    def sub_chassis_attitude(self, freq=10):
        return self._sub_chassis_attitude(
            freq=freq, callback=self.get_chassis_attitude_callback
        )

    def pub_position(self):
        if (
            self.position_data
            and self.chassis_attitude_data
            and self.position_pub
        ):
            roll, pitch, yaw = self.chassis_attitude_data
            quaternion = quaternion_from_euler(
                radians(roll), radians(pitch), radians(yaw)
            )
            self.position_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.position_msg.pose.position.x = self.position_data[0]
            self.position_msg.pose.position.y = self.position_data[1]
            self.position_msg.pose.position.z = self.position_data[2]
            self.position_msg.pose.orientation.x = quaternion[0]
            self.position_msg.pose.orientation.y = quaternion[1]
            self.position_msg.pose.orientation.z = quaternion[2]
            self.position_msg.pose.orientation.w = quaternion[3]
            self.position_pub.publish(self.position_msg)

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
        if self.gimbal_angle_pub:
            self.pub_gimbal_angle()
    
    def sub_gimbal_angle(self, freq=10):
        return self._sub_gimbal_angle(
            freq=freq, callback=self.get_gimbal_angle_callback
        )

    def pub_gimbal_angle(self):
        if self.gimbal_angle_data and self.gimbal_angle_pub:
            self.gimbal_angle_msg.pitch_angle = self.gimbal_angle_data[0]
            self.gimbal_angle_msg.yaw_angle = self.gimbal_angle_data[1]
            self.gimbal_angle_msg.pitch_ground_angle = self.gimbal_angle_data[2]
            self.gimbal_angle_msg.yaw_ground_angle = self.gimbal_angle_data[3]
            self.gimbal_angle_pub.publish(self.gimbal_angle_msg)

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
        if effect == EFFECT_OFF:
            proto._effect_mode = 0
        elif effect == EFFECT_ON:
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
    
    def set_gimbal_led(self, comp=COMP_TOP_ALL, r=255, g=255, b=255,
                       led_list=None, effect=EFFECT_ON):
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
        led_list = [0, 1, 2, 3] if led_list is None else led_list
        for led_index in led_list:
            proto._led_mask |= 1 << (led_index % 8)
        proto._r = int(util.COLOR_VALUE_CHECKER.val2proto(r))
        proto._g = int(util.COLOR_VALUE_CHECKER.val2proto(g))
        proto._b = int(util.COLOR_VALUE_CHECKER.val2proto(b))
        if effect == EFFECT_OFF:
            proto._effect_mode = 0
        elif effect == EFFECT_ON:
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
