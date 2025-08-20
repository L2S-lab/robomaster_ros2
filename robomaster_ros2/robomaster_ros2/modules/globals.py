# Global variables and constants

from enum import Enum

def host2byte(host, index):
    return index * 32 + host

def byte2host(b):
    return (b & 0x1f), (b >> 5)

def make_proto_cls_key(cmdset, cmdid):
    return cmdset * 256 + cmdid

DEFAULT_CONN_TYPE = "sta"
LOCAL_IP_STR = None
ROBOT_IP_STR = None
DEFAULT_PROTO_TYPE = "udp" 
TOTAL_ROBOT_NUM = None
DDS_SUB_TYPE_EVENT = 1
DDS_SUB_TYPE_PERIOD = 0
ROBOT_SN_LEN = 14

UNIT_METRIC = 'Unit Metric'
UNIT_INCH = 'Unit Inch'


#############################################################
# ROBOT
#############################################################
SDK_FIRST_DDS_ID = 20
SDK_LAST_DDS_ID = 255
RM_SDK_FIRST_ACTION_ID = 1
RM_SDK_LAST_ACTION_ID = 255

ROBOT_SDK_PORT_MIN = 10100
ROBOT_SDK_PORT_MAX = 10500
ROBOT_DEVICE_PORT = 20020
ROBOT_PROXY_PORT = 30030
ROBOT_BROADCAST_PORT = 40927
ROBOT_VIDEO_PORT = 40921
ROBOT_AUDIO_PORT = 40922
ROBOT_VIDEO_PROTO = "udp" 
ROBOT_AUDIO_PROTO = "udp" 

# Default ID range
RM_SDK_FIRST_SEQ_ID = 10000
RM_SDK_LAST_SEQ_ID = 20000

# Protocol ACK Type
DUSS_MB_ACK_NO = 0
DUSS_MB_ACK_NOW = 1
DUSS_MB_ACK_FINISH = 2

# Protocol encryption type
DUSS_MB_ENC_NO = 0
DUSS_MB_ENC_AES128 = 1
DUSS_MB_ENC_CUSTOM = 2

# Protocol type
DUSS_MB_TYPE_REQ = 0
DUSS_MB_TYPE_PUSH = 1

CLIENT_MAX_EVENT_NUM = 16

CONNECTION_PROTO_TCP = 'tcp'
CONNECTION_PROTO_UDP = 'udp'
FREE = "free"
GIMBAL_LEAD = "gimbal_lead"
CHASSIS_LEAD = "chassis_lead"

STREAM_360P = "360p"
STREAM_540P = "540p"
STREAM_720P = "720p"

COORDINATE_NED = 0
COORDINATE_CUR = 1
COORDINATE_CAR = 2
COORDINATE_PNED = 3     # pitch NED mode
COORDINATE_YCPN = 4     # yaw CAR, pitch NED mode
COORDINATE_YCPO = 5     # yaw CAR, pitch OFFSET mode

ARMOR_BOTTOM_BACK = 0x1
ARMOR_BOTTOM_FRONT = 0x2
ARMOR_BOTTOM_LEFT = 0x4
ARMOR_BOTTOM_RIGHT = 0x8
ARMOR_TOP_LEFT = 0x10
ARMOR_TOP_RIGHT = 0x20
ARMOR_TOP_ALL = 0x30
ARMOR_BOTTOM_ALL = 0xf
ARMOR_ALL = 0x3f

COMP_TOP_LEFT = 'top_left'
COMP_TOP_RIGHT = 'top_right'
COMP_BOTTOM_LEFT = 'bottom_left'
COMP_BOTTOM_RIGHT = 'bottom_right'
COMP_BOTTOM_FRONT = 'bottom_front'
COMP_BOTTOM_BACK = 'bottom_back'
COMP_BOTTOM_ALL = 'bottom_all'
COMP_TOP_ALL = 'top_all'
COMP_ALL = 'all'

EFFECT_ON = 'on'
EFFECT_OFF = 'off'
EFFECT_PULSE = 'pulse'
EFFECT_FLASH = 'flash'
EFFECT_BREATH = 'breath'
EFFECT_SCROLLING = 'scrolling'

DDS_BATTERY = "battery"
DDS_GIMBAL_BASE = "gimbal_base"
DDS_VELOCITY = "velocity"
DDS_ESC = "esc"
DDS_ATTITUDE = "attitude"
DDS_IMU = "imu"
DDS_POSITION = "position"
DDS_SA_STATUS = "sa_status"
DDS_CHASSIS_MODE = "chassis_mode"
DDS_SBUS = "sbus"
DDS_SERVO = "servo"
DDS_ARM = "arm"
DDS_GRIPPER = "gripper"
DDS_GIMBAL_POS = "gimbal_pos"
DDS_STICK = "stick"
DDS_MOVE_MODE = "move_mode"
DDS_TOF = "tof"
DDS_PINBOARD = "pinboard"
DDS_ALL = "all"

ROBOT_DEFAULT_HOST = host2byte(9, 6)

#############################################################
# DRONE
#############################################################
CONNECTION_PROTO_TCP = 'tcp'
CONNECTION_PROTO_UDP = 'udp'
DDS_TELLO_ATTITUDE = "tello_attitude"
DDS_TELLO_AI = "tello_ai"
DDS_TELLO_IMU = "tello_imu"
DDS_TELLO_TOF = "tello_tof"
DDS_TELLO_DRONE = "tello_drone"
DDS_TELLO_BATTERY = "tello_battery"
DDS_TELLO_TEMP = "tello_temperature"
DDS_TELLO_ALL = "tello_all"
IS_AI_FLAG = ";degree:"
TELLO_DDS_TIME_MAX = 666
TELLO_DEVICE_PORT = 8889
TELLO_SDK_PORT_MIN = 8890
TELLO_SDK_PORT_MAX = 9390
TELLO_SDK_PORT = 8890
TELLO_VIDEO_PROTO = "udp"
TELLO_VIDEO_PORT_MIN = 11111
TELLO_VIDEO_PORT_MAX = 11611
TELLO_VIDEO_PORT = 11111
NUMBER_OF_DRONES = 1

class State(Enum):
    Idle = 0
    TakingOff = 1
    Landing = 2
    Hover = 3
    Automatic = 4
    Custom = 5
    Emergency = 6
