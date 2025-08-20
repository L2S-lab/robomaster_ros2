from socket import AF_INET
from netifaces import AF_INET, interfaces, ifaddresses
from netaddr import IPNetwork
import socket, time

from rclpy.logging import get_logger
from rclpy import spin_until_future_complete
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
logger = get_logger('util')

try:
    from .globals import *
except ImportError as e:
    from globals import *
except Exception as e:
    logger.error(f"Import Error pass two: {e}")

# node: current node handle
# node_name: name of node for which you need to get params
# param_list: list of params that need to be get ['param_1','param_2']
def call_get_parameters(node:Node, node_name:str, namespace:str, parameter_names:list)->dict:
    param_dict = {}
    # create client
    client = node.create_client(GetParameters, f'{node_name}/get_parameters')

    # call as soon as ready
    ready = client.wait_for_service(timeout_sec=5.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')

    request = GetParameters.Request()
    request.names = [namespace+'.'+param for param in parameter_names]
    future = client.call_async(request)
    spin_until_future_complete(node, future)

    # handle response
    response = future.result()
    if response is None:
        e = future.exception()
        raise RuntimeError(
            f"Exception while calling service of node '{node_name}': {e}")
    for i in range(len(response.values)):#request.values[:]:
        if response.values[i].type==1:
            param_dict[parameter_names[i]]=response.values[i].bool_value
        elif response.values[i].type==2:
            param_dict[parameter_names[i]]=response.values[i].integer_value
        elif response.values[i].type==3:
            param_dict[parameter_names[i]]=response.values[i].double_value
        elif response.values[i].type==4:
            param_dict[parameter_names[i]]=response.values[i].string_value
        elif response.values[i].type==5:
            param_dict[parameter_names[i]]=response.values[i].byte_array_value
        elif response.values[i].type==6:
            param_dict[parameter_names[i]]=response.values[i].bool_array_value
        elif response.values[i].type==7:
            param_dict[parameter_names[i]]=response.values[i].integer_array_value
        elif response.values[i].type==8:
            param_dict[parameter_names[i]]=response.values[i].double_array_value
        elif response.values[i].type==9:
            param_dict[parameter_names[i]]=response.values[i].string_array_value
        else:
            node.get_logger().error(f"Type {response.values[i].type} is not supported!")
            raise TypeError(f"Type {response.values[i].type} is not supported!")

    return param_dict

def enable_sdk_drone(drone_ip, timeout=2, retry=5):
    cmd = 'command'
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(cmd.encode('utf-8'), (drone_ip, TELLO_DEVICE_PORT))
    sock.settimeout(1)
    while retry > 0:
        retry -= 1
        time.sleep(0.2)
        try:
            resp, ip = sock.recvfrom(1024)  
            if resp.upper() == b'OK':
                sock.close()
            return True
        except socket.timeout:
            pass
        sock.close()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(cmd.encode('utf-8'), (drone_ip, TELLO_DEVICE_PORT))
    sock.close()
    return False

def get_subnets():
    """
    Look through the machine's internet connection and
    returns subnet addresses and server ip
    :return: list[str]: subnets
                list[str]: addr_list
    """
    subnets = []
    ifaces = interfaces()
    addr_list = []
    for myiface in ifaces:
        addrs = ifaddresses(myiface)

        if AF_INET not in addrs:
            continue
        # Get ipv4 stuff
        ipinfo = addrs[AF_INET][0]
        address = ipinfo['addr']
        netmask = ipinfo['netmask']

        # limit range of search. This will work for router subnets
        if netmask != '255.255.255.0':
            continue

        cidr = IPNetwork('%s/%s' % (address, netmask))
        network = cidr.network
        subnets.append((network, netmask))
        addr_list.append(address)
    return subnets, addr_list


class UnitChecker(object):
    # Unit
    def __init__(self, name, default=0, start=0.0, end=0.0, step=1.0, decimal=2, scale=1.0, unit=UNIT_METRIC):
        self._name = name
        self._start = start
        self._end = end
        self._step = step
        self._decimal = decimal
        self._scale = scale
        self._unit = unit
        self._default = default
        # if decimal = 0, set None to round() to get a integer
        if self._decimal == 0:
            self._decimal = None

    @property
    def name(self):
        return self._name

    @property
    def default(self):
        return self._default

    @property
    def scale(self):
        return self._scale

    @property
    def step(self):
        return self._step

    @property
    def decimal(self):
        return self._decimal

    @property
    def start(self):
        return self._start

    @property
    def end(self):
        return self._end

    @property
    def unit(self):
        return self._unit

    def check(self, value):
        if self._start and self._end:
            if value > self._end:
                value = self._end
                logger.warning("{0}: over limit and is set to {1}".format(self._name, self._end))
            if value < self._start:
                value = self._start
                logger.warning("{0}: below limit and is set to {1}".format(self._name, self._start))
        return value

    def proto2val(self, val):
        val = val / self.scale
        val = round(val, self._decimal)
        val = self.check(val)
        return val

    def val2proto(self, val):
        val = self.check(val)
        val = val * self._scale
        val = round(val, self._decimal)
        return val


# Gimbal Target Angle
GIMBAL_PITCH_TARGET_CHECKER = UnitChecker("gimbal pitch target", default=0, start=-20.0, end=35.0, step=1, decimal=0,
                                          scale=10)
GIMBAL_YAW_TARGET_CHECKER = UnitChecker("gimbal yaw target", default=0, start=-250, end=250, step=1, decimal=0,
                                        scale=10)
# Gimbal Rotation Angle
GIMBAL_PITCH_MOVE_CHECKER = UnitChecker("gimbal pitch move", default=0, start=-55.0, end=55.0, step=1, decimal=0,
                                        scale=10)
GIMBAL_YAW_MOVE_CHECKER = UnitChecker("gimbal yaw move", default=0, start=-500, end=500, step=1, decimal=0, scale=10)

# Velocity parameters during gimbal position control mode
GIMBAL_PITCH_MOVE_SPEED_SET_CHECKER = UnitChecker("gimbal pitch move speed set", default=30, start=0, end=540, step=1,
                                                  decimal=0, scale=1)
GIMBAL_YAW_MOVE_SPEED_SET_CHECKER = UnitChecker("gimbal yaw move speed set", default=30, start=0, end=540, step=1,
                                                decimal=0, scale=1)
# Speed parameters during gimbal speed control mode
GIMBAL_PITCH_SPEED_SET_CHECKER = UnitChecker("gimbal pitch speed set", default=30, start=-540, end=540, step=1,
                                             decimal=0, scale=10)
GIMBAL_YAW_SPEED_SET_CHECKER = UnitChecker("gimbal yaw speed set", default=30, start=-540, end=540, step=1, decimal=0,
                                           scale=10)

GIMBAL_ATTI_PITCH_CHECKER = UnitChecker("gimbal atti pitch", default=0, start=None, end=None, step=1, decimal=2, 
                                        scale=10)
GIMBAL_ATTI_YAW_CHECKER = UnitChecker("gimbal atti yaw", default=0, start=None, end=None, step=1, decimal=2, scale=10) 

# Chassis value check
CHASSIS_POS_X_SET_CHECKER = UnitChecker('chassis pos x set', default=0, start=-5.0, end=5.0, step=0.01, decimal=0,
                                        scale=100)
CHASSIS_POS_Y_SET_CHECKER = UnitChecker('chassis pos y set', default=0, start=-5.0, end=5.0, step=0.01, decimal=0,
                                        scale=100)
CHASSIS_POS_Z_SET_CHECKER = UnitChecker('chassis pos z set', default=0, start=-1800, end=1800, step=0.1, decimal=0,
                                        scale=10)

CHASSIS_POS_X_SUB_CHECKER = UnitChecker('chassis pos x sub', default=0, start=None, end=None, step=0.01, decimal=5) 
CHASSIS_POS_Y_SUB_CHECKER = UnitChecker('chassis pos y sub', default=0, start=None, end=None, step=0.01, decimal=5) 
CHASSIS_POS_Z_SUB_CHECKER = UnitChecker('chassis pos z sub', default=0, start=None, end=None, step=0.1, decimal=2, 
                                        scale=10)

CHASSIS_PITCH_CHECKER = UnitChecker('chassis pitch', default=0, start=-180, end=180, step=0.1, decimal=2, scale=1)
CHASSIS_YAW_CHECKER = UnitChecker('chassis yaw', default=0, start=-180, end=180, step=0.1, decimal=2, scale=1)
CHASSIS_ROLL_CHECKER = UnitChecker('chassis roll', default=0, start=-180, end=180, step=0.1, decimal=2, scale=1)

CHASSIS_ACC_CHECKER = UnitChecker('chassis acc', default=0, start=None, end=None, step=None, decimal=5) 
CHASSIS_GYRO_CHECKER = UnitChecker('chassis gyro', default=0, start=None, end=None, step=None, decimal=5) 

CHASSIS_SPD_X_CHECKER = UnitChecker('chassis spd x', default=0, start=-3.5, end=3.5, step=0.01, decimal=2)
CHASSIS_SPD_Y_CHECKER = UnitChecker('chassis spd y', default=0, start=-3.5, end=3.5, step=0.01, decimal=2)
CHASSIS_SPD_Z_CHECKER = UnitChecker('chassis spd z', default=0, start=-600, end=600, step=1, decimal=0)

WHEEL_SPD_CHECKER = UnitChecker('wheel speed', default=0, start=-1000, end=1000, step=1, decimal=0)

PWM_VALUE_CHECKER = UnitChecker('pwm value', default=0, start=0, end=50000, step=1, decimal=0)
PWM_FREQ_CHECKER = UnitChecker('pwm freq', default=1000, start=0, end=100, step=1, decimal=0, scale=10)

# Numerical checking of robotic arms and grippers
ROBOTIC_ARM_POS_CHECK = UnitChecker('robotic arm pos', default=0, start=None, end=None, step=1, decimal=0) 
GRIPPER_POWER_CHECK = UnitChecker('gripper power', default=50, start=1, end=100, step=1, decimal=0, scale=6.6)

COLOR_VALUE_CHECKER = UnitChecker('color rgb', default=0, start=0, end=255, step=1, decimal=0)
FIRE_TIMES_CHECKER = UnitChecker('fire times', default=1, start=1, end=5, step=1, decimal=0)
