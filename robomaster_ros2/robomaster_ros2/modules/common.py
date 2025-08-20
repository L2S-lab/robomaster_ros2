from rclpy.logging import get_logger
_logger = get_logger('common')

import collections
import socket
from netaddr import IPNetwork
import time
from threading import Thread

try:
    from .globals import *
    from . import protocol, config
    from .util import get_subnets
except ImportError as e:
    from globals import *
    import protocol, config
    from util import get_subnets
except Exception as e:
    _logger.error(f"Import Error pass two: {e}")

class Handler(collections.namedtuple("Handler", ("obj name f"))):
    __slots__ = ()

class Dispatcher(object):

    def __init__(self):
        self._dispatcher_handlers = collections.defaultdict(list)

    def add_handler(self, obj, name, f):
        handler = Handler(obj, name, f)
        self._dispatcher_handlers[name] = handler
        return handler

    def remove_handler(self, name):
        del self._dispatcher_handlers[name]

    def dispatch(self, msg, **kw):
        for name in self._dispatcher_handlers:
            handler = self._dispatcher_handlers[name]
            handler.f(handler.obj, msg)

class BaseConnection(object):
    def __init__(self): 
        self._sock = None
        self._buf = bytearray()
        self._host_addr = None
        self._target_addr = None
        self._proto_type = None
        self._proto = None

    def close(self):
        if self._sock:
            self._sock.close()

    def recv(self):
        try:
            if self._sock:
                data, host = self._sock.recvfrom(2048)
        except Exception as e:
            _logger.warning("[Connection] recv, exception:{0}".format(e))
            raise
        if data is None:
            _logger.warning("[Connection] recv buff None.")
            return None
        self._buf.extend(data) 
        if len(self._buf) == 0:
            _logger.warning("[Connection] recv buff None.")
            return None

        if self._proto is None:
            self._proto = "v1"
        msg, self._buf = protocol.decode_msg(self._buf, self._proto) 
        if msg is False or msg is None:
            _logger.warning("[Connection] protocol.decode_msg is None.")
            return None
        else:
            if isinstance(msg, protocol.MsgBase):
                if not msg.unpack_protocol():
                    _logger.debug("[Connection] recv, msg.unpack_protocol failed, msg:{0}".format(msg))
                    return None
            return msg

    def send(self, buf):
        try:
            if self._sock:
                self._sock.sendto(buf, self._target_addr) 
        except Exception as e:
            _logger.warning("Connection: send, exception:{0}".format(e))
            raise

    def send_self(self, buf):
        try:
            if self._sock:
                self._sock.sendto(buf, self._host_addr) 
        except Exception as e:
            _logger.warning("Connection: send_self, exception:{0}".format(e))
            raise

class Connection(BaseConnection):
    def __init__(self, conf:config.Config=None):
        self._conf = conf
        self._host_addr = None
        if conf:
            self._host_addr = self._conf.default_sdk_addr # (local_ip,port)
            self._target_addr = self._conf.default_robot_addr
            self._proto = self._conf.cmd_proto
            if self._host_addr is None or self._target_addr is None:
                _logger.error(f"[Connection] [{self._conf._name}] __init__, host_addr or target_addr is None")

        self._sock = None
        self._buf = bytearray()
        self._robot_host_list = []    # for scan robot
        self.client_recieve_thread_flag = False

    def __repr__(self):
        return f"[Connection] [{self._conf._name}] host:{self._host_addr}, target:{self._target_addr}"

    def _scan_host(self, nb_drones:int, timeout=None):
        _logger.info(f"[Connection] Searching for {nb_drones} available drones...")

        subnets, address = get_subnets()
        possible_addr = []

        for subnet, netmask in subnets:
            for ip in IPNetwork('%s/%s' % (subnet, netmask)):
                # skip local and broadcast
                if str(ip).split('.')[3] == '0' or str(ip).split('.')[3] == '255':
                    continue
                possible_addr.append(str(ip))
        start = time.time()
        while len(self._robot_host_list) < nb_drones:
            if timeout!= None and time.time()-start > timeout:
                break
            if len(self._robot_host_list) >= nb_drones:
                break
            _logger.info(f"[Connection] still searching for drones in subnets...")

            for tello_host in self._robot_host_list:
                if tello_host[0] in possible_addr:
                    possible_addr.remove(tello_host[0])
            # skip server itself
            for ip in possible_addr:
                if ip in address:
                    continue
                self._sock.sendto(b'command', (ip, TELLO_DEVICE_PORT))
            
        return self._robot_host_list

    def scan_multi_drones(self, nb_drones:int, timeout=None, local_ip=None):
        self._host_addr = (local_ip, TELLO_SDK_PORT)
        self.create()
        receive_thread = Thread(target=self._scan_receive_task, args=(nb_drones, ), daemon=True)
        receive_thread.start()
        robot_host_list = self._scan_host(nb_drones, timeout)
        receive_thread.join()
        return robot_host_list

    def _scan_receive_task(self, num):
        ip_list = []
        while len(self._robot_host_list) < num:
            try:
                resp, ip = self._sock.recvfrom(1024)
                _logger.debug(f"[Connection] FoundTello: from ip {ip} _receive_task, recv msg: {resp}".format(resp, ip))
                ip = ''.join(str(ip[0]))
                if resp.upper() == b'OK' and ip not in self._robot_host_list and ip not in ip_list:
                    _logger.info(f"[Connection] Found Tello at {ip}")
                    self._robot_host_list.append((ip, TELLO_DEVICE_PORT))
                    ip_list.append(ip)
            except socket.error as exc:
                _logger.error(f"[Connection] Caught exception socket.error : {exc}")
        self.client_recieve_thread_flag = True
        _logger.info(f"[Connection] Scan multi-drone complete.")
    
    def create(self):
       if self._host_addr is None and self._conf is None:
           _logger.error(f"[Connection] create, {self._host_addr=} unexpected connection param set")
           return
       try:
           if self._sock is None or self._sock._closed:
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._sock.bind(self._host_addr)
           else:
               _logger.error(f"[Connection] {self._host_addr=} unexpected connection param set")
       except Exception as e:
           _logger.warning(f"[Coneection] create, {self._host_addr=}, exception:{e}")
           raise
           