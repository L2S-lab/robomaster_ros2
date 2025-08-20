try:
    from .globals import *
except ImportError as e:
    from globals import *
except Exception as e:
    raise e

class Config:
    _name = "Unknown"
    _product = "Unknown"
    _cmd_addr = None
    _cmd_proto = "v1"
    _sdk_addr = None
    _video_stream_addr = None
    _video_stream_port = None
    _video_stream_proto = "tcp"
    _audio_stream_addr = None
    _audio_stream_port = None
    _audio_stream_proto = "tcp"
    _number= None

    def __init__(self, name: str):
        self._name = name

    @property
    def default_cmd_addr_port(self):
        return self.default_cmd_addr[1] 

    @property
    def product(self):
        return self._product

    @product.setter
    def product(self, val):
        self._product = val

    @property
    def number(self):
        return self._number

    @number.setter
    def number(self, val):
        self._number = val


    @property
    def default_robot_addr(self):
        return self._cmd_addr

    @default_robot_addr.setter
    def default_robot_addr(self, val):
        self._cmd_addr = val

    @property
    def cmd_proto(self):
        return self._cmd_proto

    @cmd_proto.setter
    def cmd_proto(self, val):
        self._cmd_proto = val

    @property
    def default_sdk_addr(self):
        return self._sdk_addr

    @default_sdk_addr.setter
    def default_sdk_addr(self, val):
        self._sdk_addr = val

    @property
    def video_stream_addr(self):
        return self._video_stream_addr

    @video_stream_addr.setter
    def video_stream_addr(self, val):
        self._video_stream_addr = val

    @property
    def video_stream_port(self):
        return self._video_stream_port

    @video_stream_port.setter
    def video_stream_port(self, val):
        self._video_stream_port = val

    @property
    def video_stream_proto(self):
        return self._video_stream_proto

    @video_stream_proto.setter
    def video_stream_proto(self, val):
        self._video_stream_proto = val

    @property
    def audio_stream_addr(self):
        return self._audio_stream_addr

    @audio_stream_addr.setter
    def audio_stream_addr(self, val):
        self._audio_stream_addr = val

    @property
    def audio_stream_port(self):
        return self._audio_stream_port

    @audio_stream_port.setter
    def audio_stream_port(self, val):
        self._audio_stream_port = val

    @property
    def audio_stream_proto(self):
        return self._audio_stream_proto

    @audio_stream_proto.setter
    def audio_stream_proto(self, val):
        self._audio_stream_proto = val


te_conf = Config("TelloEduConfig")
te_conf.product = "TelloEdu"
te_conf.default_robot_addr = ('192.168.10.1', 8889)
te_conf.cmd_proto = "text"
te_conf.default_sdk_addr = ('192.168.10.2', 8890)
te_conf.video_stream_addr = ('192.168.10.2', 11111)
te_conf.video_stream_proto = "udp"

ep_conf = Config("RoboMasterEPConfig")
ep_conf.product = "RoboMasterEP"
ep_conf.cmd_proto = "v1"
ep_conf.default_robot_addr = ('192.168.0.1', 40923)
ep_conf.default_sdk_addr = ('192.168.0.2', 40924)
ep_conf.video_stream_addr = ()
ep_conf.video_stream_proto = "tcp"
ep_conf.video_stream_port = 40921
ep_conf.audio_stream_addr = ()
ep_conf.audio_stream_proto = "tcp" 
ep_conf.audio_stream_port = 40922


