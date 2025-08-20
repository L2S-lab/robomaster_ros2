import collections
import threading
from queue import Queue
from abc import abstractmethod
from concurrent.futures import ThreadPoolExecutor

from rclpy.logging import get_logger
logger = get_logger('dds')

try:
    from . import protocol
    from .globals import *
except ImportError as e:
    import protocol
    from globals import *
except Exception as e:
    logger.error(f"Import Error pass two: {e}")

SUB_UID_MAP = {
    DDS_BATTERY: 0x000200096862229f,
    DDS_GIMBAL_BASE: 0x00020009f5882874,
    DDS_VELOCITY: 0x0002000949a4009c,
    DDS_ESC: 0x00020009c14cb7c5,
    DDS_ATTITUDE: 0x000200096b986306,
    DDS_IMU: 0x00020009a7985b8d,
    DDS_POSITION: 0x00020009eeb7cece,
    DDS_SA_STATUS: 0x000200094a2c6d55,
    DDS_CHASSIS_MODE: 0x000200094fcb1146,
    DDS_SBUS: 0x0002000988223568,
    DDS_SERVO: 0x000200095f0059e7,
    DDS_ARM: 0x0002000926abd64d,
    DDS_GRIPPER: 0x00020009124d156a,
    DDS_GIMBAL_POS: 0x00020009f79b3c97,
    DDS_STICK: 0x0002000955e9a0fa,
    DDS_MOVE_MODE: 0x00020009784c7bfd,
    DDS_TOF: 0x0002000986e4c05a,
    DDS_PINBOARD: 0x00020009eebb9ffc,
}

registered_subjects = {}
dds_cmd_filter = {(0x48, 0x08)}


class _AutoRegisterSubject(type):
    '''hepler to automatically register Proto Class whereever they're defined '''

    def __new__(mcs, name, bases, attrs, **kw):
        return super().__new__(mcs, name, bases, attrs, **kw)

    def __init__(cls, name, bases, attrs, **kw):
        super().__init__(name, bases, attrs, **kw)
        if name == 'Subject':
            return
        key = name
        if key in registered_subjects.keys():
            raise ValueError("Duplicate Subject class {0}".format(name))
        registered_subjects[key] = cls


class Subject(metaclass=_AutoRegisterSubject):
    name = "Subject"
    _push_proto_cls = protocol.ProtoPushPeriodMsg
    type = DDS_SUB_TYPE_PERIOD
    uid = 0
    freq = 1

    def __init__(self):
        self._task = None
        self._subject_id = 1
        self._callback = None
        self._cb_args = None
        self._cb_kw = None

    def __repr__(self):
        return "dds subject, name:{0}".format(self.name)

    def set_callback(self, callback, args, kw):
        self._callback = callback
        self._cb_args = args
        self._cb_kw = kw

    @abstractmethod
    def data_info(self):
        return None

    def exec(self):
        self._callback(self.data_info(), *self._cb_args, **self._cb_kw) 


class SubHandler(collections.namedtuple("SubHandler", ("obj subject f"))):
    __slots__ = ()

class Subscriber():
    _host = host2byte(9, 0)
    _sub_msg_id = SDK_FIRST_DDS_ID

    def __init__(self, robot):
        self._robot = robot
        self._client = robot._client

        self.msg_sub_dict = {}
        self._publisher = collections.defaultdict(list)
        self._msg_queue = Queue()
        self._dispatcher_running = False
        self._dispatcher_thread = None
        self.excutor = ThreadPoolExecutor(max_workers=15)

    def __del__(self):
        self.stop()

    def get_next_subject_id(self):
        if self._sub_msg_id > SDK_LAST_DDS_ID:
            self._sub_msg_id = SDK_FIRST_DDS_ID
        else:
            self._sub_msg_id += 1
        return self._sub_msg_id

    def start(self):
        self._dds_mutex = threading.Lock()
        self._client.add_handler(self, "Subscriber", self._msg_recv) 
        self._dispatcher_thread = threading.Thread(target=self._dispatch_task)
        self._dispatcher_thread.start()

    def stop(self):
        self._dispatcher_running = False
        if self._dispatcher_thread:
            self._msg_queue.put(None)
            self._dispatcher_thread.join()
            self._dispatcher_thread = None
        self.excutor.shutdown(wait=False)

    @classmethod
    def _msg_recv(cls, self, msg):
        for cmd_set, cmd_id in list(dds_cmd_filter):
            if msg.cmdset == cmd_set and msg.cmdid == cmd_id:
                self._msg_queue.put(msg)

    def _dispatch_task(self):
        self._dispatcher_running = True
        logger.debug("Subscriber: dispatcher_task is running...")
        while self._dispatcher_running:
            msg = self._msg_queue.get(1) 
            if msg is None:
                if not self._dispatcher_running:
                    break
                continue
            self._dds_mutex.acquire()
            for name in self._publisher:
                handler = self._publisher[name]
                logger.debug("Subscriber: msg: {0}".format(msg))
                proto = msg.get_proto()
                if proto is None:
                    logger.warning("Subscriber: _publish, msg.get_proto None, msg:{0}".format(msg))
                    continue
                if handler.subject.type == DDS_SUB_TYPE_PERIOD and msg.cmdset == 0x48 and msg.cmdid == 0x08: 
                    logger.debug("Subscriber: _publish: msg_id:{0}, subject_id:{1}".format(proto._msg_id,
                                                                                           handler.subject._subject_id)) 
                    if proto._msg_id == handler.subject._subject_id: 
                        handler.subject.decode(proto._data_buf) 
                        if handler.subject._task is None: 
                            handler.subject._task = self.excutor.submit(handler.subject.exec) 
                        if handler.subject._task.done() is True: 
                            handler.subject._task = self.excutor.submit(handler.subject.exec) 
                elif handler.subject.type == DDS_SUB_TYPE_EVENT: 
                    if handler.subject.cmdset == msg.cmdset and handler.subject.cmdid == msg.cmdid: 
                        handler.subject.decode(proto._data_buf) 
                        if handler.subject._task is None: 
                            handler.subject._task = self.excutor.submit(handler.subject.exec) 
                        if handler.subject._task.done() is True: 
                            handler.subject._task = self.excutor.submit(handler.subject.exec) 
            self._dds_mutex.release()
            logger.debug("Subscriber: _publish, msg is {0}".format(msg))

    def add_cmd_filter(self, cmd_set, cmd_id):
        dds_cmd_filter.add((cmd_set, cmd_id))

    def del_cmd_filter(self, cmd_set, cmd_id):
        dds_cmd_filter.remove((cmd_set, cmd_id))

    def add_subject_event_info(self, subject, callback=None, *args):
        """ Adding an Event Subscription

        :param subject: The event subscribes to the corresponding subject
        :param callback: The event subscription parser function
        """
        subject.set_callback(callback, args[0], args[1])
        handler = SubHandler(self, subject, callback)
        subject._task = None
        self._dds_mutex.acquire()
        self._publisher[subject.name] = handler 
        self._dds_mutex.release()
        self.add_cmd_filter(subject.cmdset, subject.cmdid)
        return True

    def del_subject_event_info(self, subject):
        """ Deleting event subscriptions

        :param subject: The event subscribes to the corresponding subject
        :param callback: The event subscription parser function
        :return: bool: call result
        """
        if self._publisher[subject.name].subject._task is None: 
            pass
        elif self._publisher[subject.name].subject._task.done() is False: 
            self._publisher[subject.name].subject._task.cancel() 
        self.del_cmd_filter(subject.cmdset, subject.cmdid)
        return True

    def add_subject_info(self, subject, callback=None, *args):
        """ Request Data Subscription Underlying Interface

        :param subject: Data subscription corresponds to subject
        :param callback: The parsing function corresponding to the subscription data
        :return: bool: call result
        """
        subject.set_callback(callback, args[0], args[1])
        handler = SubHandler(self, subject, callback)
        self._dds_mutex.acquire()
        self._publisher[subject.name] = handler 
        self._dds_mutex.release()
        proto = protocol.ProtoAddSubMsg()
        proto._node_id = self._client.hostbyte 
        proto._sub_freq = subject.freq
        proto._sub_data_num = 1
        proto._msg_id = self.get_next_subject_id()
        subject._subject_id = proto._msg_id
        subject._task = None
        proto._sub_uid_list.append(subject.uid)
        return self._send_async_proto(proto, host2byte(9, 0))

    def del_subject_info(self, subject_name):
        """ Delete data subscription messages

        :param subject_name: The subscription subject to be deleted
        :return: bool: Delete data subscription results
        """
        logger.debug("Subscriber: del_subject_info: name:{0}, self._publisher:{1}".format(subject_name,
                     self._publisher))
        if subject_name in self._publisher:
            subject_id = self._publisher[subject_name].subject._subject_id 
            if self._publisher[subject_name].subject._task.done() is False: 
                self._publisher[subject_name].subject._task.cancel() 
            self._dds_mutex.acquire()
            del self._publisher[subject_name]
            self._dds_mutex.release()
            proto = protocol.ProtoDelMsg()
            proto._msg_id = subject_id
            proto._node_id = self._client.hostbyte 
            return self._send_async_proto(proto, host2byte(9, 0))
        else:
            logger.warning("Subscriber: fail to del_subject_info", subject_name) 

    def _send_sync_proto(self, proto, target=None):
        if not self._client:
            return False

        if target:
            msg = protocol.Msg(self._client.hostbyte, target, proto) # type: ignore
        else:
            msg = protocol.Msg(self._client.hostbyte, self._host, proto) # type: ignore
        try:
            resp_msg = self._client.send_sync_msg(msg) # type: ignore
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

    def _send_async_proto(self, proto, target=None):
        if not self._client:
            return False

        if target:
            msg = protocol.Msg(self._client.hostbyte, target, proto) # type: ignore
        else:
            msg = protocol.Msg(self._client.hostbyte, self._host, proto) # type: ignore
        try:
            return self._client.send_async_msg(msg) # type: ignore
        except Exception as e:
            logger.error("{0}: _send_async_proto, proto:{1}, exception:{2}".format(self.__class__.__name__, proto, e))
            return False
