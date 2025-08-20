import threading
import time

from rclpy.logging import get_logger
logger = get_logger('action')

try:
    from . import protocol
    from .globals import *
except ImportError:
    import protocol
    from globals import *
except Exception as e:
    logger.error(f"Import Error pass two: {e}")

__all__ = ['Action', 'ActionDispatcher', 'ACTION_IDLE', 'ACTION_RUNNING', 'ACTION_SUCCEEDED', 'ACTION_FAILED',
           'ACTION_STARTED', 'ACTION_ABORTED', 'ACTION_EXCEPTION', 'ACTION_REJECTED']

ACTION_IDLE = 'action_idle'
ACTION_RUNNING = 'action_running'
ACTION_SUCCEEDED = 'action_succeeded'
ACTION_FAILED = 'action_failed'
ACTION_STARTED = 'action_started'
ACTION_ABORTING = 'action_aborting'
ACTION_ABORTED = 'action_aborted'
ACTION_REJECTED = 'action_rejected'
ACTION_EXCEPTION = 'action_exception'

ACTION_NOW = 'action_now'
ACTION_QUEUE = 'action_queue'
ACTION_REQUEST = 'action_request'
_VALID_ACTION_TYPES = {ACTION_NOW, ACTION_QUEUE, ACTION_REQUEST}

registered_actions = {}

class _AutoRegisterAction(type):
    '''hepler to automatically register Proto Class whereever they're defined '''

    def __new__(mcs, name, bases, attrs, **kw):
        return super().__new__(mcs, name, bases, attrs, **kw)

    def __init__(cls, name, bases, attrs, **kw):
        super().__init__(name, bases, attrs, **kw)
        if name == 'Action':
            return
        key = name
        if key in registered_actions.keys():
            raise ValueError("Duplicate proto class {0}".format(name))

        if attrs['_action_proto_cls'] is None or attrs['_push_proto_cls'] is None:
            raise ValueError('action must specific proto cls and push cls')

        registered_actions[key] = cls


class Action(metaclass=_AutoRegisterAction):
    _action_mutex = threading.Lock()
    _next_action_id = RM_SDK_FIRST_ACTION_ID
    _action_proto_cls = None
    _push_proto_cls = None
    _target = host2byte(0, 0)

    def __init__(self, **kw):
        super().__init__(**kw)
        self._action_id = -1
        self._state = ACTION_IDLE
        self._failure_reason = None
        self._percent = 0

        # wait on event.
        self._event = threading.Event()
        self._obj = None
        self._on_state_changed = None

    def _get_next_action_id(self):
        self.__class__._action_mutex.acquire()
        action_id = self.__class__._next_action_id
        if self.__class__._next_action_id == RM_SDK_LAST_ACTION_ID:
            self.__class__._next_action_id = RM_SDK_FIRST_ACTION_ID
        else:
            self.__class__._next_action_id = self.__class__._next_action_id + 1
        self.__class__._action_mutex.release()
        return action_id

    def __repr__(self):
        return "<action, name:{0} id:{1:d}, state:{2}, percent:{3:d}%>".format(
            self.__class__.__name__, self._action_id, self._state, self._percent)

    @property
    def target(self):
        return self._target

    @property
    def is_running(self):
        """ Whether it is running or not. """
        return self._state == ACTION_RUNNING or self._state == ACTION_STARTED

    @property
    def is_completed(self):
        return (self._percent == 100 or self._state is ACTION_SUCCEEDED) or (self._state is ACTION_FAILED) or \
               (self.state is ACTION_EXCEPTION) or (self.state is ACTION_REJECTED)

    @property
    def _is_aborting(self):
        """ Whether the task is in the canceled state """
        return self._state == ACTION_ABORTING

    @property
    def has_succeeded(self):
        """ Whether it has been successfully completed """
        return self._state == ACTION_SUCCEEDED

    @property
    def has_failed(self):
        """ Whether the implementation has failed """
        return self._state == ACTION_FAILED

    @property
    def failure_reason(self):
        """ Get the reason for implementation failure """
        return self._failure_reason

    @property
    def state(self):
        """ Returns the current task action status. """
        return self._state

    def encode(self):
        raise NotImplementedError()

    def make_action_key(self):
        return self._action_proto_cls._cmdid * 256 + self._action_id 

    def _update_action_state(self, proto_state):
        if proto_state == 0:
            self._changeto_state(ACTION_RUNNING)
        elif proto_state == 1:
            self._changeto_state(ACTION_SUCCEEDED)
        elif proto_state == 2:
            self._changeto_state(ACTION_FAILED)
        elif proto_state == 3:
            self._changeto_state(ACTION_STARTED)

    def _changeto_state(self, state):
        """ Modify action status """
        if state != self._state:
            orgin = self._state
            self._state = state
            logger.debug("Action, name:{0} _changeto_state from {1} "
                        "to {2}".format(self.__class__.__name__, orgin, self._state))

            if self._on_state_changed and self._obj:
                self._on_state_changed(self._obj, self, orgin, self._state)
            if self.is_completed:
                self._event.set()

    def wait_for_completed(self, timeout=None):
        """ Wait for task action until completion

        :param timeout: Timeout, if the task action is not completed before the timeout, it will be returned directly
        :return: bool: If the action completes within the specified time, return True; if the action times out, return False.
        """
        if self._event.isSet() and self.is_completed:
            return True

        if timeout:
            self._event.wait(timeout)
            if not self._event.isSet():
                logger.debug("Action: wait_for_completed timeout.")
                self._changeto_state(ACTION_EXCEPTION)
                return False
        else:
            self._event.wait()
            if not self._event.isSet():
                logger.debug("Action: wait_for_completed timeout.")
                self._changeto_state(ACTION_EXCEPTION)
                return False
        return True

    def _abort(self):
        """ Cancel Task Action """
        self._changeto_state(ACTION_ABORTED)
        self._event.set()

    def found_proto(self, proto):
        if proto.cmdset == self._action_proto_cls._cmdset and proto.cmdid == self._action_proto_cls._cmdid: 
            return True
        else:
            return False

    def found_action(self, proto):
        if proto.cmdset == self._push_proto_cls._cmdset and proto.cmdid == self._push_proto_cls._cmdid: 
            return True
        else:
            return False

class ActionDispatcher(object):

    def __init__(self, client=None):
        self._client = client
        self._in_progress_mutex = threading.Lock()
        self._in_progress = {}
        self.start_time = time.time()

    def initialize(self):
        self._client.add_handler(self, "ActionDispatcher", self._on_recv) 

    @property
    def has_in_progress_actions(self):
        """ Availability of active tasks """
        return len(self._in_progress) > 0

    @classmethod
    def _on_recv(cls, self, msg):
        logger.debug("ActionDispatcher: on_recv, in_progress:{0}".format(self._in_progress))
        proto = msg.get_proto()
        if proto is None:
            return

        action = None
        found_proto = False
        found_action = False

        self._in_progress_mutex.acquire()
        for key in self._in_progress.keys():
            action = self._in_progress[key]
            if action:
                if action.found_proto(proto):
                    found_proto = True
                    break
                if action.found_action(proto):
                    found_action = True
                    break
            else:
                logger.warning("[ActionDispatcher] in_progress action is None")
        self._in_progress_mutex.release()

        if found_proto:
            if proto._retcode == 0:
                if proto._accept == 0:
                    action._changeto_state(ACTION_STARTED) 
                elif proto._accept == 1:
                    action._changeto_state(ACTION_REJECTED) 
                elif proto._accept == 2:
                    action._changeto_state(ACTION_SUCCEEDED) 
            else:
                action._changeto_state(ACTION_FAILED) 
            logger.debug("ActionDispatcher, found_proto, action:{0}".format(action))

        if found_action:
            # if isinstance(action, TextAction):
            #     logger.debug("ActionDispatcher, found text action, and will update_from_push action:{0}".format(action))
            #     if action.is_running:
            #         action.update_from_push(proto) 
            #     return

            if proto._action_id == action._action_id: 
                logger.debug("ActionDispatcher, found action, and will update_from_push action:{0}".format(action))
                if action.is_running: 
                    action.update_from_push(proto) 

    def get_msg_by_action(self, action):
        proto = action.encode()
        # if isinstance(action, TextAction):
        #     action_msg = protocol.TextMsg(proto)
        #else:
        proto._action_id = action._action_id
        action_msg = protocol.Msg(self._client.hostbyte, action.target, proto) 
        return action_msg

    def send_async_action(self, action, action_type=ACTION_NOW):
        """ Send task action commands """
        action._action_id = action._get_next_action_id()
        action_msg = self.get_msg_by_action(action)
        self._client.send_msg(action_msg) 
        logger.debug("ActionDispatcher: send_action, action:{0}".format(action))

    def send_action(self, action, action_type=ACTION_NOW):
        self.start_time = time.time()
        """ Send task action commands """
        action._action_id = action._get_next_action_id()
        
        if self.has_in_progress_actions:
            self._in_progress_mutex.acquire()
            for k in self._in_progress:
                act = self._in_progress[k]
                if action.target == act.target:
                    if time.time() - self.start_time < 3:                        
                        action = list(self._in_progress.values())[0]
                        logger.warn("[ActionDispatcher] Robot is already performing {0} action(s) {1}".format(len(self._in_progress), action))
                        # raise Exception("Robot is already performing {0} action(s) {1}".format(
                        #     len(self._in_progress), action))
                    else:
                        logger.warn("ActionDispatcher, del action:{0}. reason: timeout".format(action))
                        del self._in_progress[k]
            self._in_progress_mutex.release()
        if action.is_running:
            raise Exception("Action is already running")

        action_msg = self.get_msg_by_action(action)
        action_key = action.make_action_key()
        self._in_progress[action_key] = action
        self._client.add_handler(self, "ActionDispatcher", self._on_recv) 
        action._obj = self
        action._on_state_changed = self._on_action_state_changed

        self._client.send_msg(action_msg) 
        # if isinstance(action, TextAction):
        #     action._changeto_state(ACTION_STARTED)
        logger.debug("ActionDispatcher: send_action, action:{0}".format(action))

    @classmethod
    def _on_action_state_changed(cls, self, action, orgin, target):
        if action.is_completed:
            action_key = action.make_action_key()
            logger.debug("ActionDispatcher, in_progress:{0}".format(self._in_progress))
            self._in_progress_mutex.acquire()
            if action_key in self._in_progress.keys():
                logger.debug("ActionDispatcher, del action:{0}".format(action))
                del (self._in_progress[action_key])
            else:
                logger.warning("[ActionDispatcher] del failed, action: {0}".format(action))
            self._in_progress_mutex.release()
