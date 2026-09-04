from rclpy.logging import get_logger
logger = get_logger('camera')

import numpy
try:
    from . import media, protocol
    from .globals import *
except ImportError:
    import media, protocol
    from globals import *
except Exception as e:
    logger.error(f"Import Error pass two: {e}")

__all__ = ['Camera', 'EPCamera', 'TelloCamera', 'STREAM_360P', 'STREAM_540P', 'STREAM_720P']

class Camera(object):

    def __init__(self, robot, stream_conn=None):
        self._robot = robot
        self._client = robot._client
        self._video_enable = False
        self._audio_enable = False
        self._liveview = media.LiveView(
            stream_conn, robot._conf._name
        )

    def start_video_stream(self, display=False):
        pass

    def stop_video_stream(self):
        pass

    #NOT USED IN DRONES
    def read_video_frame(self, timeout=3, strategy="newest"):
        return self._liveview.read_video_frame(timeout, strategy)

    #NOT USED IN DRONES
    def read_cv2_image(self, timeout=3, strategy="newest"):
        frame = self.read_video_frame(timeout, strategy)
        if frame is None:
            return None
        img = numpy.asarray(frame)
        return img


class EPCamera(Camera):
    """RoboMaster EP/Core H.264 camera stream."""

    _host = protocol.host2byte(1, 0)
    _resolution_values = {
        STREAM_720P: 0,
        STREAM_360P: 1,
        STREAM_540P: 2,
    }

    def __init__(self, robot):
        self._conf = robot._conf
        self._resolution = STREAM_720P
        stream_conn = media.StreamConnection(
            addr=(
                self._conf.default_robot_addr[0],
                self._conf.video_stream_port,
            ),
            robot_ip=self._conf.default_robot_addr[0],
            ip_proto=self._conf.video_stream_proto,
        )
        super().__init__(robot, stream_conn)

    @property
    def video_stream_addr(self):
        return (
            self._conf.default_robot_addr[0],
            self._conf.video_stream_port,
        )

    def start_video_stream(self, display=False, resolution=STREAM_720P):
        if self._video_enable:
            return True
        if not self._stream_ctrl(1, 1, resolution):
            logger.error(
                f"[Camera] [{self._conf._name}] failed to enable SDK stream mode."
            )
            return False
        if not self._stream_ctrl(2, 1, resolution):
            logger.error(
                f"[Camera] [{self._conf._name}] failed to enable video stream."
            )
            self._stream_ctrl(1, 0, resolution)
            return False
        if not self._liveview.start_video_stream(display):
            logger.error(
                f"[Camera] [{self._conf._name}] failed to connect to "
                f"{self.video_stream_addr}."
            )
            self._stream_ctrl(2, 0, resolution)
            self._stream_ctrl(1, 0, resolution)
            return False
        self._resolution = str(resolution).lower()
        self._video_enable = True
        return True

    def stop_video_stream(self):
        if not self._video_enable:
            return True
        video_stopped = self._stream_ctrl(2, 0, self._resolution)
        sdk_stopped = self._stream_ctrl(1, 0, self._resolution)
        liveview_stopped = self._liveview.stop_video_stream()
        self._video_enable = False
        return video_stopped and sdk_stopped and liveview_stopped

    def _stream_ctrl(self, ctrl, state, resolution):
        resolution = str(resolution).lower()
        resolution_value = self._resolution_values.get(resolution)
        if resolution_value is None:
            logger.warning(
                f"[Camera] [{self._conf._name}] unsupported resolution "
                f"{resolution}; using {STREAM_720P}."
            )
            resolution_value = self._resolution_values[STREAM_720P]

        proto = protocol.ProtoStreamCtrl()
        proto._ctrl = ctrl
        proto._conn_type = 0  # This driver connects EP robots over Wi-Fi.
        proto._state = state
        proto._resolution = resolution_value
        return self._robot._send_sync_proto(proto, self._host)

    def get_cv2_frame(self, timeout=1, strategy="newest"):
        return self.read_cv2_image(timeout, strategy)

    def take_photo(self):
        return self._robot._send_sync_proto(
            protocol.ProtoTakePhoto(), self._host
        )

    def stop(self):
        if self._video_enable:
            self.stop_video_stream()
        self._liveview.stop()
