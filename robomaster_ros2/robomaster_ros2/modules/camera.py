from rclpy.logging import get_logger
logger = get_logger('camera')

import numpy
try:
    from . import media
    from .globals import *
except ImportError:
    import media
    from globals import *
except Exception as e:
    logger.error(f"Import Error pass two: {e}")

__all__ = ['Camera', 'EPCamera', 'TelloCamera', 'STREAM_360P', 'STREAM_540P', 'STREAM_720P']

class Camera(object):

    def __init__(self, robot):
        self._robot = robot
        self._client = robot._client
        self._video_enable = False
        self._audio_enable = False
        self._liveview = media.LiveView()

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
        img = numpy.array(frame)
        return img


