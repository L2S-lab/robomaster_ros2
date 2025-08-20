from rclpy.logging import get_logger
logger = get_logger('media')

import threading
import queue
import libmedia_codec 
import numpy
import cv2
import time
import socket
import os

try:
    from .globals import *
    from .common import *
except ImportError:
    from globals import *
    from common import *
except Exception as e:
    logger.error(f"Import Error pass two: {e}")

class StreamConnection(object):

    def __init__(self,sock:socket.socket=None, robot_ip=None):  
        self._sock = sock
        self._sock_queue = queue.Queue(32)
        self._sock_recv = None
        self._recv_count = 0
        self._receiving = False
        self._robot_ip = robot_ip
        if self._sock is None or self._robot_ip is None:
            logger.error(f"[StreamConnection] socket or robot_ip is None.")
            self.__del__()

    def __del__(self):
        if self._receiving:
            self.disconnect()

    def connect(self):
        try:
            if self._sock is None or self._sock._closed:
                logger.error(f"[StreamConnection] socket is closed or not initialized.")
                return False
            else:
                binding_addr = self._sock.getsockname()
                if binding_addr[-1] == TELLO_VIDEO_PORT:
                    pass
        except Exception as e:
            logger.error(f"[StreamConnection] connect addr, exception {e}")
            return False
        self._sock_recv = threading.Thread(target=self._recv_task)
        self._sock_recv.start()
        logger.info(f"[StreamConnection] successfully!")
        return True

    def disconnect(self):
        self._receiving = False
        self._sock_queue.put(None)
        if self._sock_recv:
            self._sock_recv.join()
        self._sock_queue.queue.clear()
        self._recv_count = 0
        logger.info(f"[StreamConnection] disconnected")

    def _recv_task(self):
        self._receiving = True
        logger.info(f"[StreamConnection] _recv_task, Start to receiving Data...")
        while self._receiving:
            try:
                if self._sock is None:
                    break
                data, addr = self._sock.recvfrom(4096)
                logger.debug(f"[StreamConnection] _recv_task, recv data from {addr}")
                if not self._receiving:
                    break
                self._recv_count += 1
                if self._sock_queue.full():
                    self._sock_queue.get()
                self._sock_queue.put(data)
            except socket.timeout:
                logger.debug(f"[StreamConnection] _recv_task， recv data timeout!")
                continue
            except Exception as e:
                logger.error(f"[StreamConnection] recv, exceptions:{str(e)}")
                self._receiving = False
                return

    def read_buf(self, timeout=1):
        try:
            buf = self._sock_queue.get(timeout=timeout)
            return buf
        except Exception as e:
            logger.debug(f"[StreamConnection] read_buf, exception {str(e)}")
            return None

class LiveView(object):

    def __init__(self, stream_conn:StreamConnection=None, name='robot_name'):
        self._video_stream_conn = stream_conn   
        self._video_decoder = libmedia_codec.H264Decoder()
        self._video_decoder_thread = None
        self._video_display_thread = None
        self._video_frame_queue = queue.Queue(64)
        self._video_streaming = False
        self._displaying = False
        self._video_frame_count = 0
        self._name = name
        self._prev_frame = None
        os.makedirs(os.getcwd()+'/.tmp', exist_ok=True)
        self._filename = os.getcwd()+'/.tmp/'+self._name+'_tmp_mem.dat'
        self._mem = None

        #self._audio_stream_conn = StreamConnection()
        #self._audio_decoder = libmedia_codec.OpusDecoder()
        #self._audio_decoder_thread = None
        #self._audio_playing_thread = None
        #self._audio_frame_queue = queue.Queue(32)
        #self._audio_streaming = False
        #self._playing = False
        #self._audio_frame_count = 0

    def __del__(self):
        self.stop()

    def stop(self):
        if self._video_streaming:
            self.stop_video_stream()
        self._mem.flush()
        os.remove(self._filename)
        #if self._audio_streaming:
        #    self.stop_audio_stream()

    def start_video_stream(self, display=False):
        try:
            self._video_stream_conn.connect()
            self._video_streaming = True
            self._video_decoder_thread = threading.Thread(target=self._video_decoder_task)
            self._video_decoder_thread.start()
            if display:
                self._video_display_thread = threading.Thread(target=self._video_display_task)
                self._video_display_thread.start()
        except Exception as e:
            logger.error(f"[Liveview] [{self._name}] start_video_stream, exception {e}")
            return False
        return True

    def stop_video_stream(self):
        try:
            self._video_streaming = False
            self._displaying = False
            if self._video_stream_conn:
                self._video_stream_conn.disconnect()
            if self._displaying:
                if self._video_display_thread:
                    self._video_frame_queue.put(None)
                    self._video_display_thread.join()
            if self._video_decoder_thread:
                self._video_decoder_thread.join()
            self._video_frame_queue.queue.clear()
        except Exception as e:
            logger.error(f"[Liveview] [{self._name}] disconnect exception {e}")
            return False
        logger.info(f"[Liveview] [{self._name}] stop_video_stream stopped.")
        return True

    def read_video_frame(self, timeout=1, strategy="newest"):
        if strategy == "pipeline":
            if self._video_frame_queue.empty():
                return self._prev_frame
            self._prev_frame = self._video_frame_queue.get(timeout=timeout)
            return self._prev_frame
        elif strategy == "newest":
            if not self._video_frame_queue.empty():
                while self._video_frame_queue.qsize() > 1:
                    self._video_frame_queue.get(timeout=timeout)
                self._prev_frame = self._video_frame_queue.get(timeout=timeout)
                return self._prev_frame
            else:
                return self._prev_frame
        else:
            logger.error(f"[Liveview] [{self._name}] read_video_frame, unsupported strategy:{strategy}")

    def _h264_decode(self, data):
        res_frame_list = []
        try:
            frames = self._video_decoder.decode(data)
            for frame_data in frames:
                (frame, width, height, ls) = frame_data
                if not self._mem:
                    self._mem = numpy.memmap(self._filename, dtype=numpy.ubyte, mode="w+", shape=len(frame))
                if frame:
                    self._mem = frame
                    try:
                        frame = numpy.frombuffer(self._mem, dtype=numpy.ubyte)
                        frame = (frame.reshape((height, width, 3)))
                        res_frame_list.append(frame)
                    except MemoryError as e:
                        logger.error(f"[Liveview] [{self._name}] numpy, MemoryError: {e}")
                        pass
        except:
            logger.error(f"[Liveview] [{self._name}] h264_decode, exception {e}")
            pass
        return res_frame_list

    def _video_decoder_task(self):
        self._video_streaming = True
        logger.info(f"[Liveview] [{self._name}] _video_decoder_task, started!")
        while self._video_streaming:
            data = b''
            buf = self._video_stream_conn.read_buf()
            if not self._video_streaming:
                break
            if buf:
                data += buf
                frames = self._h264_decode(data)
                for frame in frames:
                    try:
                        if self._video_frame_queue.full():
                            self._video_frame_queue.get()
                            #with self._video_frame_queue.mutex:
                            #    self._video_frame_queue.queue.clear()
                        self._video_frame_queue.put(frame, timeout=1)
                    except Exception as e:
                        logger.debug(f"[Liveview] [{self._name}] _video_decoder_task, decoder queue is full, {str(e)}")
                        continue
        logger.info(f"[Liveview] [{self._name}] _video_decoder_task, quit.")

    def _video_display_task(self, name="RoboMaster LiveView"):
        self._displaying = True
        logger.info(f"[Liveview] [{self._name}] _video_display_task, started!")
        while self._displaying & self._video_streaming:
            try:
                frame = self._video_frame_queue.get()
                if frame is None:
                    logger.warning(f"[Liveview] [{self._name}] _video_display_task, get frame None.")
                    if not self._displaying:
                        break
            except Exception as e:
                logger.warning(f"[Liveview] [{self._name}] display_task, video_frame_queue is empty, e {e}")
                continue
            img = numpy.array(frame)
            cv2.imshow(name, img)
            cv2.waitKey(1)
        logger.info(f"[Liveview] [{self._name}] _video_display_task, quit.")

    def read_audio_frame(self, timeout=1):
        return self._audio_frame_queue.get(timeout=timeout)

    def start_audio_stream(self, addr=None, ip_proto="tcp"):
        try:
            self._audio_stream_conn.connect(addr, ip_proto)
            self._audio_decoder_thread = threading.Thread(
                target=self._audio_decoder_task)
            self._audio_decoder_thread.start()
        except Exception as e:
            logger.error(f"[Liveview] [{self._name}] start_audio_stream, exception {e}")
            return False
        return True

    def stop_audio_stream(self):
        try:
            logger.info(f"[Liveview] [{self._name}] stop_audio_stream stopping...")
            self._audio_streaming = False
            if self._audio_decoder_thread:
                self._audio_decoder_thread.join()
            self._audio_stream_conn.disconnect()
            self._video_frame_queue.queue.clear()
            # make sure the robot is disconnected
            time.sleep(0.5)
        except Exception as e:
            logger.error(f"[Liveview] [{self._name}] disconnect exception {e}")
            return False
        logger.info(f"[Liveview] [{self._name}] stop_video_stream stopped.")
        return True

    def _audio_decoder_task(self):
        self._audio_streaming = True
        while self._audio_streaming:
            data = b''

            buf = self._audio_stream_conn.read_buf()
            if buf:
                data += buf

                if len(data) != 0:
                    frame = self._audio_decoder.decode(data)
                    if frame:
                        try:
                            self._audio_frame_count += 1
                            logger.info(f"[Liveview] [{self._name}] audio_decoder_task, get frame {self._audio_frame_count}.")
                            self._audio_frame_queue.put(frame, timeout=1)
                        except Exception as e:
                            if not self._audio_streaming:
                                break
                            logger.warning(f"[Liveview] [{self._name}] _audio_decoder_task, audio_frame_queue full, {e}")
                            continue
        logger.info(f"[Liveview] [{self._name}] _audio_decoder_task, quit.")

