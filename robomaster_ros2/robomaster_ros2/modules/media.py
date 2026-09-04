import importlib
import queue
import socket
import threading

import cv2
import numpy
from rclpy.logging import get_logger

try:
    import av
except ImportError:
    av = None

logger = get_logger('media')


class H264Decoder(object):
    """Decode an incremental H.264 byte stream through PyAV."""

    def __init__(self, codec=None):
        if codec is not None:
            self._codec = codec
        elif av is None:
            raise RuntimeError(
                "PyAV is required for video decoding. Install python3-av."
            )
        else:
            self._codec = av.CodecContext.create("h264", "r")
            try:
                self._codec.thread_type = "AUTO"
            except (AttributeError, ValueError):
                logger.warning(
                    "[H264Decoder] PyAV decoder threading is unavailable."
                )

    def decode(self, data):
        """Return complete BGR frames decoded from the next stream chunk."""
        frames = []
        for packet in self._codec.parse(data):
            for frame in self._codec.decode(packet):
                image = frame.to_ndarray(format="bgr24")
                if (image.ndim != 3 or image.shape[2] != 3 or
                        image.dtype != numpy.uint8):
                    raise ValueError(
                        "PyAV returned an invalid BGR video frame."
                    )
                frames.append(numpy.ascontiguousarray(image))
        return frames


class StreamConnection(object):

    def __init__(self, sock: socket.socket = None, robot_ip=None,
                 addr=None, ip_proto="udp"):
        self._sock = sock
        self._owns_socket = sock is None
        self._sock_queue = queue.Queue(1024)
        self._sock_recv = None
        self._recv_count = 0
        self._drop_count = 0
        self._receiving = False
        self._robot_ip = robot_ip
        self._addr = addr
        self._ip_proto = ip_proto.lower()

    def __del__(self):
        try:
            if (self._receiving or
                    (self._owns_socket and self._sock is not None)):
                self.disconnect()
        except Exception:
            pass

    @staticmethod
    def _clear_queue(data_queue):
        while True:
            try:
                data_queue.get_nowait()
            except queue.Empty:
                return

    def connect(self, addr=None, ip_proto=None):
        if self._receiving:
            return True
        if addr is not None:
            self._addr = addr
        if ip_proto is not None:
            self._ip_proto = ip_proto.lower()

        try:
            if self._sock is None:
                if self._addr is None:
                    logger.error("[StreamConnection] stream address is not set.")
                    return False
                if self._ip_proto == "tcp":
                    self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self._sock.settimeout(3.0)
                    self._sock.connect(self._addr)
                elif self._ip_proto == "udp":
                    self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    self._sock.bind(self._addr)
                else:
                    logger.error(
                        f"[StreamConnection] unsupported protocol {self._ip_proto}."
                    )
                    return False
                self._owns_socket = True
            elif self._sock.fileno() < 0:
                logger.error("[StreamConnection] socket is closed.")
                return False

            try:
                self._sock.setsockopt(
                    socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024
                )
            except OSError:
                pass
            self._sock.settimeout(0.2)
        except Exception as e:
            logger.error(
                f"[StreamConnection] connect to {self._addr}, exception {e}"
            )
            if self._owns_socket and self._sock is not None:
                self._sock.close()
                self._sock = None
            return False

        self._clear_queue(self._sock_queue)
        self._receiving = True
        self._sock_recv = threading.Thread(
            target=self._recv_task,
            name="robomaster-video-recv",
            daemon=True,
        )
        self._sock_recv.start()
        logger.info(
            f"[StreamConnection] connected using {self._ip_proto}."
        )
        return True

    def disconnect(self):
        self._receiving = False
        if self._owns_socket and self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        try:
            self._sock_queue.put_nowait(None)
        except queue.Full:
            try:
                self._sock_queue.get_nowait()
            except queue.Empty:
                pass
            self._sock_queue.put_nowait(None)
        if self._sock_recv:
            self._sock_recv.join(timeout=2.0)
        if self._owns_socket and self._sock is not None:
            self._sock.close()
            self._sock = None
        self._clear_queue(self._sock_queue)
        self._sock_recv = None
        self._recv_count = 0
        logger.info("[StreamConnection] disconnected.")

    def _recv_task(self):
        logger.info("[StreamConnection] receiving stream data.")
        while self._receiving:
            try:
                if self._sock is None:
                    break
                if self._ip_proto == "tcp":
                    data = self._sock.recv(65536)
                    addr = self._addr
                    if not data:
                        break
                else:
                    data, addr = self._sock.recvfrom(65535)
                if not self._receiving:
                    break
                if (self._robot_ip and addr and
                        addr[0] != self._robot_ip):
                    continue
                self._recv_count += 1
                if self._sock_queue.full():
                    self._sock_queue.get_nowait()
                    self._drop_count += 1
                    if self._drop_count % 100 == 1:
                        logger.warning(
                            "[StreamConnection] video receive queue overflow; "
                            f"dropped {self._drop_count} chunks."
                        )
                self._sock_queue.put_nowait(data)
            except socket.timeout:
                continue
            except Exception as e:
                if self._receiving:
                    logger.error(
                        f"[StreamConnection] receive exception: {e}"
                    )
                break
        self._receiving = False

    def read_buf(self, timeout=1):
        try:
            buf = self._sock_queue.get(timeout=timeout)
            return buf
        except queue.Empty:
            return None


class LiveView(object):

    def __init__(self, stream_conn: StreamConnection = None,
                 name='robot_name', frame_queue_size=2):
        self._video_stream_conn = stream_conn   
        self._video_decoder = None
        self._video_decoder_thread = None
        self._video_display_thread = None
        self._video_frame_queue = queue.Queue(frame_queue_size)
        self._video_streaming = False
        self._displaying = False
        self._video_frame_count = 0
        self._name = name
        self._audio_stream_conn = None
        self._audio_decoder = None
        self._audio_decoder_thread = None
        self._audio_frame_queue = queue.Queue(32)
        self._audio_streaming = False
        self._audio_frame_count = 0

    def __del__(self):
        try:
            self.stop()
        except Exception:
            pass

    def stop(self):
        if self._video_streaming:
            self.stop_video_stream()
        if self._audio_streaming:
            self.stop_audio_stream()

    def start_video_stream(self, display=False):
        if self._video_streaming:
            return True
        if self._video_stream_conn is None:
            logger.error(
                f"[Liveview] [{self._name}] video connection is not set."
            )
            return False
        try:
            self._video_decoder = H264Decoder()
            logger.info(
                f"[Liveview] [{self._name}] using PyAV H.264 decoder."
            )
            if not self._video_stream_conn.connect():
                self._video_decoder = None
                return False
            self._video_streaming = True
            self._video_decoder_thread = threading.Thread(
                target=self._video_decoder_task,
                name=f"{self._name}-video-decode",
                daemon=True,
            )
            self._video_decoder_thread.start()
            if display:
                self._displaying = True
                self._video_display_thread = threading.Thread(
                    target=self._video_display_task,
                    name=f"{self._name}-video-display",
                    daemon=True,
                )
                self._video_display_thread.start()
        except Exception as e:
            self._video_streaming = False
            self._video_decoder = None
            self._video_stream_conn.disconnect()
            logger.error(f"[Liveview] [{self._name}] start_video_stream, exception {e}")
            return False
        return True

    def stop_video_stream(self):
        try:
            self._video_streaming = False
            self._displaying = False
            if self._video_stream_conn:
                self._video_stream_conn.disconnect()
            if self._video_display_thread:
                try:
                    self._video_frame_queue.put_nowait(None)
                except queue.Full:
                    pass
                self._video_display_thread.join(timeout=2.0)
            if self._video_decoder_thread:
                self._video_decoder_thread.join(timeout=2.0)
            StreamConnection._clear_queue(self._video_frame_queue)
            self._video_display_thread = None
            self._video_decoder_thread = None
            self._video_decoder = None
        except Exception as e:
            logger.error(f"[Liveview] [{self._name}] disconnect exception {e}")
            return False
        logger.info(f"[Liveview] [{self._name}] stop_video_stream stopped.")
        return True

    def read_video_frame(self, timeout=1, strategy="newest"):
        if strategy not in ("pipeline", "newest"):
            logger.error(
                f"[Liveview] [{self._name}] read_video_frame, "
                f"unsupported strategy:{strategy}"
            )
            return None

        try:
            frame = self._video_frame_queue.get(timeout=max(timeout, 0.0))
        except queue.Empty:
            return None

        if strategy == "newest":
            while True:
                try:
                    newer_frame = self._video_frame_queue.get_nowait()
                    if newer_frame is not None:
                        frame = newer_frame
                except queue.Empty:
                    break
        return frame

    def _h264_decode(self, data):
        if self._video_decoder is None:
            return []
        try:
            return self._video_decoder.decode(data)
        except Exception as e:
            logger.error(f"[Liveview] [{self._name}] h264_decode, exception {e}")
            return []

    def _video_decoder_task(self):
        logger.info(f"[Liveview] [{self._name}] _video_decoder_task, started!")
        while self._video_streaming:
            buf = self._video_stream_conn.read_buf()
            if not self._video_streaming:
                break
            if buf:
                data = bytearray(buf)
                while len(data) < 256 * 1024:
                    next_buf = self._video_stream_conn.read_buf(timeout=0)
                    if not next_buf:
                        break
                    data.extend(next_buf)
                frames = self._h264_decode(bytes(data))
                for frame in frames:
                    try:
                        if self._video_frame_queue.full():
                            self._video_frame_queue.get_nowait()
                        self._video_frame_queue.put_nowait(frame)
                        self._video_frame_count += 1
                    except queue.Full:
                        pass
        logger.info(f"[Liveview] [{self._name}] _video_decoder_task, quit.")

    def _video_display_task(self):
        name = f"RoboMaster LiveView - {self._name}"
        logger.info(f"[Liveview] [{self._name}] _video_display_task, started!")
        while self._displaying and self._video_streaming:
            try:
                frame = self._video_frame_queue.get(timeout=0.2)
                if frame is None:
                    break
            except queue.Empty:
                continue
            cv2.imshow(name, frame)
            cv2.waitKey(1)
        cv2.destroyWindow(name)
        logger.info(f"[Liveview] [{self._name}] _video_display_task, quit.")

    def read_audio_frame(self, timeout=1):
        return self._audio_frame_queue.get(timeout=timeout)

    def start_audio_stream(self, addr=None, ip_proto="tcp"):
        try:
            try:
                libmedia_codec = importlib.import_module("libmedia_codec")
            except ImportError as exc:
                raise RuntimeError(
                    "libmedia_codec is required for EP audio decoding."
                ) from exc
            self._audio_stream_conn = StreamConnection(
                addr=addr, ip_proto=ip_proto
            )
            self._audio_decoder = libmedia_codec.OpusDecoder()
            if not self._audio_stream_conn.connect():
                return False
            self._audio_streaming = True
            self._audio_decoder_thread = threading.Thread(
                target=self._audio_decoder_task,
                name=f"{self._name}-audio-decode",
                daemon=True,
            )
            self._audio_decoder_thread.start()
        except Exception as e:
            logger.error(f"[Liveview] [{self._name}] start_audio_stream, exception {e}")
            return False
        return True

    def stop_audio_stream(self):
        try:
            logger.info(f"[Liveview] [{self._name}] stop_audio_stream stopping...")
            self._audio_streaming = False
            if self._audio_stream_conn:
                self._audio_stream_conn.disconnect()
            if self._audio_decoder_thread:
                self._audio_decoder_thread.join(timeout=2.0)
            StreamConnection._clear_queue(self._audio_frame_queue)
        except Exception as e:
            logger.error(f"[Liveview] [{self._name}] disconnect exception {e}")
            return False
        logger.info(f"[Liveview] [{self._name}] stop_video_stream stopped.")
        return True

    def _audio_decoder_task(self):
        while self._audio_streaming:
            buf = self._audio_stream_conn.read_buf()
            if buf:
                if len(buf) != 0:
                    frame = self._audio_decoder.decode(buf)
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
