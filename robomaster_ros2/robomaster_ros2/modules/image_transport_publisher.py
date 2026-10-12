import atexit
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix, PackageNotFoundError
from sensor_msgs.msg import Image


class CompressedImagePublisher:
    def __init__(self, node, topic, qos):
        self._node = node
        self._publisher = None
        self._process = None
        try:
            get_package_prefix('compressed_image_transport')
            prefix = get_package_prefix('robomaster_image_transport')
        except PackageNotFoundError as error:
            raise RuntimeError(
                'Camera publishing requires compressed_image_transport and '
                'robomaster_image_transport; install dependencies and build '
                'the workspace.'
            ) from error

        executable = Path(prefix) / 'lib/robomaster_image_transport/compressed_republisher'
        bridge_name = node.get_name() + '_image_transport'
        namespace = node.get_namespace()
        image_base = node.resolve_topic_name(topic, only_expand=True)
        image_topic = node.resolve_topic_name(topic)
        compressed_topic = node.resolve_topic_name(topic + '/compressed')
        self._process = subprocess.Popen([
            str(executable), '--ros-args',
            '-r', '__node:=' + bridge_name,
            '-r', '__ns:=' + namespace,
            '-r', 'image:=' + image_topic,
            '-r', image_base + '/compressed:=' + compressed_topic,
        ])
        try:
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                if self._process.poll() is not None:
                    raise RuntimeError(
                        f'Compressed image transport exited for {image_topic}.'
                    )
                publishers = node.get_publishers_info_by_topic(compressed_topic)
                if any(
                    info.node_name == bridge_name and
                    info.node_namespace == namespace and
                    info.topic_type == 'sensor_msgs/msg/CompressedImage'
                    for info in publishers
                ):
                    break
                if not node.context.ok():
                    raise RuntimeError('ROS shut down during camera transport startup.')
                time.sleep(0.05)
            else:
                raise RuntimeError(
                    f'Compressed image transport did not start for {image_topic} '
                    'within 5 seconds.'
                )
            self._publisher = node.create_publisher(Image, topic, qos)
        except BaseException:
            self.close()
            raise
        atexit.register(self.close)

    def publish(self, image):
        if self._process is None:
            return
        if self._process.poll() is not None:
            raise RuntimeError(
                'Required compressed image transport process exited for '
                f'{self._node.get_fully_qualified_name()}.'
            )
        self._publisher.publish(image)

    def close(self):
        atexit.unregister(self.close)
        process = self._process
        self._process = None
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        if self._publisher is not None:
            self._node.destroy_publisher(self._publisher)
            self._publisher = None
