import os
import time

import cv2
import numpy as np
import rclpy
import requests
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DepthBridgeNode(Node):
    """Publish the newest MiDaS PNG from the backend as a ROS depth image."""

    def __init__(self):
        super().__init__('depth_bridge_node')

        self.declare_parameter(
            'backend_url',
            os.environ.get(
                'FYP_BACKEND_URL',
                'http://127.0.0.1:8000/latest-depth-image',
            ),
        )
        self.declare_parameter('depth_topic', '/smartphone/depth')
        self.declare_parameter('depth_frame_id', 'camera_depth_frame')
        self.declare_parameter('poll_period', 0.2)
        self.declare_parameter('request_timeout', 0.5)
        self.declare_parameter('offline_log_period', 10.0)

        self.backend_url = self.get_parameter('backend_url').value
        self.depth_frame_id = self.get_parameter('depth_frame_id').value
        self.request_timeout = max(
            0.05, float(self.get_parameter('request_timeout').value)
        )
        self.offline_log_period = max(
            1.0, float(self.get_parameter('offline_log_period').value)
        )
        poll_period = max(0.05, float(self.get_parameter('poll_period').value))
        depth_topic = self.get_parameter('depth_topic').value

        self.depth_pub = self.create_publisher(
            Image,
            depth_topic,
            qos_profile_sensor_data,
        )
        self._session = requests.Session()
        self._last_sequence = None
        self._last_offline_log = float('-inf')
        self.timer = self.create_timer(poll_period, self.publish_depth)

        self.get_logger().info(
            f'Smartphone depth bridge ready: {self.backend_url} -> {depth_topic}'
        )

    def _log_backend_unavailable(self, detail):
        """Report an unavailable backend without flooding a healthy ROS graph."""
        now = time.monotonic()
        if now - self._last_offline_log >= self.offline_log_period:
            self.get_logger().warning(
                f'Depth backend unavailable; retrying in the background: {detail}'
            )
            self._last_offline_log = now

    def publish_depth(self):
        try:
            response = self._session.get(
                self.backend_url,
                timeout=self.request_timeout,
                headers={'Cache-Control': 'no-cache'},
            )
        except requests.RequestException as exc:
            self._log_backend_unavailable(str(exc))
            return

        if response.status_code != requests.codes.ok:
            self._log_backend_unavailable(f'HTTP {response.status_code}')
            return

        content_type = response.headers.get('Content-Type', '')
        if content_type and not content_type.startswith('image/'):
            self._log_backend_unavailable(
                f'unexpected content type {content_type!r}'
            )
            return

        # The backend increments this header only after it writes a new MiDaS
        # frame. Do not keep feeding a frozen frame into the local costmap.
        sequence = response.headers.get('X-FYP-Depth-Sequence')
        if sequence and sequence == self._last_sequence:
            return

        image_array = np.frombuffer(response.content, dtype=np.uint8)
        depth = cv2.imdecode(image_array, cv2.IMREAD_GRAYSCALE)
        if depth is None or depth.size == 0:
            self._log_backend_unavailable('could not decode MiDaS PNG')
            return

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.depth_frame_id
        msg.height, msg.width = depth.shape
        msg.encoding = 'mono8'
        msg.is_bigendian = 0
        msg.step = depth.shape[1]
        msg.data = depth.tobytes()

        self.depth_pub.publish(msg)
        self._last_sequence = sequence

    def destroy_node(self):
        self._session.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DepthBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
