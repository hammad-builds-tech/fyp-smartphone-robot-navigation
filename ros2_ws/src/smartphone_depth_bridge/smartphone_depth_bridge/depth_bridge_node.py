import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import requests
import cv2
import numpy as np


class DepthBridgeNode(Node):

    def __init__(self):
        super().__init__('depth_bridge_node')

        self.declare_parameter(
            'backend_url',
            os.environ.get(
                'FYP_BACKEND_URL',
                'http://127.0.0.1:8000/latest-depth-image'
            )
        )

        self.depth_pub = self.create_publisher(
            Image,
            '/smartphone/depth',
            10
        )

        self.backend_url = self.get_parameter(
            'backend_url'
        ).get_parameter_value().string_value

        self.timer = self.create_timer(
            0.1,
            self.publish_depth
        )

        self.get_logger().info(
            'Smartphone Depth Bridge started'
        )
        self.get_logger().info(
            'Reading depth from FastAPI backend'
        )

    def publish_depth(self):

        try:
            response = requests.get(
                self.backend_url,
                timeout=1.0
            )

            if response.status_code != 200:
                self.get_logger().warning(
                    f'Backend returned {response.status_code}'
                )
                return

            image_array = np.frombuffer(
                response.content,
                dtype=np.uint8
            )

            depth = cv2.imdecode(
                image_array,
                cv2.IMREAD_GRAYSCALE
            )

            if depth is None:
                self.get_logger().warning(
                    'Could not decode depth image'
                )
                return

            msg = Image()

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_depth_frame'

            msg.height = depth.shape[0]
            msg.width = depth.shape[1]

            msg.encoding = 'mono8'
            msg.is_bigendian = 0
            msg.step = depth.shape[1]

            msg.data = depth.tobytes()

            self.depth_pub.publish(msg)

        except Exception as e:
            self.get_logger().warning(
                f'Depth connection error: {e}'
            )


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
