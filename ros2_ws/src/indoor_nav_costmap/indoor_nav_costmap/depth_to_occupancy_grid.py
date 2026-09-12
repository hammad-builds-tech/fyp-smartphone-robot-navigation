import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import numpy as np


class DepthToOccupancyGrid(Node):

    def __init__(self):
        super().__init__('depth_to_occupancy_grid')

        self.bridge = CvBridge()

        self.depth_sub = self.create_subscription(
            Image,
            '/smartphone/depth',
            self.depth_callback,
            10
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/depth_occupancy_grid',
            10
        )

        self.latest_depth = None

        self.get_logger().info('Depth → Occupancy Grid node started')
        self.get_logger().info('Listening: /smartphone/depth')
        self.get_logger().info('Publishing: /depth_occupancy_grid')

    def depth_callback(self, msg):

        try:
            depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='mono8'
            )

            depth = np.asarray(depth, dtype=np.uint8)

            # Downsample image to make a small 2D grid
            small = cv2_resize(depth, 80, 60)

            grid = np.zeros((60, 80), dtype=np.int8)

            # MiDaS depth is relative.
            # Low values = closer in our current representation.
            obstacle_mask = small < 90

            grid[obstacle_mask] = 100

            # Unknown / free area
            grid[~obstacle_mask] = 0

            map_msg = OccupancyGrid()

            map_msg.header.stamp = self.get_clock().now().to_msg()
            map_msg.header.frame_id = 'camera_depth_frame'

            map_msg.info.resolution = 0.05
            map_msg.info.width = 80
            map_msg.info.height = 60

            map_msg.info.origin.position.x = -2.0
            map_msg.info.origin.position.y = -1.5
            map_msg.info.origin.position.z = 0.0

            map_msg.info.origin.orientation.w = 1.0

            map_msg.data = grid.flatten().tolist()

            self.map_pub.publish(map_msg)

        except Exception as e:
            self.get_logger().warning(
                f'Occupancy grid error: {e}'
            )


def cv2_resize(image, width, height):

    import cv2

    return cv2.resize(
        image,
        (width, height),
        interpolation=cv2.INTER_AREA
    )


def main(args=None):

    rclpy.init(args=args)

    node = DepthToOccupancyGrid()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
