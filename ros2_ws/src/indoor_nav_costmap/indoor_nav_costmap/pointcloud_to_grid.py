import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class PointCloudToGrid(Node):
    def __init__(self):
        super().__init__('pointcloud_to_grid')

        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            10
        )

        self.get_logger().info(
            'PointCloud -> XZ Occupancy Grid started'
        )

    def callback(self, msg):
        try:
            grid = np.zeros((120, 120), dtype=np.int8)

            resolution = 0.05

            count = 0

            for p in pc2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            ):
                x = float(p[0])
                y = float(p[1])
                z = float(p[2])

                if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(z):
                    continue

                # Forward distance
                if z < 1.0 or z > 6.0:
                    continue

                # Keep points around camera/obstacle height
                if y < -0.5 or y > 0.1:
                    continue

                # X: left/right
                if x < -3.0 or x > 3.0:
                    continue

                # Z: forward
                if z < 0.0 or z > 6.0:
                    continue

                gx = int((x + 3.0) / resolution)
                gy = int(z / resolution)

                if 0 <= gx < 120 and 0 <= gy < 120:
                    grid[gy, gx] = 100
                    count += 1

            map_msg = OccupancyGrid()

            map_msg.header.stamp = self.get_clock().now().to_msg()
            map_msg.header.frame_id = 'camera_depth_frame'

            map_msg.info.resolution = resolution
            map_msg.info.width = 120
            map_msg.info.height = 120

            map_msg.info.origin.position.x = -3.0
            map_msg.info.origin.position.y = 0.0
            map_msg.info.origin.position.z = 0.0

            map_msg.info.origin.orientation.w = 1.0

            map_msg.data = grid.flatten().tolist()

            self.pub.publish(map_msg)

            self.get_logger().info(
                f'Occupied points: {count}'
            )

        except Exception as e:
            self.get_logger().warning(
                f'Map conversion error: {e}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = PointCloudToGrid()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
