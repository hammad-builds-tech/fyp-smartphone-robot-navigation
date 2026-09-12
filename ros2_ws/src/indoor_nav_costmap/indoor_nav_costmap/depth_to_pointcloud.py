import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import struct


class DepthToPointCloud(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/smartphone/depth',
            self.depth_callback,
            10
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/camera/depth/points',
            10
        )

        self.get_logger().info('Depth → PointCloud node started')

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='mono8'
            ).astype(np.float32)

            depth = depth[::8, ::8]

            h, w = depth.shape

            fx = 500.0
            fy = 500.0
            cx = 320.0 / 8.0
            cy = 240.0 / 8.0

            points = []

            for v in range(h):
                for u in range(w):
                    d = depth[v, u]

                    if d < 5 or d > 250:
                        continue

                    z = 1.0 / (d / 255.0)

                    if z > 10.0:
                        continue

                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy

                    points.append((x, y, z))

            msg_out = PointCloud2()

            msg_out.header.stamp = self.get_clock().now().to_msg()
            msg_out.header.frame_id = 'camera_depth_frame'

            msg_out.height = 1
            msg_out.width = len(points)

            msg_out.fields = [
                PointField(
                    name='x',
                    offset=0,
                    datatype=PointField.FLOAT32,
                    count=1
                ),
                PointField(
                    name='y',
                    offset=4,
                    datatype=PointField.FLOAT32,
                    count=1
                ),
                PointField(
                    name='z',
                    offset=8,
                    datatype=PointField.FLOAT32,
                    count=1
                )
            ]

            msg_out.is_bigendian = False
            msg_out.point_step = 12
            msg_out.row_step = msg_out.point_step * msg_out.width
            msg_out.is_dense = False

            data = []

            for x, y, z in points:
                data.append(struct.pack('<fff', x, y, z))

            msg_out.data = b''.join(data)

            if points:
                xs = [p[0] for p in points]
                ys = [p[1] for p in points]
                zs = [p[2] for p in points]

                self.get_logger().info(
                    f'XYZ range: '
                    f'X={min(xs):.2f}..{max(xs):.2f}, '
                    f'Y={min(ys):.2f}..{max(ys):.2f}, '
                    f'Z={min(zs):.2f}..{max(zs):.2f}'
                )

            self.pub.publish(msg_out)

        except Exception as e:
            self.get_logger().warning(
                f'Point cloud error: {e}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = DepthToPointCloud()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
