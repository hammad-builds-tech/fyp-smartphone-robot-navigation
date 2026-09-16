import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np


class DepthObstacleNode(Node):

    def __init__(self):
        super().__init__("depth_obstacle_node")

        self.bridge = CvBridge()
        self.obstacle_depth = None

        self.depth_sub = self.create_subscription(
            Image,
            "/smartphone/depth",
            self.depth_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        self.timer = self.create_timer(
            0.2,
            self.control_robot,
        )

        self.get_logger().info(
            "Depth obstacle safety node started"
        )

    def depth_callback(self, msg):

        try:
            depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="mono8",
            )

            depth = np.asarray(
                depth,
                dtype=np.float32,
            )

            h, w = depth.shape

            y1 = int(h * 0.30)
            y2 = int(h * 0.75)
            x1 = int(w * 0.30)
            x2 = int(w * 0.70)

            center = depth[y1:y2, x1:x2]

            valid = center[np.isfinite(center)]

            if valid.size:
                self.obstacle_depth = float(
                    np.percentile(valid, 10)
                )

        except Exception as e:
            self.get_logger().warning(
                f"Depth processing error: {e}"
            )

    def control_robot(self):

        # Safety node only.
        # Do not fight Nav2 by publishing normal driving commands.
        if self.obstacle_depth is None:
            return

        if self.obstacle_depth < 45:

            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

            self.get_logger().warning(
                "Very close depth obstacle: STOP"
            )


def main(args=None):

    rclpy.init(args=args)

    node = DepthObstacleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
