"""Report mapping bridge — assembles the depth-derived occupancy grid.

The FYP report pipeline ends with a 2D occupancy grid published for Nav2.
The Stage 4 grid is produced in the camera frame (it moves with the robot),
so Nav2 costmaps would see unknown cells behind the robot and unstable
obstacles. This node closes the loop the report-aligned way: it transforms
each incoming camera-frame grid (depth -> point cloud -> filtered -> grid)
into the persistent `map` frame using TF and maintains a fixed-size
assembled occupancy map:

* occupied cells are marked where the filtered cloud reported obstacles
* free cells are written where the projection reported free space
  (re-observation of free space clears stale marks)
* never-observed cells remain unknown (-1)

The assembled map is published on ``map_topic`` in the static-layer wire
format so both Nav2 costmaps ingest the smartphone-derived environment
representation directly. Datatypes are standard Nav2 interfaces — no custom
messages are introduced.
"""

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped


class PointcloudCostmapLayerNode(Node):
    """Assemble camera-frame occupancy grids into a persistent map."""

    def __init__(self):
        super().__init__("pointcloud_costmap_layer")
        self.declare_parameter("grid_topic", "/depth_occupancy_grid")
        self.declare_parameter("map_topic", "/smartphone_map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("map_resolution", 0.05)
        self.declare_parameter("map_size_x", 20.0)
        self.declare_parameter("map_size_y", 20.0)
        self.declare_parameter("map_origin_x", -10.0)
        self.declare_parameter("map_origin_y", -10.0)
        self.declare_parameter("publish_rate", 1.0)

        grid_topic = self.get_parameter("grid_topic").value
        self.map_topic = self.get_parameter("map_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.res = float(self.get_parameter("map_resolution").value)
        sx = float(self.get_parameter("map_size_x").value)
        sy = float(self.get_parameter("map_size_y").value)
        ox = float(self.get_parameter("map_origin_x").value)
        oy = float(self.get_parameter("map_origin_y").value)
        rate = float(self.get_parameter("publish_rate").value)

        self.w = int(round(sx / self.res))
        self.h = int(round(sy / self.res))
        self.origin_x = ox
        self.origin_y = oy
        # Persistent assembled map; -1 (unknown) until first observation.
        self.assembled = np.full((self.h, self.w), -1, dtype=np.int8)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, grid_topic, self.grid_callback, 10)
        self.map_timer = self.create_timer(1.0 / max(rate, 0.1), self.publish_map)
        self.get_logger().info(
            f"Assembling {grid_topic} -> {self.map_topic} "
            f"({self.w}x{self.h} @ {self.res} m, frame {self.map_frame})"
        )

    def grid_callback(self, msg):
        """Transform the camera-frame grid into the map and merge it."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                rclpy.time.Time(),  # latest available
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except Exception as exc:
            self.get_logger().warning(f"TF {self.map_frame}<-{msg.header.frame_id}: {exc}", throttle_duration_sec=5.0)
            return

        yaw = self._yaw_from_quat(tf.transform.rotation)
        c, s = np.cos(yaw), np.sin(yaw)
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y

        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        rows, cols = np.nonzero(data != -1)
        if rows.size == 0:
            return
        vals = data[rows, cols]

        # Camera-frame grid cell centres -> map frame (2D rigid transform).
        gx = msg.info.origin.position.x + (cols + 0.5) * msg.info.resolution
        gy = msg.info.origin.position.y + (rows + 0.5) * msg.info.resolution
        mx = tx + c * gx - s * gy
        my = ty + s * gx + c * gy

        acx = np.floor((mx - self.origin_x) / self.res).astype(np.int64)
        acy = np.floor((my - self.origin_y) / self.res).astype(np.int64)
        inb = (acx >= 0) & (acx < self.w) & (acy >= 0) & (acy < self.h)
        if not inb.any():
            return
        occ = vals[inb] == 100
        # Free observations overwrite stale occupied marks; occupied marks
        # overwrite free/unknown. Apply free first, then occupied.
        free_cells = (~occ)
        self.assembled[acy[inb][free_cells], acx[inb][free_cells]] = 0
        self.assembled[acy[inb][occ], acx[inb][occ]] = 100

    @staticmethod
    def _yaw_from_quat(q):
        return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.res
        msg.info.width = self.w
        msg.info.height = self.h
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.assembled.flatten().tolist()
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudCostmapLayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
