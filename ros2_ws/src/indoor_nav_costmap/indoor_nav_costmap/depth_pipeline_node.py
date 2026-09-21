"""Report Stage 3 + Stage 4 — depth -> 3D point cloud -> filtered 2D map.

Implements the processing model defined in the FYP report
("SMARTPHONE-BASED REAL-TIME 3D INDOOR MAPPING AND AUTONOMOUS NAVIGATION SYSTEM"):

Stage 3 — 3D Point Cloud Generation
    For every pixel (u, v) with depth d:
        X = (u - cx) * d / fx
        Y = (v - cy) * d / fy
        Z = d
    using the camera intrinsics supplied as parameters.

Stage 4 — Point Cloud Filtering and 2D Map Generation
    1. voxel downsampling
    2. pass-through filtering (out-of-range points removed)
    3. statistical outlier removal
    4. RANSAC floor removal
    followed by projection onto a 2D occupancy grid published as
    nav_msgs/OccupancyGrid.

MiDaS produces *relative* depth. As an implementation treatment (documented,
not native metric depth), the relative depth is normalized by anchoring the
5th-percentile (farthest) surface at ``far_range`` so the projected cloud
covers a bounded, conservative workspace.

Publications:
    /camera/depth/points   sensor_msgs/PointCloud2  (filtered cloud)
    /depth_occupancy_grid  nav_msgs/OccupancyGrid   (camera-frame projection)
"""

import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField


def voxel_downsample(points, voxel_size):
    """Step 1: voxel downsampling (mean point per occupied voxel)."""
    if points.shape[0] == 0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, inverse, counts = np.unique(
        keys, axis=0, return_inverse=True, return_counts=True
    )
    sums = np.zeros((counts.shape[0], 3), dtype=np.float64)
    np.add.at(sums, inverse, points)
    return (sums / counts[:, None]).astype(np.float32)


def passthrough_filter(points, z_min, z_max):
    """Step 2: pass-through filtering — keep points within the depth range."""
    if points.shape[0] == 0:
        return points
    mask = (
        np.isfinite(points).all(axis=1)
        & (points[:, 2] >= z_min)
        & (points[:, 2] <= z_max)
    )
    return points[mask]


def statistical_outlier_removal(points, mean_k=16, std_ratio=2.0):
    """Step 3: statistical outlier removal (PCL semantics).

    Removes points whose mean distance to their k nearest neighbours exceeds
    the global mean distance plus ``std_ratio`` standard deviations.
    """
    n = points.shape[0]
    if n == 0:
        return points
    k = min(mean_k, n - 1)
    if k < 1:
        return points
    # Chunked brute-force kNN: n is small after voxel downsampling (< 5k).
    mean_dists = np.empty(n, dtype=np.float32)
    chunk = 512
    for start in range(0, n, chunk):
        block = points[start : start + chunk]
        d2 = (
            (block[:, None, :] - points[None, :, :]) ** 2
        ).sum(axis=2)
        np.fill_diagonal(d2[:, start : start + chunk], np.inf)
        if k < n:
            part = np.partition(d2, k, axis=1)[:, :k]
        else:
            part = d2
        mean_dists[start : start + chunk] = np.sqrt(part.mean(axis=1))
    threshold = mean_dists.mean() + std_ratio * mean_dists.std()
    return points[mean_dists <= threshold]


def ransac_floor_removal(points, distance_threshold=0.04, max_iterations=2):
    """Step 4: RANSAC floor removal.

    Fits a dominant plane to points sampled from the lowest depth band and
    removes its inliers when the plane is floor-like (normal predominantly
    along the camera's vertical axis and offset consistent with a floor).
    """
    if points.shape[0] < 50:
        return points
    z_low = np.percentile(points[:, 2], 40)
    candidates = points[points[:, 2] <= max(z_low, 1.0)]
    if candidates.shape[0] < 30:
        return points

    best_inliers = None
    best_count = 0
    rng = np.random.default_rng(7)
    for _ in range(max_iterations):
        sample = candidates[rng.choice(candidates.shape[0], 3, replace=False)]
        v1 = sample[1] - sample[0]
        v2 = sample[2] - sample[0]
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        d = -np.dot(normal, sample[0])
        dists = np.abs(points @ normal + d)
        inliers = dists < distance_threshold
        # Floor-like plane: normal mostly vertical (camera y is down) and
        # offset below the camera.
        if abs(normal[1]) < 0.85 or d > 0.6:
            continue
        if inliers.sum() > best_count:
            best_count = int(inliers.sum())
            best_inliers = inliers
    if best_inliers is None:
        return points
    return points[~best_inliers]


class DepthPipelineNode(Node):
    """Report Stage 3 + Stage 4 as one ROS 2 node."""

    def __init__(self):
        super().__init__("depth_pipeline_node")
        self.bridge = CvBridge()

        self.declare_parameter("depth_topic", "/smartphone/depth")
        self.declare_parameter("pointcloud_topic", "/camera/depth/points")
        self.declare_parameter("grid_topic", "/depth_occupancy_grid")
        self.declare_parameter("frame_id", "camera_depth_frame")
        # Camera intrinsics (pinhole model from the report's Stage 3).
        self.declare_parameter("fx", 554.25)
        self.declare_parameter("fy", 554.25)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("stride", 4)
        # Stage 4 filter settings.
        self.declare_parameter("voxel_size", 0.10)
        self.declare_parameter("z_min", 0.5)
        self.declare_parameter("z_max", 4.0)
        self.declare_parameter("sor_mean_k", 16)
        self.declare_parameter("sor_std_ratio", 2.0)
        self.declare_parameter("ransac_distance_threshold", 0.04)
        # Relative-depth treatment: farthest 5th-percentile surface anchors
        # at far_range metres (documented MiDaS normalization).
        self.declare_parameter("far_range", 4.0)
        # Occupancy grid geometry: lateral x in [-x_size/2, x_size/2],
        # forward z in [0, y_size].
        self.declare_parameter("grid_resolution", 0.05)
        self.declare_parameter("grid_x_size", 12.0)
        self.declare_parameter("grid_y_size", 6.0)
        self.declare_parameter("free_radius", 2.0)
        self.declare_parameter("obstacle_height_min", -0.15)
        self.declare_parameter("obstacle_height_max", 0.60)

        def p(name):
            return self.get_parameter(name).value

        self.depth_topic = p("depth_topic")
        self.frame_id = p("frame_id")
        self.fx = float(p("fx"))
        self.fy = float(p("fy"))
        self.cx = float(p("cx"))
        self.cy = float(p("cy"))
        self.stride = int(p("stride"))
        self.voxel_size = float(p("voxel_size"))
        self.z_min = float(p("z_min"))
        self.z_max = float(p("z_max"))
        self.sor_mean_k = int(p("sor_mean_k"))
        self.sor_std_ratio = float(p("sor_std_ratio"))
        self.ransac_threshold = float(p("ransac_distance_threshold"))
        self.far_range = float(p("far_range"))
        self.res = float(p("grid_resolution"))
        self.grid_x_size = float(p("grid_x_size"))
        self.grid_y_size = float(p("grid_y_size"))
        self.free_radius = float(p("free_radius"))
        self.h_min = float(p("obstacle_height_min"))
        self.h_max = float(p("obstacle_height_max"))

        self.grid_w = int(round(self.grid_x_size / self.res))
        self.grid_h = int(round(self.grid_y_size / self.res))

        self.cloud_pub = self.create_publisher(
            PointCloud2, p("pointcloud_topic"), qos_profile_sensor_data
        )
        self.grid_pub = self.create_publisher(
            OccupancyGrid, p("grid_topic"), 10
        )
        self.sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Depth pipeline ready: {self.depth_topic} -> "
            f"{p('pointcloud_topic')} + {p('grid_topic')} (report Stage 3+4)"
        )

    def relative_to_metric(self, depth):
        """Normalize relative MiDaS depth into conservative pseudo-metric Z."""
        valid = depth[np.isfinite(depth) & (depth > 0)]
        if valid.size < 100:
            return None
        d_far = float(np.percentile(valid, 5))
        if d_far <= 1e-6:
            return None
        # MiDaS value is inverse-depth-like: smaller value = farther.
        z = self.far_range * (d_far / np.maximum(depth, 1e-6))
        return np.clip(z, self.z_min, self.far_range).astype(np.float32)

    def make_point_cloud(self, depth):
        """Report Stage 3: pinhole unprojection with the camera intrinsics."""
        stride = self.stride
        h, w = depth.shape
        vs, us = np.mgrid[0:h:stride, 0:w:stride]
        zs = depth[::stride, ::stride].astype(np.float32)
        finite = np.isfinite(zs)
        xs = ((us - self.cx) * zs / self.fx).astype(np.float32)
        ys = ((vs - self.cy) * zs / self.fy).astype(np.float32)
        pts = np.stack([xs[finite], ys[finite], zs[finite]], axis=1)
        return pts

    def filter_cloud(self, points):
        """Report Stage 4: voxel -> passthrough -> SOR -> RANSAC floor."""
        points = voxel_downsample(points, self.voxel_size)
        points = passthrough_filter(points, self.z_min, self.z_max)
        points = statistical_outlier_removal(
            points, self.sor_mean_k, self.sor_std_ratio
        )
        points = ransac_floor_removal(points, self.ransac_threshold)
        return points

    def project_grid(self, points):
        """Project the filtered cloud to a camera-frame occupancy grid.

        Cells are free from z_min up to the nearest obstacle along each image
        column, occupied at the obstacle, and unknown beyond (camera FOV is
        forward-looking; unobserved space stays unknown, never fake-free).
        """
        grid = np.full((self.grid_h, self.grid_w), -1, dtype=np.int8)
        if points.shape[0] == 0:
            return grid

        gx = np.floor((points[:, 0] + self.grid_x_size / 2.0) / self.res).astype(int)
        gy = np.floor(points[:, 2] / self.res).astype(int)
        inb = (
            (gx >= 0) & (gx < self.grid_w)
            & (gy >= 0) & (gy < self.grid_h)
            & (points[:, 1] >= self.h_min) & (points[:, 1] <= self.h_max)
        )
        occupied = np.zeros((self.grid_h, self.grid_w), dtype=bool)
        occupied[gy[inb], gx[inb]] = True

        # Nearest obstacle range per lateral cell (columns of the grid).
        nearest = np.full(self.grid_w, self.grid_y_size, dtype=np.float32)
        hit_rows, hit_cols = np.nonzero(occupied)
        if hit_cols.size:
            np.minimum.at(nearest, hit_cols, hit_rows * self.res)

        # Free space: every observed lateral column is free from z_min up to
        # its nearest obstacle; observed columns without an obstacle within
        # free_radius are free up to free_radius.
        observed_cols = np.unique(gx[inb]) if inb.any() else np.array([], int)
        col_obs_rows = np.nonzero(occupied.any(axis=0))[0]
        observed_cols = np.union1d(observed_cols, col_obs_rows)
        free_end = np.minimum(nearest - self.res, self.free_radius)
        for col in observed_cols:
            end_row = int(free_end[col] / self.res)
            free_rows = int(max(self.z_min, 0.0) / self.res)
            if end_row > free_rows:
                grid[free_rows:end_row, col] = 0
            if occupied[:, col].any():
                hit_row = hit_rows[hit_cols == col].min()
                grid[hit_row, col] = 100
        return grid

    def publish_cloud(self, points):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = points.shape[0]
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = False
        msg.data = points.astype(np.float32).tobytes()
        self.cloud_pub.publish(msg)

    def publish_grid(self, grid):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = self.res
        msg.info.width = self.grid_w
        msg.info.height = self.grid_h
        msg.info.origin.position.x = -self.grid_x_size / 2.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        self.grid_pub.publish(msg)

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            metric = self.relative_to_metric(depth)
            if metric is None:
                return
            points = self.make_point_cloud(metric)
            points = self.filter_cloud(points)
            self.publish_cloud(points)
            self.publish_grid(self.project_grid(points))
        except Exception as exc:  # never let one bad frame kill the node
            self.get_logger().warning(f"Depth pipeline error: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthPipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
