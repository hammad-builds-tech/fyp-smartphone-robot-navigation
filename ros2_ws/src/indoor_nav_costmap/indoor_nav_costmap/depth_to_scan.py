import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import numpy as np


class DepthToScan(Node):

    def __init__(self):
        super().__init__("depth_to_scan")

        self.bridge = CvBridge()

        self.declare_parameter("depth_topic", "/smartphone/depth")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_frame_id", "base_link")
        self.declare_parameter("horizontal_fov_rad", 2.09439510239)
        self.declare_parameter("scan_yaw_offset_rad", 0.0)
        self.declare_parameter("vertical_roi_top", 0.20)
        self.declare_parameter("vertical_roi_bottom", 0.85)
        self.declare_parameter("target_beams", 120)
        self.declare_parameter("min_range", 0.15)
        self.declare_parameter("obstacle_max_range", 3.0)
        self.declare_parameter("clearing_max_range", 3.5)
        self.declare_parameter("inverse_depth", True)

        depth_topic = self.get_parameter("depth_topic").value
        scan_topic = self.get_parameter("scan_topic").value
        self.scan_frame_id = self.get_parameter("scan_frame_id").value
        self.horizontal_fov_rad = float(
            self.get_parameter("horizontal_fov_rad").value
        )
        self.scan_yaw_offset_rad = float(
            self.get_parameter("scan_yaw_offset_rad").value
        )
        self.vertical_roi_top = float(
            self.get_parameter("vertical_roi_top").value
        )
        self.vertical_roi_bottom = float(
            self.get_parameter("vertical_roi_bottom").value
        )
        self.target_beams = int(self.get_parameter("target_beams").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.obstacle_max_range = float(
            self.get_parameter("obstacle_max_range").value
        )
        self.clearing_max_range = float(
            self.get_parameter("clearing_max_range").value
        )
        self.inverse_depth = bool(self.get_parameter("inverse_depth").value)

        if not (
            0.0 <= self.vertical_roi_top < self.vertical_roi_bottom <= 1.0
        ):
            raise ValueError("vertical ROI fractions must satisfy 0 <= top < bottom <= 1")
        if self.horizontal_fov_rad <= 0.0 or self.target_beams < 2:
            raise ValueError("horizontal FOV must be positive and target_beams must be at least 2")
        if not (
            0.0 < self.min_range < self.obstacle_max_range < self.clearing_max_range
        ):
            raise ValueError(
                "range limits must satisfy 0 < min < obstacle_max < clearing_max"
            )

        self.sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            qos_profile_sensor_data,
        )

        self.pub = self.create_publisher(
            LaserScan,
            scan_topic,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"DepthToScan ready: {depth_topic} -> {scan_topic} "
            f"in {self.scan_frame_id}"
        )

    @staticmethod
    def ranges_from_relative_depth(
        depth,
        vertical_roi_top,
        vertical_roi_bottom,
        target_beams,
        min_range,
        clearing_max_range,
        inverse_depth,
    ):
        """Convert a MiDaS relative-depth image into conservative scan ranges.

        MiDaS returns inverse relative depth: larger values are closer. The
        conversion preserves the nearest value in each output beam so a narrow
        obstacle cannot disappear when the image is downsampled.
        """
        depth = np.asarray(depth, dtype=np.float32)
        if depth.ndim != 2:
            return None

        height, width = depth.shape
        if height < 2 or width < 2:
            return None

        y1 = int(height * vertical_roi_top)
        y2 = int(height * vertical_roi_bottom)
        if y2 <= y1:
            return None

        column_depth = np.nanmedian(depth[y1:y2, :], axis=0)
        valid = column_depth[np.isfinite(column_depth)]
        beam_count = min(target_beams, width)

        if valid.size < 2:
            return np.full(beam_count, clearing_max_range, dtype=np.float32)

        low = float(np.percentile(valid, 5))
        high = float(np.percentile(valid, 95))
        if high - low < 1e-6:
            return np.full(beam_count, clearing_max_range, dtype=np.float32)

        normalized = np.clip((column_depth - low) / (high - low), 0.0, 1.0)
        if inverse_depth:
            ranges = clearing_max_range - normalized * (
                clearing_max_range - min_range
            )
        else:
            ranges = min_range + normalized * (
                clearing_max_range - min_range
            )
        ranges[~np.isfinite(ranges)] = clearing_max_range

        # Keep the nearest range in each angular bucket to avoid missing a
        # thin obstacle between evenly spaced image samples.
        edges = np.linspace(0, width, beam_count + 1, dtype=np.int32)
        downsampled = np.empty(beam_count, dtype=np.float32)
        for index in range(beam_count):
            start, end = edges[index], edges[index + 1]
            downsampled[index] = np.min(ranges[start:end])

        return np.clip(downsampled, min_range, clearing_max_range)

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )
            ranges = self.ranges_from_relative_depth(
                depth,
                self.vertical_roi_top,
                self.vertical_roi_bottom,
                self.target_beams,
                self.min_range,
                self.clearing_max_range,
                self.inverse_depth,
            )
            if ranges is None:
                self.get_logger().warning("Ignoring malformed MiDaS depth image")
                return

            scan = LaserScan()
            if msg.header.stamp.sec or msg.header.stamp.nanosec:
                scan.header.stamp = msg.header.stamp
            else:
                scan.header.stamp = self.get_clock().now().to_msg()
            scan.header.frame_id = self.scan_frame_id
            scan.angle_min = (
                self.scan_yaw_offset_rad - self.horizontal_fov_rad / 2.0
            )
            scan.angle_max = (
                self.scan_yaw_offset_rad + self.horizontal_fov_rad / 2.0
            )
            scan.angle_increment = (
                scan.angle_max - scan.angle_min
            ) / (len(ranges) - 1)
            scan.time_increment = 0.0
            scan.scan_time = 0.0
            scan.range_min = self.min_range
            scan.range_max = self.clearing_max_range
            scan.ranges = ranges.tolist()
            self.pub.publish(scan)

        except (ValueError, TypeError) as exc:
            self.get_logger().warning(f"DepthToScan conversion error: {exc}")


def main(args=None):

    rclpy.init(args=args)

    node = DepthToScan()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
