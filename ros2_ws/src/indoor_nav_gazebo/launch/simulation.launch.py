"""Phase 1 — Gazebo simulation.

Starts the indoor world, the Gazebo <-> ROS bridge and spawns the robot.
Readiness gate for this phase (checked by run_system.sh): /odom and /clock
flowing, robot visible in /odom at the spawn pose.
"""

from pathlib import Path
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Pin Gazebo Transport to loopback. Without this, gz transport can pick a
    # DOWN interface (e.g. unplugged ethernet) for its discovery multicast and
    # silently drop every gz->ROS message (no /clock, /odom, /tf).
    os.environ["GZ_IP"] = "127.0.0.1"

    pkg_dir = Path(__file__).resolve().parent.parent

    world = pkg_dir / "worlds" / "indoor_world.sdf"
    robot = pkg_dir / "models" / "fyp_robot" / "fyp_robot.sdf"

    gazebo = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-r",
            "-s",
            str(world),
        ],
        additional_env={"GZ_IP": "127.0.0.1"},
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "fyp_robot",
            "-file",
            str(robot),
            # Spawn in open floor (obstacle_1 is 0.5 m from the world origin,
            # too close for a start pose with robot_radius 0.40). z=0.05 puts
            # the wheel bottoms (0.28 below base_link origin, model pose +0.25)
            # just above the floor so the robot settles without a hard drop.
            "-x",
            "0",
            "-y",
            "-2",
            "-z",
            "0.05",
        ],
        output="screen",
    )

    # base_link -> camera_depth_frame (the virtual smartphone mount).
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_static",
        arguments=[
            "--x", "0.3",
            "--y", "0.0",
            "--z", "0.2",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "camera_depth_frame",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            bridge,
            camera_tf,
            spawn_robot,
        ]
    )
