from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    pkg_dir = Path(__file__).resolve().parent.parent
    world = pkg_dir / "worlds" / "indoor_world.sdf"
    robot = pkg_dir / "models" / "fyp_robot" / "fyp_robot.sdf"

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", str(world)],
        output="screen"
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen"
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "fyp_robot",
            "-file", str(robot),
            "-x", "0",
            "-y", "0",
            "-z", "0.35"
        ],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        bridge,
        TimerAction(
            period=3.0,
            actions=[spawn_robot]
        )
    ])
