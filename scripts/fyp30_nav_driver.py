#!/usr/bin/env python3
"""FYP realroom30 offline navigation driver.

Modes:
  validate X Y   plan robot->(X,Y) via /compute_path_to_pose, report length
  navigate X Y   send /navigate_to_pose, monitor, dump trajectory
  reverse  X Y   plan (X,Y)->robot (reverse plan for U-shape evidence)

Run with /usr/bin/python3 after sourcing /opt/ros/lyrical/setup.bash.
"""
import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener

# Per-dataset trajectory evidence: run_navigation.sh points this at
# <dataset>/trajectory_odom.log so parallel datasets never mix odom segments.
TRAJ_LOG = os.environ.get('FYP_TRAJ_LOG', '/tmp/robot_traj_final_full.log')


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class NavDriver(Node):
    def __init__(self):
        super().__init__('fyp_nav_final')
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.cmd_vel = None
        self.odom = None
        qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Twist, '/cmd_vel', self._cb_vel, qos)
        self.create_subscription(Odometry, '/odom', self._cb_odom, qos)
        self.traj_fh = open(TRAJ_LOG, 'a', buffering=1)
        self.traj_fh.write('# --- run start (wall %s) ---\n' % time.strftime('%Y-%m-%dT%H:%M:%S'))

    def _cb_vel(self, m):
        self.cmd_vel = m

    def _cb_odom(self, m):
        self.odom = m
        p = m.pose.pose.position
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.traj_fh.write('%.2f %.4f %.4f\n' % (t, p.x, p.y))

    def robot_pose(self, budget=40.0):
        """Lookup map->base_link, retrying up to budget seconds."""
        t0 = time.time()
        while time.time() - t0 < budget:
            try:
                tr = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
                p = tr.transform.translation
                q = tr.transform.rotation
                return (p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w))
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.5)
        raise RuntimeError('TF map->base_link unavailable after %.0fs' % budget)

    def wait_plan_client(self, budget=40.0):
        self._plan_cli = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        t0 = time.time()
        while not self._plan_cli.wait_for_server(timeout_sec=2.0):
            if time.time() - t0 > budget:
                raise RuntimeError('compute_path_to_pose action server not up')
            self.get_logger().info('waiting for compute_path_to_pose server...')

    def wait_nav_client(self, budget=40.0):
        self._nav_cli = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        t0 = time.time()
        while not self._nav_cli.wait_for_server(timeout_sec=2.0):
            if time.time() - t0 > budget:
                raise RuntimeError('navigate_to_pose action server not up')
            self.get_logger().info('waiting for navigate_to_pose server...')

    def plan_from(self, start, goal):
        """ComputePathToPose; start/goal are (x, y, yaw) tuples. Returns (poses, length)."""
        g = ComputePathToPose.Goal()
        g.planner_id = 'GridBased'
        g.goal = NavigateToPose.Goal().pose  # reuse PoseStamped default
        ps = g.goal
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(goal[0])
        ps.pose.position.y = float(goal[1])
        ps.pose.orientation.z = math.sin(goal[2] / 2.0)
        ps.pose.orientation.w = math.cos(goal[2] / 2.0)
        if start is not None:
            g.start = NavigateToPose.Goal().pose
            g.start.header.frame_id = 'map'
            g.start.pose.position.x = float(start[0])
            g.start.pose.position.y = float(start[1])
            g.start.pose.orientation.z = math.sin(start[2] / 2.0)
            g.start.pose.orientation.w = math.cos(start[2] / 2.0)
        fut = self._plan_cli.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return None, None
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=120.0)
        res = rf.result()
        if res is None or res.status != 4:
            return None, None
        poses = res.result.path.poses
        if not poses:
            return None, None
        length = 0.0
        for a, b in zip(poses, poses[1:]):
            length += math.hypot(b.pose.position.x - a.pose.position.x,
                                 b.pose.position.y - a.pose.position.y)
        return poses, length

    def dump_plan(self, poses, path):
        with open(path, 'w') as fh:
            fh.write('# x y\n')
            for ps in poses:
                fh.write('%.4f %.4f\n' % (ps.pose.position.x, ps.pose.position.y))
        print('plan dumped: %s (%d poses)' % (path, len(poses)))

    def navigate(self, goal, monitor_every=20.0, give_up=600.0):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(goal[0])
        g.pose.pose.position.y = float(goal[1])
        g.pose.pose.orientation.z = math.sin(goal[2] / 2.0)
        g.pose.pose.orientation.w = math.cos(goal[2] / 2.0)
        print('NAV_GOAL: (%.3f, %.3f, yaw %.2f)' % (goal[0], goal[1], goal[2]))
        fut = self._nav_cli.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print('NAV_FAIL: goal rejected')
            return False
        t0 = time.time()
        next_print = monitor_every
        rf = gh.get_result_async()
        while not rf.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            el = time.time() - t0
            if el >= next_print:
                v = 0.0
                if self.cmd_vel is not None:
                    v = math.hypot(self.cmd_vel.linear.x, self.cmd_vel.linear.y)
                try:
                    rp = self.robot_pose(budget=2.0)
                    print('t=%5.0fs vel=%.3f m/s pos=(%.3f, %.3f)' % (el, v, rp[0], rp[1]))
                except Exception:
                    print('t=%5.0fs vel=%.3f m/s pos=(TF busy)' % (el, v))
                next_print += monitor_every
            if el > give_up:
                print('NAV_TIMEOUT after %.0fs, cancelling' % el)
                gh.cancel_goal()
                rclpy.spin_until_future_complete(self, rf, timeout_sec=15.0)
                return False
        res = rf.result()
        status = res.status if res is not None else -1
        print('NAV_RESULT_STATUS: %s' % status)
        if status != 4:
            return False
        rp = self.robot_pose(budget=10.0)
        print('REACHED the goal! final_pose=(%.3f, %.3f, yaw %.2f)' % rp)
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['validate', 'navigate', 'reverse'])
    ap.add_argument('x', type=float)
    ap.add_argument('y', type=float)
    args = ap.parse_args()

    rclpy.init()
    n = NavDriver()
    goal = (args.x, args.y, 0.0)

    if args.mode == 'navigate':
        n.wait_nav_client()
        rp = n.robot_pose()
        print('START pose: (%.3f, %.3f, yaw %.2f)' % rp)
        ok = n.navigate(goal)
        print('OVERALL: %s' % ('PASS' if ok else 'FAIL'))
        n.traj_fh.close()
        sys.exit(0 if ok else 1)

    n.wait_plan_client()
    rp = n.robot_pose()
    if args.mode == 'validate':
        poses, length = n.plan_from(rp, goal)
        if poses is None:
            print('PLAN_FAIL')
            sys.exit(1)
        straight = math.hypot(args.x - rp[0], args.y - rp[1])
        print('PLAN_OK poses=%d len=%.2f m straight=%.2f m ratio=%.2f' %
              (len(poses), length, straight, length / max(straight, 1e-6)))
        n.dump_plan(poses, '/tmp/fyp30_plan.log')
    else:  # reverse: from goal back to robot
        poses, length = n.plan_from(goal, rp)
        if poses is None:
            print('PLAN_FAIL')
            sys.exit(1)
        straight = math.hypot(args.x - rp[0], args.y - rp[1])
        print('REVERSE_PLAN_OK poses=%d len=%.2f m straight=%.2f m ratio=%.2f' %
              (len(poses), length, straight, length / max(straight, 1e-6)))
        n.dump_plan(poses, '/tmp/fyp30_plan_reverse.log')
    n.traj_fh.close()


if __name__ == '__main__':
    main()
