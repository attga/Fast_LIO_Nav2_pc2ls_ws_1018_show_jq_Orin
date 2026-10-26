#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Patrol log4444 waypoints 12 -> 13 -> ... -> 20 -> 12 as a closed loop.

Waypoints come from round6map/log4444_numbered_points_1p7m.csv. The source
ndt_demo1 route is spaced about 1.7 m; these fixed selected points have
1.60-1.71 m straight gaps, and the closing 20->12 edge is 1.976 m.
The first lap visits waypoint 12 first. Each lap uses separate outbound and
return navigation tasks so no task ends at the same point where it starts.
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


# Heading is the local tangent of the closed path, in map-frame degrees.
LOOP_WAYPOINTS = [
    ( -8.147, -20.455,  178.65, "log4444_wp_012"),
    ( -9.647, -21.275, -129.88, "log4444_wp_013"),
    (-10.165, -22.830,  -97.53, "log4444_wp_014"),
    (-10.065, -24.534,  -68.42, "log4444_wp_015"),
    ( -9.028, -25.779,  -24.28, "log4444_wp_016"),
    ( -7.329, -25.730,   17.49, "log4444_wp_017"),
    ( -5.992, -24.851,   63.80, "log4444_wp_018"),
    ( -6.119, -23.154,   97.91, "log4444_wp_019"),
    ( -6.460, -21.483,  125.09, "log4444_wp_020"),
]


def make_pose(navigator: BasicNavigator,
              x: float, y: float, yaw_deg: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class LoopPatrol(Node):
    """Run each closed lap using Nav2 NavigateThroughPoses."""

    def __init__(self):
        super().__init__("through_poses_patrol_ndt_loop")
        self.declare_parameter("loop_dwell", 0.0)
        self.declare_parameter("max_loops", 0)
        self.declare_parameter("nav_timeout", 6000.0)
        self.loop_dwell = float(self.get_parameter("loop_dwell").value)
        self.max_loops = int(self.get_parameter("max_loops").value)
        self.nav_timeout = float(self.get_parameter("nav_timeout").value)
        if self.loop_dwell < 0 or self.max_loops < 0 or self.nav_timeout <= 0:
            raise ValueError("loop_dwell/max_loops must be nonnegative and nav_timeout positive")
        self.navigator = BasicNavigator()

    def _navigate_once(self, checkpoints: list, lap: int) -> tuple[bool, int, bool]:
        poses = [make_pose(self.navigator, x, y, yaw_deg)
                 for x, y, yaw_deg, _ in checkpoints]
        self.get_logger().info(
            f"Lap {lap}: sending {[label for _, _, _, label in checkpoints]}")
        if not self.navigator.goThroughPoses(poses):
            self.get_logger().error(f"Lap {lap}: Nav2 rejected the navigation task")
            return False, len(poses), True
        start_time = time.monotonic()
        remaining = len(poses)
        while rclpy.ok() and not self.navigator.isTaskComplete():
            if time.monotonic() - start_time > self.nav_timeout:
                self.get_logger().error(f"Lap {lap}: navigation timeout; cancelling")
                self.navigator.cancelTask()
                return False, remaining, True
            feedback = self.navigator.getFeedback()
            if feedback:
                try:
                    remaining = feedback.number_of_poses_remaining
                    self.get_logger().info(
                        f"Lap {lap}: {remaining} poses remaining, "
                        f"{feedback.distance_remaining:.1f} m",
                        throttle_duration_sec=5.0)
                except AttributeError:
                    pass
            time.sleep(0.2)
        if not rclpy.ok():
            return False, remaining, True
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            return True, 0, False
        self.get_logger().error(f"Lap {lap}: task ended with {result}")
        return False, remaining, False

    def _run_lap(self, checkpoints: list, lap: int) -> bool:
        remaining = checkpoints
        while remaining and rclpy.ok():
            success, count, abort = self._navigate_once(remaining, lap)
            if success:
                return True
            if abort or not 0 < count <= len(remaining):
                return False
            remaining = remaining[-count:]
            self.get_logger().warn(
                f"Lap {lap}: retrying the last {count} checkpoints in 5 s")
            time.sleep(5.0)
        return False

    def run_patrol(self):
        self.get_logger().info("Waiting for Nav2 (bt_navigator)...")
        self.navigator._waitForNodeToActivate("bt_navigator")
        self.get_logger().info(
            f"NDT loop: 12 -> 13 -> ... -> 20 -> 12; "
            f"max_loops={self.max_loops or 'infinite'}")
        # With one NavigateThroughPoses task for the whole closed loop, the
        # last goal is waypoint 12 while the robot starts near waypoint 12.
        # Nav2's final-goal check can then succeed before visiting 13..20.
        if not self._run_lap([LOOP_WAYPOINTS[0]], 0):
            self.get_logger().error("Could not reach starting waypoint 12")
            return

        lap = 0
        while rclpy.ok() and (self.max_loops == 0 or lap < self.max_loops):
            lap += 1
            self.get_logger().info(f"Lap {lap}: outbound 12 -> 13 -> ... -> 20")
            if not self._run_lap(LOOP_WAYPOINTS[1:], lap):
                self.get_logger().error("Outbound leg did not complete. Patrol ended.")
                break
            self.get_logger().info(f"Lap {lap}: return 20 -> 12")
            if not self._run_lap([LOOP_WAYPOINTS[0]], lap):
                self.get_logger().error("Return leg did not complete. Patrol ended.")
                break
            self.get_logger().info(f"Lap {lap} complete at waypoint 12")
            if self.loop_dwell > 0:
                time.sleep(self.loop_dwell)


def main():
    rclpy.init()
    node = LoopPatrol()
    try:
        node.run_patrol()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
