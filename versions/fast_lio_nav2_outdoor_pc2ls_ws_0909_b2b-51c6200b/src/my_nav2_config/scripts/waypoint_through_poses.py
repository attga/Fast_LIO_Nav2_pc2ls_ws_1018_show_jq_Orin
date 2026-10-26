#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_through_poses.py -- NavigateThroughPoses sparse-checkpoint architecture
=================================================================================
New architecture: replaces 80 dense interpolated waypoints with 8 sparse
checkpoints sent via Nav2's NavigateThroughPoses action.

Key improvements vs waypoint_patrol.py:
  1. Intermediate points are NOT precise goals - the global planner finds a
     continuous path THROUGH them. No stop-start at every waypoint.
  2. Planner respects costmap - curves never interpolate into obstacle zones.
  3. Dynamic obstacles handled by MPPI locally; on failure: safe wait, not spin.

Tunable parameters:
  end_dwell    : dwell time at each endpoint (s, default 5.0)
  max_loops    : total patrol loops, 0=infinite (default 0)
  nav_timeout  : per-leg timeout (s, default 180)
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


# ---------------------------------------------------------------------------
# Outbound sparse checkpoints  (x, y, yaw_deg, label)
# Strategy: 1 point per straight segment, 1 point per curve (outer arc).
# The global planner finds the actual path - no dense interpolation needed.
#
# NOTE: CP-01 (2.134, 2.42) = actual ICP-locked parking position (test04/05/06).
#       CP-02 (2.134, 5.44) = transition point, ensures smooth spline exit.
#       CP-03 (2.134, 8.458) = original log-start position (robot already moving).
# ---------------------------------------------------------------------------
OUTBOUND_CHECKPOINTS = [
    ( 2.134,   2.420,   91.0, "parking_start"),      # 实际停车位（ICP锁定）
    ( 2.134,   5.440,   91.0, "east_transition"),     # 过渡点
    ( 2.134,   8.458,   91.0, "start_east"),
    ( 1.904,  19.923,   91.0, "east_road_mid"),
    (-5.769,  26.665,  165.0, "ne_corner_exit"),
    (-28.703, 26.930,  180.0, "north_road_mid"),
    (-51.530, 27.000,  180.0, "nw_corner_outer_arc"),
    (-57.161, 17.119, -100.0, "nw_corner_exit"),
    (-58.268,  5.651,  -90.0, "west_road_mid"),
    (-58.090, -11.493, -90.0, "west_end"),
]

# Return checkpoints: reverse of outbound with flipped yaw
RETURN_CHECKPOINTS = [
    (x, y, (yaw + 180.0) % 360.0 - 180.0, "ret_" + label)
    for x, y, yaw, label in reversed(OUTBOUND_CHECKPOINTS)
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


class ThroughPosesPatrol(Node):
    """NavigateThroughPoses sparse-checkpoint patrol controller."""

    def __init__(self):
        super().__init__("through_poses_patrol")

        self.declare_parameter("end_dwell",   5.0)
        self.declare_parameter("max_loops",   0)
        self.declare_parameter("nav_timeout", 600.0)

        self.end_dwell   = self.get_parameter("end_dwell").value
        self.max_loops   = self.get_parameter("max_loops").value
        self.nav_timeout = self.get_parameter("nav_timeout").value

        self.navigator = BasicNavigator()
        self._log_checkpoints()

    # ------------------------------------------------------------------
    def _log_checkpoints(self):
        sep = "=" * 64
        self.get_logger().info(
            f"\n{sep}\n"
            f"  NavigateThroughPoses Patrol\n"
            f"  Outbound checkpoints : {len(OUTBOUND_CHECKPOINTS)}\n"
            f"  Return  checkpoints  : {len(RETURN_CHECKPOINTS)}\n"
            f"  End dwell  : {self.end_dwell:.1f} s\n"
            f"  Nav timeout: {self.nav_timeout:.0f} s\n"
            f"  Max loops  : {'infinite' if self.max_loops == 0 else self.max_loops}\n"
            f"{sep}"
        )
        for i, (x, y, yd, lbl) in enumerate(OUTBOUND_CHECKPOINTS):
            self.get_logger().info(
                f"  OUT CP-{i+1:02d}: ({x:8.3f}, {y:8.3f})  yaw={yd:7.1f}  [{lbl}]")
        for i, (x, y, yd, lbl) in enumerate(RETURN_CHECKPOINTS):
            self.get_logger().info(
                f"  RET CP-{i+1:02d}: ({x:8.3f}, {y:8.3f})  yaw={yd:7.1f}  [{lbl}]")

    # ------------------------------------------------------------------
    def _navigate_through(self, checkpoints: list, leg: str) -> bool:
        """Send all checkpoints at once via NavigateThroughPoses.

        Nav2 global planner creates ONE continuous path through all via-points.
        Robot does not stop at each point - smooth, continuous motion.
        """
        # Skip index 0 (robot's current start position)
        poses = [
            make_pose(self.navigator, x, y, yd)
            for x, y, yd, _ in checkpoints[1:]
        ]
        self.get_logger().info(
            f"[{leg}] Sending {len(poses)} checkpoints -> goThroughPoses")
        self.navigator.goThroughPoses(poses)

        t0 = time.time()
        while not self.navigator.isTaskComplete():
            elapsed = time.time() - t0
            if elapsed > self.nav_timeout:
                self.get_logger().error(
                    f"[{leg}] Timeout after {self.nav_timeout:.0f}s - cancelling")
                self.navigator.cancelTask()
                return False
            fb = self.navigator.getFeedback()
            if fb:
                try:
                    self.get_logger().info(
                        f"[{leg}] poses_remaining={fb.number_of_poses_remaining}"
                        f"  dist={fb.distance_remaining:.1f}m",
                        throttle_duration_sec=5.0)
                except AttributeError:
                    pass
            time.sleep(0.2)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"[{leg}] SUCCESS")
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().warn(f"[{leg}] CANCELED")
            return False
        else:
            self.get_logger().error(f"[{leg}] FAILED (result={result})")
            return False

    # ------------------------------------------------------------------
    def run_patrol(self):
        self.get_logger().info("Waiting for Nav2 (bt_navigator)...")
        self.navigator._waitForNodeToActivate("bt_navigator")
        self.get_logger().info("Nav2 active. Starting NavigateThroughPoses patrol.")

        loop = 0
        while rclpy.ok():
            loop += 1
            if self.max_loops > 0 and loop > self.max_loops:
                self.get_logger().info(f"Completed {self.max_loops} loops. Done.")
                break
            sep = "=" * 50
            self.get_logger().info(f"\n{sep}\n  Loop #{loop} start\n{sep}")

            # Outbound
            if not self._navigate_through(OUTBOUND_CHECKPOINTS, "outbound"):
                self.get_logger().error("Outbound failed. Retry in 5s...")
                time.sleep(5.0)
                continue
            self.get_logger().info(f"Outbound done. Dwelling {self.end_dwell:.0f}s")
            time.sleep(self.end_dwell)

            # Return
            if not self._navigate_through(RETURN_CHECKPOINTS, "return"):
                self.get_logger().error("Return failed. Retry in 5s...")
                time.sleep(5.0)
                continue
            self.get_logger().info(f"Return done. Dwelling {self.end_dwell:.0f}s")
            time.sleep(self.end_dwell)

            self.get_logger().info(f"Loop #{loop} complete.")

        self.get_logger().info("Patrol ended.")


# ---------------------------------------------------------------------------
def main():
    rclpy.init()
    node = ThroughPosesPatrol()
    try:
        node.run_patrol()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
