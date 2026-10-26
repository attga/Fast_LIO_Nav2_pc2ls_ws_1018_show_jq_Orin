#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# Streamlined 18 Key Waypoints (Robust Resume Version)
# Date    : 2026-09-21
# Method  : 18 Key Topological & Geometric Control Waypoints
# Points  : 18 outbound + 18 return (Total route ~389.3m)
# Features: Optimized for Nav2 SmacPlanner2D + SimpleSmoother (1Hz) & MPPI Controller
# Safety  : Incorporates WP175 lawn bypass (0.00% Keepout Mask intrusion)
# ============================================================
"""
waypoint_through_poses_buinding2building_18wps_LIO_SAM.py
=================================================================================
Streamlined 18 key topological waypoints for outdoor building-to-building patrol.
Uses Nav2 NavigateThroughPoses action with robust resume upon failure.
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

# ---------------------------------------------------------------------------
# Outbound checkpoints (~1.50m spacing, 261 points)
# ---------------------------------------------------------------------------

OUTBOUND_CHECKPOINTS = [
    (     -0.416,      0.027,    12.19, "b2b_wp_001"),
    (     48.868,      8.759,   -36.28, "b2b_wp_035"),
    (     60.492,      0.128,    -3.11, "b2b_wp_045"),
    (    120.000,      4.134,   -18.06, "b2b_wp_085"),
    (    125.830,     -0.353,   -53.72, "b2b_wp_090"),
    (    155.212,    -43.699,   -46.77, "b2b_wp_125"),
    (    160.753,    -48.730,   -40.30, "b2b_wp_130"),
    (    167.297,    -52.231,   -14.94, "b2b_wp_135"),
    (    184.821,    -55.966,    11.02, "b2b_wp_147"),
    (    205.438,    -43.689,    39.02, "b2b_wp_165"),
    (    210.634,    -38.464,    72.41, "b2b_wp_170"),
    (    211.024,    -31.282,    42.23, "b2b_wp_175"),
    (    223.966,    -24.112,     5.64, "b2b_wp_185"),
    (    246.368,    -22.684,     5.86, "b2b_wp_200"),
    (    252.868,    -19.499,    56.70, "b2b_wp_205"),
    (    266.362,      7.214,    65.30, "b2b_wp_225"),
    (    282.679,     40.880,    61.78, "b2b_wp_250"),
    (    294.000,     50.450,     9.63, "b2b_wp_261"),
]

RETURN_CHECKPOINTS = [
    (    294.000,     50.450,  -170.37, "ret_b2b_wp_261"),
    (    282.679,     40.880,  -118.22, "ret_b2b_wp_250"),
    (    266.362,      7.214,  -114.70, "ret_b2b_wp_225"),
    (    252.868,    -19.499,  -123.30, "ret_b2b_wp_205"),
    (    246.368,    -22.684,  -174.14, "ret_b2b_wp_200"),
    (    223.966,    -24.112,  -174.36, "ret_b2b_wp_185"),
    (    211.024,    -31.282,  -137.77, "ret_b2b_wp_175"),
    (    210.634,    -38.464,  -107.59, "ret_b2b_wp_170"),
    (    205.438,    -43.689,  -140.98, "ret_b2b_wp_165"),
    (    184.821,    -55.966,  -168.98, "ret_b2b_wp_147"),
    (    167.297,    -52.231,   165.06, "ret_b2b_wp_135"),
    (    160.753,    -48.730,   139.70, "ret_b2b_wp_130"),
    (    155.212,    -43.699,   133.23, "ret_b2b_wp_125"),
    (    125.830,     -0.353,   126.28, "ret_b2b_wp_090"),
    (    120.000,      4.134,   161.94, "ret_b2b_wp_085"),
    (     60.492,      0.128,   176.89, "ret_b2b_wp_045"),
    (     48.868,      8.759,   143.72, "ret_b2b_wp_035"),
    (     -0.416,      0.027,  -167.81, "ret_b2b_wp_001"),
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
        self.declare_parameter("nav_timeout", 6000.0)

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
            f"  NavigateThroughPoses Patrol (Robust Resume Version)\n"
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
    def _navigate_through(self, poses: list, leg: str) -> tuple:
        """Send checkpoints via NavigateThroughPoses and return (success, remaining_count)."""
        self.get_logger().info(
            f"[{leg}] Sending {len(poses)} checkpoints -> goThroughPoses")
        self.navigator.goThroughPoses(poses)

        t0 = time.time()
        last_remaining = len(poses)
        
        while not self.navigator.isTaskComplete():
            elapsed = time.time() - t0
            if elapsed > self.nav_timeout:
                self.get_logger().error(f"[{leg}] Timeout after {self.nav_timeout:.0f}s - cancelling")
                self.navigator.cancelTask()
                return False, last_remaining
                
            fb = self.navigator.getFeedback()
            if fb:
                try:
                    last_remaining = fb.number_of_poses_remaining
                    self.get_logger().info(
                        f"[{leg}] poses_remaining={last_remaining}"
                        f"  dist={fb.distance_remaining:.1f}m",
                        throttle_duration_sec=5.0)
                except AttributeError:
                    pass
            time.sleep(0.2)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"[{leg}] SUCCESS")
            return True, 0
        elif result == TaskResult.CANCELED:
            self.get_logger().warn(f"[{leg}] CANCELED")
            return False, last_remaining
        else:
            self.get_logger().error(f"[{leg}] FAILED (result={result})")
            return False, last_remaining

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

            # ======= Outbound =======
            remaining_outbound = [
                make_pose(self.navigator, x, y, yd)
                for x, y, yd, _ in OUTBOUND_CHECKPOINTS[1:]
            ]
            
            while remaining_outbound and rclpy.ok():
                success, left = self._navigate_through(remaining_outbound, "outbound")
                if success:
                    break
                else:
                    self.get_logger().error(f"Outbound failed with {left} points left. Wait 5s and resume...")
                    time.sleep(5.0)
                    if 0 < left <= len(remaining_outbound):
                        remaining_outbound = remaining_outbound[-left:]
                    else:
                        break
                        
            self.get_logger().info(f"Outbound done. Dwelling {self.end_dwell:.0f}s")
            time.sleep(self.end_dwell)

            # ======= Return =======
            remaining_return = [
                make_pose(self.navigator, x, y, yd)
                for x, y, yd, _ in RETURN_CHECKPOINTS[1:]
            ]
            
            while remaining_return and rclpy.ok():
                success, left = self._navigate_through(remaining_return, "return")
                if success:
                    break
                else:
                    self.get_logger().error(f"Return failed with {left} points left. Wait 5s and resume...")
                    time.sleep(5.0)
                    if 0 < left <= len(remaining_return):
                        remaining_return = remaining_return[-left:]
                    else:
                        break
                        
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
