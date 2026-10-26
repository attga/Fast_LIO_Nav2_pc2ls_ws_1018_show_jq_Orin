#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_through_poses_robust.py -- Centerline waypoints (with manual chicane override)
=================================================================================
Modified from waypoint_through_poses.py to add robust failure recovery.
If a leg fails (e.g. blocked by crowd), the script tracks the remaining unreached
checkpoints and resumes from the failure point, rather than restarting from the beginning.
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


# ---------------------------------------------------------------------------
# Outbound sparse checkpoints
# ---------------------------------------------------------------------------
OUTBOUND_CHECKPOINTS = [
    (   2.080,  -12.257,    89.1, "center_wp_01"),
    (   2.080,   -7.162,    90.1, "center_wp_02"),
    (   2.080,   -2.036,    90.2, "center_wp_03"),
    (   2.080,    2.821,    90.0, "center_wp_04"),
    (   2.080,    7.944,    89.6, "center_wp_05"),
    (   2.080,   12.792,    87.5, "center_wp_06"),
    (   2.300,   17.775,   116.2, "center_wp_07"),
    (   0.000,   22.449,   145.8, "center_wp_08"),
    (  -4.196,   25.298,   179.4, "center_wp_09"),
    (  -9.400,   25.350,   176.0, "center_wp_10"),
    ( -14.251,   25.689,   179.5, "center_wp_11"),
    ( -19.364,   25.733,   178.3, "center_wp_12"),
    ( -24.210,   25.877,   179.5, "center_wp_13"),
    ( -29.332,   25.920,   179.2, "center_wp_14"),
    ( -34.186,   25.986,   178.5, "center_wp_15"),
    ( -39.312,   26.121,  -176.0, "center_wp_16"),
    ( -44.316,   25.767,   175.2, "center_wp_17"),
    ( -49.500,   26.200,  -143.6, "center_wp_18"),
    ( -55.200,   22.000,  -114.6, "center_wp_19"),
    ( -56.800,   18.500,   -93.4, "center_wp_20"),
    ( -57.200,   11.800,   -88.6, "center_wp_21"),
    ( -57.077,    6.917,   -90.8, "center_wp_22"),
    ( -57.147,    1.793,   -96.4, "center_wp_23"),
    ( -57.676,   -2.940,  -126.1, "center_wp_24"),
    ( -58.310,   -3.810,  -126.1, "center_wp_25"),
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
