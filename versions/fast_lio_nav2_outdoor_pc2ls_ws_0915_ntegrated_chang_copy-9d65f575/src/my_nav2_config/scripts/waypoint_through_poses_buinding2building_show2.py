#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Repeatedly patrol the recorded show2 route from the 20:41:11 log.

Waypoints follow the outbound recording at the demo2 spacing of 1.69625 m.
The final interval is shorter so the patrol reaches the recorded endpoint.
Start the robot near waypoint 0. The return uses the same path in reverse.
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


# Approximate start/end positions were picked from the recorded trajectory.
# Each leg skips its first pose, which is the expected current location.
OUTBOUND_CHECKPOINTS = [
    (  -1.837,  -22.971, -129.18, "show2_wp_000"),
    (  -2.908,  -24.286, -148.53, "show2_wp_001"),
    (  -4.355,  -25.171, -149.26, "show2_wp_002"),
    (  -5.813,  -26.038, -149.91, "show2_wp_003"),
    (  -7.281,  -26.889, -151.01, "show2_wp_004"),
    (  -8.764,  -27.711, -154.80, "show2_wp_005"),
    ( -10.299,  -28.433, -156.48, "show2_wp_006"),
    ( -11.363,  -28.896, -156.48, "show2_wp_007"),
]
RETURN_CHECKPOINTS = [
    (x, y, (yaw_deg % 360.0) - 180.0, f"ret_{label}")
    for x, y, yaw_deg, label in reversed(OUTBOUND_CHECKPOINTS)
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
        super().__init__("through_poses_patrol_show2")

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
            f"  NavigateThroughPoses show2 Recorded-Route Round Trip\n"
            f"  Outbound checkpoints: {len(OUTBOUND_CHECKPOINTS)}\n"
            f"  Return checkpoints  : {len(RETURN_CHECKPOINTS)}\n"
            f"  End dwell           : {self.end_dwell:.1f} s\n"
            f"  Nav timeout         : {self.nav_timeout:.0f} s\n"
            f"  Max round trips     : {'infinite' if self.max_loops == 0 else self.max_loops}\n"
            f"{sep}"
        )
        for leg, checkpoints in (("OUT", OUTBOUND_CHECKPOINTS),
                                 ("RET", RETURN_CHECKPOINTS)):
            for i, (x, y, yaw_deg, label) in enumerate(checkpoints):
                self.get_logger().info(
                    f"  {leg} CP-{i:02d}: ({x:8.3f}, {y:8.3f})"
                    f"  yaw={yaw_deg:7.2f}  [{label}]")

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
    def _run_leg(self, checkpoints: list, leg: str) -> bool:
        remaining = [
            make_pose(self.navigator, x, y, yaw_deg)
            for x, y, yaw_deg, _ in checkpoints[1:]
        ]
        while remaining and rclpy.ok():
            success, left = self._navigate_through(remaining, leg)
            if success:
                return True
            if not 0 < left <= len(remaining):
                self.get_logger().error(
                    f"[{leg}] Invalid remaining count {left}. Ending patrol.")
                return False
            self.get_logger().error(
                f"[{leg}] Failed with {left} points left. Wait 5s and resume...")
            time.sleep(5.0)
            remaining = remaining[-left:]
        return False

    # ------------------------------------------------------------------
    def run_patrol(self):
        self.get_logger().info("Waiting for Nav2 (bt_navigator)...")
        self.navigator._waitForNodeToActivate("bt_navigator")
        self.get_logger().info("Nav2 active. Starting show2 recorded-route round trips.")

        loop = 0
        while rclpy.ok():
            loop += 1
            if self.max_loops > 0 and loop > self.max_loops:
                self.get_logger().info(f"Completed {self.max_loops} round trips. Done.")
                break
            self.get_logger().info(f"Round trip #{loop} start")

            if not self._run_leg(OUTBOUND_CHECKPOINTS, "outbound"):
                self.get_logger().error("Outbound did not complete. Ending patrol.")
                break
            self.get_logger().info(f"Outbound complete. Dwelling {self.end_dwell:.0f}s")
            time.sleep(self.end_dwell)

            if not self._run_leg(RETURN_CHECKPOINTS, "return"):
                self.get_logger().error("Return did not complete. Ending patrol.")
                break
            self.get_logger().info(f"Return complete. Dwelling {self.end_dwell:.0f}s")
            time.sleep(self.end_dwell)

            self.get_logger().info(f"Round trip #{loop} complete.")

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
