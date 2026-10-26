#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Patrol show1 using the short_02_signal_1015 interaction.

Waypoints follow the outbound recording at the demo2 spacing of 1.69625 m.
The final interval is shorter so the patrol reaches the recorded endpoint.
Start the robot near waypoint 0. The return uses the same path in reverse.

Wait for Int32 6 on /move/dest, then wait 5 seconds before outbound navigation.
After success and end_dwell, publish Int32 1 on /move/arrived at up to 2 Hz
until Bool True arrives on /move/arrived_received or Int32 12 on /move/dest.
An arrival acknowledgement only stops publishing; command 12 starts the return.
After return success and end_dwell, publish Bool True on /parking/done three
times, 0.5 seconds apart.
Publish Float32 /move/distance_remaining at up to 1 Hz within 10 m of each
leg's goal, and publish 0.0 on success. Failed legs resume remaining goals.
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Bool, Float32


# Approximate start/end positions were picked from the recorded trajectory.
# Each leg skips its first pose, which is the expected current location.
OUTBOUND_CHECKPOINTS = [
    (  -4.833,  -20.267, -150.78, "show1_wp_000"),
    (  -6.313,  -21.095, -151.12, "show1_wp_001"),
    (  -7.799,  -21.914, -152.73, "show1_wp_002"),
    (  -9.306,  -22.692, -152.66, "show1_wp_003"),
    ( -10.813,  -23.471, -153.27, "show1_wp_004"),
    ( -12.328,  -24.234, -152.63, "show1_wp_005"),
    ( -12.976,  -24.569, -152.63, "show1_wp_006"),
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
        super().__init__("through_poses_patrol_show1_signal")

        self.declare_parameter("end_dwell",   5.0)
        self.declare_parameter("max_loops",   0)
        self.declare_parameter("nav_timeout", 6000.0)

        self.end_dwell   = self.get_parameter("end_dwell").value
        self.max_loops   = self.get_parameter("max_loops").value
        self.nav_timeout = self.get_parameter("nav_timeout").value

        self.navigator = BasicNavigator()

        self.latest_command = 0
        self.arrived_received = False
        self.sub_start = self.create_subscription(
            Int32, '/move/dest', self.start_signal_cb, 10)
        self.sub_arrived_received = self.create_subscription(
            Bool, '/move/arrived_received', self.arrived_received_cb, 10)
        self.pub_reached = self.create_publisher(Int32, '/move/arrived', 10)
        self.pub_parking_done = self.create_publisher(Bool, '/parking/done', 10)
        self.pub_dist_remaining = self.create_publisher(
            Float32, '/move/distance_remaining', 10)
        self._log_checkpoints()

    def start_signal_cb(self, msg: Int32):
        self.latest_command = msg.data

    def arrived_received_cb(self, msg: Bool):
        if msg.data:
            self.arrived_received = True
            self.get_logger().info(
                "Received arrived confirmation (/move/arrived_received: True)")

    def _spin_for(self, duration: float) -> bool:
        start = time.time()
        while rclpy.ok() and time.time() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        return rclpy.ok()

    def _wait_for_command(self, command: int, description: str) -> bool:
        self.get_logger().info(
            f"Waiting for {description} signal ({command}) on /move/dest...")
        while rclpy.ok() and self.latest_command != command:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not rclpy.ok():
            return False
        self.latest_command = 0
        return True

    # ------------------------------------------------------------------
    def _log_checkpoints(self):
        sep = "=" * 64
        self.get_logger().info(
            f"\n{sep}\n"
            f"  NavigateThroughPoses show1 Signal-Controlled Round Trip\n"
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
        last_dist_pub_time = 0.0
        
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
                    dist_rem = fb.distance_remaining
                    self.get_logger().info(
                        f"[{leg}] poses_remaining={last_remaining}"
                        f"  dist={dist_rem:.1f}m",
                        throttle_duration_sec=5.0)
                    if dist_rem <= 10.0:
                        now = time.time()
                        if now - last_dist_pub_time >= 1.0:
                            dist_msg = Float32()
                            dist_msg.data = float(dist_rem)
                            self.pub_dist_remaining.publish(dist_msg)
                            last_dist_pub_time = now
                except AttributeError:
                    pass
            time.sleep(0.2)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"[{leg}] SUCCESS")
            dist_msg = Float32()
            dist_msg.data = 0.0
            self.pub_dist_remaining.publish(dist_msg)
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
        self.get_logger().info("Nav2 active. Starting show1 signal-controlled round trips.")

        loop = 0
        while rclpy.ok():
            loop += 1
            if self.max_loops > 0 and loop > self.max_loops:
                self.get_logger().info(f"Completed {self.max_loops} round trips. Done.")
                break
            self.get_logger().info(f"Round trip #{loop} start")

            if not self._wait_for_command(6, "start"):
                break
            self.get_logger().info(
                "Start signal (6) received. Waiting 5s before starting navigation...")
            if not self._spin_for(5.0):
                break
            self.get_logger().info("Wait complete, beginning outbound journey.")

            if not self._run_leg(OUTBOUND_CHECKPOINTS, "outbound"):
                self.get_logger().error("Outbound did not complete. Ending patrol.")
                break
            self.get_logger().info(f"Outbound complete. Dwelling {self.end_dwell:.0f}s")
            if not self._spin_for(self.end_dwell):
                break

            self.arrived_received = False
            arrived_msg = Int32()
            arrived_msg.data = 1
            self.get_logger().info(
                "Waiting for return start signal (12) on /move/dest "
                "(publishing 1 to /move/arrived until /move/arrived_received)...")
            last_pub_time = 0.0
            while rclpy.ok() and self.latest_command != 12:
                if not self.arrived_received:
                    now = time.time()
                    if now - last_pub_time >= 0.5:
                        self.pub_reached.publish(arrived_msg)
                        last_pub_time = now
                rclpy.spin_once(self, timeout_sec=0.1)

            if not rclpy.ok():
                break
            self.latest_command = 0
            self.get_logger().info(
                "Return start signal (12) received. Stopped publishing 1, "
                "beginning return journey.")

            if not self._run_leg(RETURN_CHECKPOINTS, "return"):
                self.get_logger().error("Return did not complete. Ending patrol.")
                break
            self.get_logger().info(f"Return complete. Dwelling {self.end_dwell:.0f}s")
            time.sleep(self.end_dwell)
            if not rclpy.ok():
                break

            parking_msg = Bool()
            parking_msg.data = True
            for _ in range(3):
                self.pub_parking_done.publish(parking_msg)
                time.sleep(0.5)
            self.get_logger().info(
                "Sent arrived signal (True) to /parking/done (Return Complete).")

            self.get_logger().info(f"Round trip #{loop} complete.")

        self.get_logger().info("Patrol ended.")

# ---------------------------------------------------------------------------
def main():
    rclpy.init()
    node = ThroughPosesPatrol()
    try:
        node.run_patrol()
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
