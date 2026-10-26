# ============================================================
# Auto-generated 2026-09-10 from localization_20260909_172913.log
# Method  : centerline avg(outbound, reversed-return) + Option A Patch
# Points  : 79 outbound + 79 return
# Spacing : ~5.0m target arc-length
# Patch 1 : wp043-046 y offset -0.50m
# Patch 2 : wp049-052 Option A (outbound south channel, avoids lawn)
# ============================================================
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
    (    -0.416,       0.027,    11.03, "b2b_wp_001"),
    (     4.428,       0.971,     7.02, "b2b_wp_002"),
    (     9.382,       1.581,     9.24, "b2b_wp_003"),
    (    14.308,       2.382,    16.43, "b2b_wp_004"),
    (    19.070,       3.786,    28.14, "b2b_wp_005"),
    (    23.439,       6.123,    14.57, "b2b_wp_006"),
    (    28.270,       7.379,    13.89, "b2b_wp_007"),
    (    33.116,       8.577,    10.76, "b2b_wp_008"),
    (    38.013,       9.508,     2.32, "b2b_wp_009"),
    (    43.000,       9.710,    -4.62, "b2b_wp_010"),
    (    47.922,       9.312,   -43.52, "b2b_wp_011"),
    (    51.508,       5.906,   -47.11, "b2b_wp_012"),
    (    54.904,       2.251,   -24.08, "b2b_wp_013"),
    (    59.382,       0.249,     0.79, "b2b_wp_014"),
    (    64.368,       0.318,     4.38, "b2b_wp_015"),
    (    69.345,       0.699,     5.52, "b2b_wp_016"),
    (    74.313,       1.180,     3.15, "b2b_wp_017"),
    (    79.297,       1.454,     0.96, "b2b_wp_018"),
    (    84.287,       1.537,    10.73, "b2b_wp_019"),
    (    89.188,       2.466,     5.27, "b2b_wp_020"),
    (    94.153,       2.925,     9.44, "b2b_wp_021"),
    (    99.076,       3.743,     1.53, "b2b_wp_022"),
    (   104.060,       3.876,    -2.63, "b2b_wp_023"),
    (   109.040,       3.647,     8.83, "b2b_wp_024"),
    (   113.972,       4.413,     0.03, "b2b_wp_025"),
    (   118.940,       4.416,   -25.62, "b2b_wp_026"),
    (   123.402,       2.276,   -49.53, "b2b_wp_027"),
    (   126.626,      -1.503,   -58.17, "b2b_wp_028"),
    (   129.258,      -5.744,   -55.57, "b2b_wp_029"),
    (   132.080,      -9.862,   -56.88, "b2b_wp_030"),
    (   134.808,     -14.042,   -56.89, "b2b_wp_031"),
    (   137.534,     -18.223,   -56.20, "b2b_wp_032"),
    (   140.310,     -22.370,   -52.45, "b2b_wp_033"),
    (   143.352,     -26.326,   -55.67, "b2b_wp_034"),
    (   146.167,     -30.448,   -58.61, "b2b_wp_035"),
    (   148.767,     -34.709,   -53.41, "b2b_wp_036"),
    (   151.739,     -38.713,   -56.43, "b2b_wp_037"),
    (   154.494,     -42.864,   -43.98, "b2b_wp_038"),
    (   158.064,     -46.309,   -41.06, "b2b_wp_039"),
    (   161.821,     -49.582,   -27.85, "b2b_wp_040"),
    (   166.227,     -51.910,   -13.43, "b2b_wp_041"),
    (   171.076,     -53.067,   -14.59, "b2b_wp_042"),
    (   176.006,     -54.350,   -13.03, "b2b_wp_043"),
    (   180.867,     -55.475,    -1.46, "b2b_wp_044"),
    (   185.674,     -55.598,    68.02, "b2b_wp_045"),
    (   187.531,     -50.998,    36.90, "b2b_wp_046"),
    (   191.684,     -47.880,    20.33, "b2b_wp_047"),
    (   196.363,     -46.146,    39.78, "b2b_wp_048"),
    (   200.626,     -47.385,    35.10, "b2b_wp_049"),
    (   204.643,     -44.381,    36.20, "b2b_wp_050"),
    (   207.920,     -41.818,    49.60, "b2b_wp_051"),
    (   210.775,     -37.840,    76.40, "b2b_wp_052"),
    (   210.407,     -32.103,    34.47, "b2b_wp_053"),
    (   214.522,     -29.278,    36.81, "b2b_wp_054"),
    (   218.517,     -26.287,    24.39, "b2b_wp_055"),
    (   223.011,     -24.250,     2.56, "b2b_wp_056"),
    (   227.996,     -24.027,     5.50, "b2b_wp_057"),
    (   232.964,     -23.548,     4.40, "b2b_wp_058"),
    (   237.941,     -23.165,     3.56, "b2b_wp_059"),
    (   242.923,     -22.856,     4.64, "b2b_wp_060"),
    (   247.897,     -22.452,    26.47, "b2b_wp_061"),
    (   252.260,     -20.279,    62.61, "b2b_wp_062"),
    (   254.546,     -15.868,    63.22, "b2b_wp_063"),
    (   256.795,     -11.412,    64.54, "b2b_wp_064"),
    (   258.940,      -6.906,    60.73, "b2b_wp_065"),
    (   261.380,      -2.551,    63.11, "b2b_wp_066"),
    (   263.638,       1.900,    62.48, "b2b_wp_067"),
    (   265.944,       6.327,    65.52, "b2b_wp_068"),
    (   268.013,      10.870,    63.72, "b2b_wp_069"),
    (   270.223,      15.346,    65.94, "b2b_wp_070"),
    (   272.258,      19.904,    61.96, "b2b_wp_071"),
    (   274.602,      24.306,    61.36, "b2b_wp_072"),
    (   276.994,      28.685,    64.22, "b2b_wp_073"),
    (   279.165,      33.180,    66.46, "b2b_wp_074"),
    (   281.159,      37.755,    62.97, "b2b_wp_075"),
    (   283.424,      42.195,    53.35, "b2b_wp_076"),
    (   286.400,      46.196,    37.67, "b2b_wp_077"),
    (   290.328,      49.229,    14.70, "b2b_wp_078"),
    (   295.120,      50.486,    14.70, "b2b_wp_079"),
]

# Return checkpoints: reverse of outbound with flipped yaw
RETURN_CHECKPOINTS = [
    (   295.120,      50.486,    14.70, "ret_b2b_wp_079"),
    (   290.328,      49.229,    14.70, "ret_b2b_wp_078"),
    (   286.400,      46.196,    37.67, "ret_b2b_wp_077"),
    (   283.424,      42.195,    53.35, "ret_b2b_wp_076"),
    (   281.159,      37.755,    62.97, "ret_b2b_wp_075"),
    (   279.165,      33.180,    66.46, "ret_b2b_wp_074"),
    (   276.994,      28.685,    64.22, "ret_b2b_wp_073"),
    (   274.602,      24.306,    61.36, "ret_b2b_wp_072"),
    (   272.258,      19.904,    61.96, "ret_b2b_wp_071"),
    (   270.223,      15.346,    65.94, "ret_b2b_wp_070"),
    (   268.013,      10.870,    63.72, "ret_b2b_wp_069"),
    (   265.944,       6.327,    65.52, "ret_b2b_wp_068"),
    (   263.638,       1.900,    62.48, "ret_b2b_wp_067"),
    (   261.380,      -2.551,    63.11, "ret_b2b_wp_066"),
    (   258.940,      -6.906,    60.73, "ret_b2b_wp_065"),
    (   256.795,     -11.412,    64.54, "ret_b2b_wp_064"),
    (   254.546,     -15.868,    63.22, "ret_b2b_wp_063"),
    (   252.260,     -20.279,    62.61, "ret_b2b_wp_062"),
    (   247.897,     -22.452,    26.47, "ret_b2b_wp_061"),
    (   242.923,     -22.856,     4.64, "ret_b2b_wp_060"),
    (   237.941,     -23.165,     3.56, "ret_b2b_wp_059"),
    (   232.964,     -23.548,     4.40, "ret_b2b_wp_058"),
    (   227.996,     -24.027,     5.50, "ret_b2b_wp_057"),
    (   223.011,     -24.250,     2.56, "ret_b2b_wp_056"),
    (   218.517,     -26.287,    24.39, "ret_b2b_wp_055"),
    (   214.522,     -29.278,    36.81, "ret_b2b_wp_054"),
    (   210.407,     -32.103,    34.47, "ret_b2b_wp_053"),
    (   210.775,     -37.840,    76.40, "ret_b2b_wp_052"),
    (   207.920,     -41.818,    49.60, "ret_b2b_wp_051"),
    (   204.643,     -44.381,    36.20, "ret_b2b_wp_050"),
    (   200.626,     -47.385,    35.10, "ret_b2b_wp_049"),
    (   196.363,     -46.146,    39.78, "ret_b2b_wp_048"),
    (   191.684,     -47.880,    20.33, "ret_b2b_wp_047"),
    (   187.531,     -50.998,    36.90, "ret_b2b_wp_046"),
    (   185.674,     -55.598,    68.02, "ret_b2b_wp_045"),
    (   180.867,     -55.475,    -1.46, "ret_b2b_wp_044"),
    (   176.006,     -54.350,   -13.03, "ret_b2b_wp_043"),
    (   171.076,     -53.067,   -14.59, "ret_b2b_wp_042"),
    (   166.227,     -51.910,   -13.43, "ret_b2b_wp_041"),
    (   161.821,     -49.582,   -27.85, "ret_b2b_wp_040"),
    (   158.064,     -46.309,   -41.06, "ret_b2b_wp_039"),
    (   154.494,     -42.864,   -43.98, "ret_b2b_wp_038"),
    (   151.739,     -38.713,   -56.43, "ret_b2b_wp_037"),
    (   148.767,     -34.709,   -53.41, "ret_b2b_wp_036"),
    (   146.167,     -30.448,   -58.61, "ret_b2b_wp_035"),
    (   143.352,     -26.326,   -55.67, "ret_b2b_wp_034"),
    (   140.310,     -22.370,   -52.45, "ret_b2b_wp_033"),
    (   137.534,     -18.223,   -56.20, "ret_b2b_wp_032"),
    (   134.808,     -14.042,   -56.89, "ret_b2b_wp_031"),
    (   132.080,      -9.862,   -56.88, "ret_b2b_wp_030"),
    (   129.258,      -5.744,   -55.57, "ret_b2b_wp_029"),
    (   126.626,      -1.503,   -58.17, "ret_b2b_wp_028"),
    (   123.402,       2.276,   -49.53, "ret_b2b_wp_027"),
    (   118.940,       4.416,   -25.62, "ret_b2b_wp_026"),
    (   113.972,       4.413,     0.03, "ret_b2b_wp_025"),
    (   109.040,       3.647,     8.83, "ret_b2b_wp_024"),
    (   104.060,       3.876,    -2.63, "ret_b2b_wp_023"),
    (    99.076,       3.743,     1.53, "ret_b2b_wp_022"),
    (    94.153,       2.925,     9.44, "ret_b2b_wp_021"),
    (    89.188,       2.466,     5.27, "ret_b2b_wp_020"),
    (    84.287,       1.537,    10.73, "ret_b2b_wp_019"),
    (    79.297,       1.454,     0.96, "ret_b2b_wp_018"),
    (    74.313,       1.180,     3.15, "ret_b2b_wp_017"),
    (    69.345,       0.699,     5.52, "ret_b2b_wp_016"),
    (    64.368,       0.318,     4.38, "ret_b2b_wp_015"),
    (    59.382,       0.249,     0.79, "ret_b2b_wp_014"),
    (    54.904,       2.251,   -24.08, "ret_b2b_wp_013"),
    (    51.508,       5.906,   -47.11, "ret_b2b_wp_012"),
    (    47.922,       9.312,   -43.52, "ret_b2b_wp_011"),
    (    43.000,       9.710,    -4.62, "ret_b2b_wp_010"),
    (    38.013,       9.508,     2.32, "ret_b2b_wp_009"),
    (    33.116,       8.577,    10.76, "ret_b2b_wp_008"),
    (    28.270,       7.379,    13.89, "ret_b2b_wp_007"),
    (    23.439,       6.123,    14.57, "ret_b2b_wp_006"),
    (    19.070,       3.786,    28.14, "ret_b2b_wp_005"),
    (    14.308,       2.382,    16.43, "ret_b2b_wp_004"),
    (     9.382,       1.581,     9.24, "ret_b2b_wp_003"),
    (     4.428,       0.971,     7.02, "ret_b2b_wp_002"),
    (    -0.416,       0.027,    11.03, "ret_b2b_wp_001"),
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
