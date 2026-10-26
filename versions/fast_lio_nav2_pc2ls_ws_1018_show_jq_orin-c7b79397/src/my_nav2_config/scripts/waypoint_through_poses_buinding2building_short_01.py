# ============================================================
# Auto-generated 2026-09-10 from localization_20260909_172913.log
# Method  : centerline avg(outbound, reversed-return) + Option A Patch
# Points  : 109 outbound + 109 return (truncated to b2b_wp_109 as terminal endpoint)
# Spacing : approximately 2.4m; larger gaps exist at selected bends
# Patch 1 : wp043-046 y offset -0.50m
# Patch 2 : wp049-052 Option A (outbound south channel, avoids lawn)
# 2026-09-23: remove the local wp202/wp203 backtrack in both directions.
# Patch 3 : 2026-10-08 wp074-089 updated from localization_2026-10-08_13-34-56.log
#           (detour section along north bypass corridor, max offset 4.7m at wp084)
# Patch 4 : 2026-10-09 terminal point truncated to b2b_wp_109 (213.583, -30.842, 27.05)
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
    (    -0.121,      -0.099,     9.41, "b2b_wp_001"),
    (     2.135,       0.275,     7.52, "b2b_wp_002"),
    (     4.520,       0.590,     5.43, "b2b_wp_003"),
    (     6.571,       0.785,     4.97, "b2b_wp_004"),
    (     8.596,       0.961,     8.70, "b2b_wp_005"),
    (    10.863,       1.308,    10.59, "b2b_wp_006"),
    (    12.890,       1.687,    12.42, "b2b_wp_007"),
    (    15.051,       2.163,    13.87, "b2b_wp_008"),
    (    17.383,       2.739,    14.28, "b2b_wp_009"),
    (    19.795,       3.353,    13.69, "b2b_wp_010"),
    (    22.271,       3.956,    12.93, "b2b_wp_011"),
    (    24.762,       4.528,    13.31, "b2b_wp_012"),
    (    27.196,       5.104,    14.08, "b2b_wp_013"),
    (    29.592,       5.705,    14.00, "b2b_wp_014"),
    (    31.931,       6.288,    11.80, "b2b_wp_015"),
    (    34.339,       6.791,     8.74, "b2b_wp_016"),
    (    36.830,       7.174,     6.13, "b2b_wp_017"),
    (    39.289,       7.438,     0.64, "b2b_wp_018"),
    (    41.956,       7.468,    -9.95, "b2b_wp_019"),
    (    44.066,       7.098,   -19.95, "b2b_wp_020"),
    (    46.033,       6.384,   -27.74, "b2b_wp_021"),
    (    47.866,       5.420,   -35.93, "b2b_wp_022"),
    (    49.588,       4.172,   -45.06, "b2b_wp_023"),
    (    51.413,       2.343,   -44.61, "b2b_wp_024"),
    (    52.961,       0.816,   -36.32, "b2b_wp_025"),
    (    54.630,      -0.411,   -21.15, "b2b_wp_026"),
    (    56.807,      -1.253,    -5.69, "b2b_wp_027"),
    (    58.803,      -1.452,     1.05, "b2b_wp_028"),
    (    60.994,      -1.412,     3.42, "b2b_wp_029"),
    (    63.323,      -1.273,     3.40, "b2b_wp_030"),
    (    65.797,      -1.126,     3.89, "b2b_wp_031"),
    (    68.209,      -0.962,     3.64, "b2b_wp_032"),
    (    70.709,      -0.803,     3.35, "b2b_wp_033"),
    (    73.120,      -0.662,     4.21, "b2b_wp_034"),
    (    75.526,      -0.485,     5.23, "b2b_wp_035"),
    (    77.929,      -0.265,     6.80, "b2b_wp_036"),
    (    80.386,       0.028,     8.52, "b2b_wp_037"),
    (    82.842,       0.396,    10.09, "b2b_wp_038"),
    (    85.254,       0.825,     9.26, "b2b_wp_039"),
    (    87.616,       1.210,     7.87, "b2b_wp_040"),
    # (    89.960,       1.534,     5.11, "b2b_wp_041"),
    # (    92.341,       1.747,     2.95, "b2b_wp_042"),
    # (    94.790,       1.873,     4.95, "b2b_wp_043"),
    # (    97.215,       2.083,     8.02, "b2b_wp_044"),
    # (    99.593,       2.418,    10.98, "b2b_wp_045"),
    # (   101.953,       2.876,    11.60, "b2b_wp_046"),
    # (   104.345,       3.367,     7.86, "b2b_wp_047"),
    # (   106.838,       3.711,     2.25, "b2b_wp_048"),
    # (   109.251,       3.806,    -3.64, "b2b_wp_049"),
    # (   111.658,       3.653,    -7.23, "b2b_wp_050"),
    # (   114.085,       3.345,   -11.53, "b2b_wp_051"),
    # (   116.560,       2.840,   -15.93, "b2b_wp_052"),
    # (   119.047,       2.130,   -22.91, "b2b_wp_053"),
    # (   121.439,       1.119,   -34.00, "b2b_wp_054"),
    # (   123.513,      -0.280,   -43.49, "b2b_wp_055"),
    # (   125.265,      -1.942,   -50.14, "b2b_wp_056"),
    # (   126.834,      -3.821,   -54.08, "b2b_wp_057"),
    # (   128.283,      -5.821,   -55.09, "b2b_wp_058"),
    # (   129.690,      -7.837,   -55.65, "b2b_wp_059"),
    # (   131.067,      -9.852,   -57.44, "b2b_wp_060"),
    # (   132.378,     -11.905,   -57.04, "b2b_wp_061"),
    # (   133.687,     -13.924,   -56.03, "b2b_wp_062"),
    # (   135.034,     -15.923,   -55.04, "b2b_wp_063"),
    # (   136.431,     -17.921,   -54.15, "b2b_wp_064"),
    # (   137.899,     -19.953,   -55.77, "b2b_wp_065"),
    # (   139.313,     -22.031,   -56.34, "b2b_wp_066"),
    # (   140.684,     -24.090,   -56.72, "b2b_wp_067"),
    # (   142.038,     -26.153,   -56.30, "b2b_wp_068"),
    # (   143.366,     -28.144,   -53.72, "b2b_wp_069"),
    # (   144.740,     -30.016,   -49.72, "b2b_wp_070"),
    # (   146.322,     -32.495,   -57.21, "b2b_wp_071"),
    # (   147.925,     -34.960,   -55.48, "b2b_wp_072"),
    # (   149.653,     -37.338,   -56.12, "b2b_wp_073"),
    # (   151.200,     -39.837,   -56.34, "b2b_wp_074"),
    # (   152.903,     -42.217,   -54.91, "b2b_wp_075"),
    # (   154.217,     -44.132,   -59.50, "b2b_wp_076"),
    # (   155.552,     -46.717,   -56.89, "b2b_wp_077"),
    # (   157.300,     -49.000,   -41.22, "b2b_wp_078"),
    # (   159.650,     -50.350,   -11.22, "b2b_wp_079"),
    # (   161.950,     -50.050,    21.91, "b2b_wp_080"),
    # (   164.403,     -48.242,    31.56, "b2b_wp_081"),
    # (   166.641,     -46.875,    30.23, "b2b_wp_082"),
    # (   168.750,     -45.700,    17.22, "b2b_wp_083"),
    # (   170.900,     -45.500,   -13.83, "b2b_wp_084"),
    # (   172.750,     -46.700,   -44.52, "b2b_wp_085"),
    # (   174.400,     -49.152,   -56.82, "b2b_wp_086"),
    # (   176.100,     -51.529,   -50.36, "b2b_wp_087"),
    # (   177.823,     -53.284,   -42.95, "b2b_wp_088"),
    # (   180.001,     -55.161,   -45.47, "b2b_wp_089"),
    # (   181.784,     -57.311,   -12.88, "b2b_wp_090"),
    # (   184.059,     -57.831,     4.61, "b2b_wp_091"),
    # (   186.304,     -57.650,    23.39, "b2b_wp_092"),
    # (   188.304,     -56.785,    39.20, "b2b_wp_093"),
    # (   190.088,     -55.330,    47.19, "b2b_wp_094"),
    # (   191.645,     -53.649,    49.76, "b2b_wp_095"),
    # (   193.104,     -51.925,    52.16, "b2b_wp_096"),
    # (   194.444,     -50.200,    59.71, "b2b_wp_097"),
    # (   195.582,     -48.252,    67.07, "b2b_wp_098"),
    # (   196.481,     -46.127,    72.78, "b2b_wp_099"),
    # (   197.179,     -43.875,    72.02, "b2b_wp_100"),
    # (   197.949,     -41.502,    65.05, "b2b_wp_101"),
    # (   198.964,     -39.320,    51.60, "b2b_wp_102"),
    # (   200.469,     -37.421,    35.38, "b2b_wp_103"),
    # (   202.556,     -35.939,    24.76, "b2b_wp_104"),
    # (   204.854,     -34.879,    20.67, "b2b_wp_105"),
    # (   207.118,     -34.025,    22.71, "b2b_wp_106"),
    # (   209.256,     -33.130,    26.93, "b2b_wp_107"),
    # (   211.313,     -32.085,    28.70, "b2b_wp_108"),
    # (   213.583,     -30.842,    27.05, "b2b_wp_109"),
]



# Return checkpoints: reverse of outbound with flipped yaw
RETURN_CHECKPOINTS = [
    # (   213.583,     -30.842,  -151.30, "ret_b2b_wp_109"),
    # (   211.313,     -32.085,  -153.07, "ret_b2b_wp_108"),
    # (   209.256,     -33.130,  -157.29, "ret_b2b_wp_107"),
    # (   207.118,     -34.025,  -159.33, "ret_b2b_wp_106"),
    # (   204.854,     -34.879,  -155.24, "ret_b2b_wp_105"),
    # (   202.556,     -35.939,  -144.62, "ret_b2b_wp_104"),
    # (   200.469,     -37.421,  -128.40, "ret_b2b_wp_103"),
    # (   198.964,     -39.320,  -114.95, "ret_b2b_wp_102"),
    # (   197.949,     -41.502,  -107.98, "ret_b2b_wp_101"),
    # (   197.179,     -43.875,  -107.22, "ret_b2b_wp_100"),
    # (   196.481,     -46.127,  -112.93, "ret_b2b_wp_099"),
    # (   195.582,     -48.252,  -120.29, "ret_b2b_wp_098"),
    # (   194.444,     -50.200,  -127.84, "ret_b2b_wp_097"),
    # (   193.104,     -51.925,  -130.24, "ret_b2b_wp_096"),
    # (   191.645,     -53.649,  -132.81, "ret_b2b_wp_095"),
    # (   190.088,     -55.330,  -140.80, "ret_b2b_wp_094"),
    # (   188.304,     -56.785,  -156.61, "ret_b2b_wp_093"),
    # (   186.304,     -57.650,  -175.39, "ret_b2b_wp_092"),
    # (   184.059,     -57.831,   167.12, "ret_b2b_wp_091"),
    # (   181.784,     -57.311,   158.64, "ret_b2b_wp_090"),
    # (   180.001,     -55.161,   134.53, "ret_b2b_wp_089"),
    # (   177.823,     -53.284,   137.05, "ret_b2b_wp_088"),
    # (   176.100,     -51.529,   129.64, "ret_b2b_wp_087"),
    # (   174.400,     -49.152,   123.18, "ret_b2b_wp_086"),
    # (   172.750,     -46.700,   135.48, "ret_b2b_wp_085"),
    # (   170.900,     -45.500,   166.17, "ret_b2b_wp_084"),
    # (   168.750,     -45.700,  -162.78, "ret_b2b_wp_083"),
    # (   166.641,     -46.875,  -149.77, "ret_b2b_wp_082"),
    # (   164.403,     -48.242,  -148.44, "ret_b2b_wp_081"),
    # (   161.950,     -50.050,  -158.09, "ret_b2b_wp_080"),
    # (   159.650,     -50.350,   168.78, "ret_b2b_wp_079"),
    # (   157.300,     -49.000,   138.78, "ret_b2b_wp_078"),
    # (   155.552,     -46.717,   123.11, "ret_b2b_wp_077"),
    # (   154.217,     -44.132,   120.50, "ret_b2b_wp_076"),
    # (   152.903,     -42.217,   125.09, "ret_b2b_wp_075"),
    # (   151.200,     -39.837,   123.66, "ret_b2b_wp_074"),
    # (   149.653,     -37.338,   123.88, "ret_b2b_wp_073"),
    # (   147.925,     -34.960,   124.52, "ret_b2b_wp_072"),
    # (   146.322,     -32.495,   122.79, "ret_b2b_wp_071"),
    # (   144.740,     -30.016,   126.28, "ret_b2b_wp_070"),
    # (   143.366,     -28.144,   123.70, "ret_b2b_wp_069"),
    # (   142.038,     -26.153,   123.28, "ret_b2b_wp_068"),
    # (   140.684,     -24.090,   123.66, "ret_b2b_wp_067"),
    # (   139.313,     -22.031,   124.23, "ret_b2b_wp_066"),
    # (   137.899,     -19.953,   125.85, "ret_b2b_wp_065"),
    # (   136.431,     -17.921,   124.96, "ret_b2b_wp_064"),
    # (   135.034,     -15.923,   123.97, "ret_b2b_wp_063"),
    # (   133.687,     -13.924,   122.96, "ret_b2b_wp_062"),
    # (   132.378,     -11.905,   122.56, "ret_b2b_wp_061"),
    # (   131.067,      -9.852,   124.35, "ret_b2b_wp_060"),
    # (   129.690,      -7.837,   124.91, "ret_b2b_wp_059"),
    # (   128.283,      -5.821,   125.92, "ret_b2b_wp_058"),
    # (   126.834,      -3.821,   129.86, "ret_b2b_wp_057"),
    # (   125.265,      -1.942,   136.51, "ret_b2b_wp_056"),
    # (   123.513,      -0.280,   146.00, "ret_b2b_wp_055"),
    # (   121.439,       1.119,   157.09, "ret_b2b_wp_054"),
    # (   119.047,       2.130,   164.07, "ret_b2b_wp_053"),
    # (   116.560,       2.840,   168.47, "ret_b2b_wp_052"),
    # (   114.085,       3.345,   172.77, "ret_b2b_wp_051"),
    # (   111.658,       3.653,   176.36, "ret_b2b_wp_050"),
    # (   109.251,       3.806,  -177.75, "ret_b2b_wp_049"),
    # (   106.838,       3.711,  -172.14, "ret_b2b_wp_048"),
    # (   104.345,       3.367,  -168.40, "ret_b2b_wp_047"),
    # (   101.953,       2.876,  -169.02, "ret_b2b_wp_046"),
    # (    99.593,       2.418,  -171.98, "ret_b2b_wp_045"),
    # (    97.215,       2.083,  -175.05, "ret_b2b_wp_044"),
    # (    94.790,       1.873,  -177.05, "ret_b2b_wp_043"),
    # (    92.341,       1.747,  -174.89, "ret_b2b_wp_042"),
    # (    89.960,       1.534,  -172.13, "ret_b2b_wp_041"),
    (    87.616,       1.210,  -170.74, "ret_b2b_wp_040"),
    (    85.254,       0.825,  -169.91, "ret_b2b_wp_039"),
    (    82.842,       0.396,  -171.48, "ret_b2b_wp_038"),
    (    80.386,       0.028,  -173.20, "ret_b2b_wp_037"),
    (    77.929,      -0.265,  -174.77, "ret_b2b_wp_036"),
    (    75.526,      -0.485,  -175.79, "ret_b2b_wp_035"),
    (    73.120,      -0.662,  -176.65, "ret_b2b_wp_034"),
    (    70.709,      -0.803,  -176.36, "ret_b2b_wp_033"),
    (    68.209,      -0.962,  -176.11, "ret_b2b_wp_032"),
    (    65.797,      -1.126,  -176.60, "ret_b2b_wp_031"),
    (    63.323,      -1.273,  -176.58, "ret_b2b_wp_030"),
    (    60.994,      -1.412,  -178.95, "ret_b2b_wp_029"),
    (    58.803,      -1.452,   174.31, "ret_b2b_wp_028"),
    (    56.807,      -1.253,   158.85, "ret_b2b_wp_027"),
    (    54.630,      -0.411,   143.68, "ret_b2b_wp_026"),
    (    52.961,       0.816,   135.39, "ret_b2b_wp_025"),
    (    51.413,       2.343,   134.94, "ret_b2b_wp_024"),
    (    49.588,       4.172,   144.07, "ret_b2b_wp_023"),
    (    47.866,       5.420,   152.26, "ret_b2b_wp_022"),
    (    46.033,       6.384,   160.05, "ret_b2b_wp_021"),
    (    44.066,       7.098,   170.05, "ret_b2b_wp_020"),
    (    41.956,       7.468,  -179.36, "ret_b2b_wp_019"),
    (    39.289,       7.438,  -173.87, "ret_b2b_wp_018"),
    (    36.830,       7.174,  -171.26, "ret_b2b_wp_017"),
    (    34.339,       6.791,  -168.20, "ret_b2b_wp_016"),
    (    31.931,       6.288,  -166.00, "ret_b2b_wp_015"),
    (    29.592,       5.705,  -165.92, "ret_b2b_wp_014"),
    (    27.196,       5.104,  -166.69, "ret_b2b_wp_013"),
    (    24.762,       4.528,  -167.07, "ret_b2b_wp_012"),
    (    22.271,       3.956,  -166.31, "ret_b2b_wp_011"),
    (    19.795,       3.353,  -165.72, "ret_b2b_wp_010"),
    (    17.383,       2.739,  -166.13, "ret_b2b_wp_009"),
    (    15.051,       2.163,  -167.58, "ret_b2b_wp_008"),
    (    12.890,       1.687,  -169.41, "ret_b2b_wp_007"),
    (    10.863,       1.308,  -171.30, "ret_b2b_wp_006"),
    (     8.596,       0.961,  -175.03, "ret_b2b_wp_005"),
    (     6.571,       0.785,  -174.57, "ret_b2b_wp_004"),
    (     4.520,       0.590,  -172.48, "ret_b2b_wp_003"),
    (     2.135,       0.275,  -170.59, "ret_b2b_wp_002"),
    (    -0.121,      -0.099,  -170.59, "ret_b2b_wp_001"),
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
