# ============================================================
# Auto-generated 2026-09-10 from localization_20260909_172913.log
# Method  : centerline avg(outbound, reversed-return) + Option A Patch
# Points  : 203 outbound + 203 return (original waypoint IDs retained)
# Spacing : approximately 2.4m; larger gaps exist at selected bends
# Patch 1 : wp043-046 y offset -0.50m
# Patch 2 : wp049-052 Option A (outbound south channel, avoids lawn)
# 2026-09-23: remove the local wp202/wp203 backtrack in both directions.
# Patch 3 : 2026-10-08 wp074-089 updated from localization_2026-10-08_13-34-56.log
#           (detour section along north bypass corridor, max offset 4.7m at wp084)
# The wp201-wp204 corridor was checked against map_jeck static/keepout grids
# with the 0.5m-square footprint circumscribed radius plus 0.15m margin.
# Other checkpoints and both terminal poses are preserved; Nav2 still plans
# around live obstacles. This map check does not replace a field test.
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
    (    89.960,       1.534,     5.11, "b2b_wp_041"),
    (    92.341,       1.747,     2.95, "b2b_wp_042"),
    (    94.790,       1.873,     4.95, "b2b_wp_043"),
    (    97.215,       2.083,     8.02, "b2b_wp_044"),
    (    99.593,       2.418,    10.98, "b2b_wp_045"),
    (   101.953,       2.876,    11.60, "b2b_wp_046"),
    (   104.345,       3.367,     7.86, "b2b_wp_047"),
    (   106.838,       3.711,     2.25, "b2b_wp_048"),
    (   109.251,       3.806,    -3.64, "b2b_wp_049"),
    (   111.658,       3.653,    -7.23, "b2b_wp_050"),
    (   114.085,       3.345,   -11.53, "b2b_wp_051"),
    (   116.560,       2.840,   -15.93, "b2b_wp_052"),
    (   119.047,       2.130,   -22.91, "b2b_wp_053"),
    (   121.439,       1.119,   -34.00, "b2b_wp_054"),
    (   123.513,      -0.280,   -43.49, "b2b_wp_055"),
    (   125.265,      -1.942,   -50.14, "b2b_wp_056"),
    (   126.834,      -3.821,   -54.08, "b2b_wp_057"),
    (   128.283,      -5.821,   -55.09, "b2b_wp_058"),
    (   129.690,      -7.837,   -55.65, "b2b_wp_059"),
    (   131.067,      -9.852,   -57.44, "b2b_wp_060"),
    (   132.378,     -11.905,   -57.04, "b2b_wp_061"),
    (   133.687,     -13.924,   -56.03, "b2b_wp_062"),
    (   135.034,     -15.923,   -55.04, "b2b_wp_063"),
    (   136.431,     -17.921,   -54.15, "b2b_wp_064"),
    (   137.899,     -19.953,   -55.77, "b2b_wp_065"),
    (   139.313,     -22.031,   -56.34, "b2b_wp_066"),
    (   140.684,     -24.090,   -56.72, "b2b_wp_067"),
    (   142.038,     -26.153,   -56.30, "b2b_wp_068"),
    (   143.366,     -28.144,   -53.72, "b2b_wp_069"),
    (   144.740,     -30.016,   -49.72, "b2b_wp_070"),
    (   146.322,     -32.495,   -57.21, "b2b_wp_071"),
    (   147.925,     -34.960,   -55.48, "b2b_wp_072"),
    (   149.653,     -37.338,   -56.12, "b2b_wp_073"),
    (   151.200,     -39.837,   -56.34, "b2b_wp_074"),
    (   152.903,     -42.217,   -54.91, "b2b_wp_075"),
    (   154.217,     -44.132,   -59.50, "b2b_wp_076"),
    (   155.552,     -46.717,   -56.89, "b2b_wp_077"),
    (   157.300,     -49.000,   -41.22, "b2b_wp_078"),
    (   159.650,     -50.350,   -11.22, "b2b_wp_079"),
    (   161.950,     -50.050,    21.91, "b2b_wp_080"),
    (   164.403,     -48.242,    31.56, "b2b_wp_081"),
    (   166.641,     -46.875,    30.23, "b2b_wp_082"),
    (   168.750,     -45.700,    17.22, "b2b_wp_083"),
    (   170.900,     -45.500,   -13.83, "b2b_wp_084"),
    (   172.750,     -46.700,   -44.52, "b2b_wp_085"),
    (   174.400,     -49.152,   -56.82, "b2b_wp_086"),
    (   176.100,     -51.529,   -50.36, "b2b_wp_087"),
    (   177.823,     -53.284,   -42.95, "b2b_wp_088"),
    (   180.001,     -55.161,   -45.47, "b2b_wp_089"),
    (   181.784,     -57.311,   -12.88, "b2b_wp_090"),
    (   184.059,     -57.831,     4.61, "b2b_wp_091"),
    (   186.304,     -57.650,    23.39, "b2b_wp_092"),
    (   188.304,     -56.785,    39.20, "b2b_wp_093"),
    (   190.088,     -55.330,    47.19, "b2b_wp_094"),
    (   191.645,     -53.649,    49.76, "b2b_wp_095"),
    (   193.104,     -51.925,    52.16, "b2b_wp_096"),
    (   194.444,     -50.200,    59.71, "b2b_wp_097"),
    (   195.582,     -48.252,    67.07, "b2b_wp_098"),
    (   196.481,     -46.127,    72.78, "b2b_wp_099"),
    (   197.179,     -43.875,    72.02, "b2b_wp_100"),
    (   197.949,     -41.502,    65.05, "b2b_wp_101"),
    (   198.964,     -39.320,    51.60, "b2b_wp_102"),
    (   200.469,     -37.421,    35.38, "b2b_wp_103"),
    (   202.556,     -35.939,    24.76, "b2b_wp_104"),
    (   204.854,     -34.879,    20.67, "b2b_wp_105"),
    (   207.118,     -34.025,    22.71, "b2b_wp_106"),
    (   209.256,     -33.130,    26.93, "b2b_wp_107"),
    (   211.313,     -32.085,    28.70, "b2b_wp_108"),
    (   213.583,     -30.842,    27.05, "b2b_wp_109"),
    (   215.410,     -29.909,    24.19, "b2b_wp_110"),
    (   217.242,     -29.086,    22.40, "b2b_wp_111"),
    (   219.649,     -28.094,    16.86, "b2b_wp_112"),
    (   222.002,     -27.381,    12.79, "b2b_wp_113"),
    (   224.327,     -26.853,     9.80, "b2b_wp_114"),
    (   226.690,     -26.445,     5.93, "b2b_wp_115"),
    (   229.153,     -26.189,     6.15, "b2b_wp_116"),
    (   231.586,     -25.927,     4.64, "b2b_wp_117"),
    (   234.024,     -25.729,     1.47, "b2b_wp_118"),
    (   236.400,     -25.668,     1.81, "b2b_wp_119"),
    (   238.833,     -25.591,     1.70, "b2b_wp_120"),
    (   241.264,     -25.519,     5.40, "b2b_wp_121"),
    (   243.674,     -25.291,     7.10, "b2b_wp_122"),
    (   246.065,     -24.993,     0.76, "b2b_wp_123"),
    (   248.413,     -24.962,    -1.24, "b2b_wp_124"),
    (   250.727,     -25.012,     5.62, "b2b_wp_125"),
    (   252.750,     -24.813,    30.53, "b2b_wp_126"),
    (   254.739,     -23.640,    54.51, "b2b_wp_127"),
    (   255.991,     -21.884,    63.21, "b2b_wp_128"),
    (   257.054,     -19.779,    64.11, "b2b_wp_129"),
    (   258.143,     -17.535,    63.82, "b2b_wp_130"),
    (   259.223,     -15.338,    66.32, "b2b_wp_131"),
    (   260.217,     -13.071,    66.39, "b2b_wp_132"),
    (   261.196,     -10.831,    65.17, "b2b_wp_133"),
    (   262.207,      -8.646,    65.28, "b2b_wp_134"),
    (   263.219,      -6.448,    64.35, "b2b_wp_135"),
    (   264.287,      -4.224,    63.75, "b2b_wp_136"),
    (   265.351,      -2.066,    63.84, "b2b_wp_137"),
    (   266.440,       0.151,    62.73, "b2b_wp_138"),
    (   267.557,       2.318,    63.77, "b2b_wp_139"),
    (   268.603,       4.441,    64.25, "b2b_wp_140"),
    (   269.667,       6.647,    65.49, "b2b_wp_141"),
    (   270.668,       8.842,    65.73, "b2b_wp_142"),
    (   271.678,      11.082,    65.71, "b2b_wp_143"),
    (   272.684,      13.311,    65.39, "b2b_wp_144"),
    (   273.681,      15.488,    64.43, "b2b_wp_145"),
    (   274.732,      17.685,    64.73, "b2b_wp_146"),
    (   275.747,      19.835,    65.40, "b2b_wp_147"),
    (   276.730,      21.982,    67.46, "b2b_wp_148"),
    (   277.644,      24.184,    68.54, "b2b_wp_149"),
    (   278.519,      26.410,    69.50, "b2b_wp_150"),
    (   279.366,      28.675,    69.53, "b2b_wp_151"),
    (   280.259,      31.067,    67.27, "b2b_wp_152"),
    (   281.256,      33.447,    63.05, "b2b_wp_153"),
    (   282.414,      35.725,    61.56, "b2b_wp_154"),
    (   283.582,      37.882,    60.26, "b2b_wp_155"),
    (   284.790,      39.996,    59.75, "b2b_wp_156"),
    (   286.000,      42.071,    60.34, "b2b_wp_157"),
    (   287.196,      44.171,    57.99, "b2b_wp_158"),
    (   288.457,      46.188,    56.18, "b2b_wp_159"),
    (   289.732,      48.091,    56.72, "b2b_wp_160"),
    (   291.054,      50.105,    41.59, "b2b_wp_161"),
    (   296.521,      54.957,    19.77, "b2b_wp_162"),
    (   298.594,      55.702,     7.67, "b2b_wp_163"),
    (   300.644,      55.978,     2.78, "b2b_wp_164"),
    (   302.683,      56.077,    17.87, "b2b_wp_165"),
    (   304.777,      56.752,    25.76, "b2b_wp_166"),
    (   306.779,      57.718,    32.18, "b2b_wp_167"),
    (   308.597,      58.862,    43.18, "b2b_wp_168"),
    (   310.192,      60.359,    63.66, "b2b_wp_169"),
    (   311.104,      62.201,    67.81, "b2b_wp_170"),
    (   312.097,      64.636,    93.08, "b2b_wp_171"),
    (   311.989,      66.641,   103.84, "b2b_wp_172"),
    (   311.489,      68.671,   117.94, "b2b_wp_173"),
    (   310.476,      70.581,   143.57, "b2b_wp_174"),
    (   308.593,      71.971,   151.98, "b2b_wp_175"),
    (   306.588,      73.038,   113.28, "b2b_wp_176"),
    (   305.140,      76.403,    51.51, "b2b_wp_177"),
    (   306.673,      78.331,    48.47, "b2b_wp_178"),
    (   308.260,      80.123,    41.38, "b2b_wp_179"),
    (   310.050,      81.700,    31.95, "b2b_wp_180"),
    (   312.014,      82.925,    24.18, "b2b_wp_181"),
    (   314.148,      83.883,    16.66, "b2b_wp_182"),
    (   316.394,      84.555,    11.31, "b2b_wp_183"),
    (   318.684,      85.013,     8.85, "b2b_wp_184"),
    (   321.066,      85.384,     4.97, "b2b_wp_185"),
    (   323.491,      85.595,     2.78, "b2b_wp_186"),
    (   325.986,      85.716,     0.55, "b2b_wp_187"),
    (   328.471,      85.740,     0.09, "b2b_wp_188"),
    (   330.907,      85.744,     1.24, "b2b_wp_189"),
    (   333.348,      85.797,     2.28, "b2b_wp_190"),
    (   335.789,      85.894,     2.24, "b2b_wp_191"),
    (   338.221,      85.989,    -0.99, "b2b_wp_192"),
    (   340.656,      85.947,    -2.08, "b2b_wp_193"),
    (   343.139,      85.857,     0.93, "b2b_wp_194"),
    (   345.596,      85.897,     4.52, "b2b_wp_195"),
    (   348.038,      86.090,     8.30, "b2b_wp_196"),
    (   350.479,      86.446,     8.12, "b2b_wp_197"),
    (   352.884,      86.789,     5.88, "b2b_wp_198"),
    (   355.362,      87.044,     3.85, "b2b_wp_199"),
    (   357.858,      87.212,     3.76, "b2b_wp_200"),
    (   360.247,      87.369,   -24.72, "b2b_wp_201"),
    (   362.938,      86.130,    16.13, "b2b_wp_204"),
    (   365.071,      86.747,    16.13, "b2b_wp_205"),
]



# Return checkpoints: reverse of outbound with flipped yaw
RETURN_CHECKPOINTS = [
    (   365.071,      86.747,  -163.87, "ret_b2b_wp_205"),
    (   362.938,      86.130,   155.28, "ret_b2b_wp_204"),
    (   360.247,      87.369,  -176.24, "ret_b2b_wp_201"),
    (   357.858,      87.212,  -176.15, "ret_b2b_wp_200"),
    (   355.362,      87.044,  -174.12, "ret_b2b_wp_199"),
    (   352.884,      86.789,  -171.88, "ret_b2b_wp_198"),
    (   350.479,      86.446,  -171.70, "ret_b2b_wp_197"),
    (   348.038,      86.090,  -175.48, "ret_b2b_wp_196"),
    (   345.596,      85.897,  -179.07, "ret_b2b_wp_195"),
    (   343.139,      85.857,   177.92, "ret_b2b_wp_194"),
    (   340.656,      85.947,   179.01, "ret_b2b_wp_193"),
    (   338.221,      85.989,  -177.76, "ret_b2b_wp_192"),
    (   335.789,      85.894,  -177.72, "ret_b2b_wp_191"),
    (   333.348,      85.797,  -178.76, "ret_b2b_wp_190"),
    (   330.907,      85.744,  -179.91, "ret_b2b_wp_189"),
    (   328.471,      85.740,  -179.45, "ret_b2b_wp_188"),
    (   325.986,      85.716,  -177.22, "ret_b2b_wp_187"),
    (   323.491,      85.595,  -175.03, "ret_b2b_wp_186"),
    (   321.066,      85.384,  -171.15, "ret_b2b_wp_185"),
    (   318.684,      85.013,  -168.69, "ret_b2b_wp_184"),
    (   316.394,      84.555,  -163.34, "ret_b2b_wp_183"),
    (   314.148,      83.883,  -155.82, "ret_b2b_wp_182"),
    (   312.014,      82.925,  -148.05, "ret_b2b_wp_181"),
    (   310.050,      81.700,  -138.62, "ret_b2b_wp_180"),
    (   308.260,      80.123,  -131.53, "ret_b2b_wp_179"),
    (   306.673,      78.331,  -128.49, "ret_b2b_wp_178"),
    (   305.140,      76.403,   -66.72, "ret_b2b_wp_177"),
    (   306.588,      73.038,   -28.02, "ret_b2b_wp_176"),
    (   308.593,      71.971,   -36.43, "ret_b2b_wp_175"),
    (   310.476,      70.581,   -62.06, "ret_b2b_wp_174"),
    (   311.489,      68.671,   -76.16, "ret_b2b_wp_173"),
    (   311.989,      66.641,   -86.92, "ret_b2b_wp_172"),
    (   312.097,      64.636,  -112.19, "ret_b2b_wp_171"),
    (   311.104,      62.201,  -116.34, "ret_b2b_wp_170"),
    (   310.192,      60.359,  -136.82, "ret_b2b_wp_169"),
    (   308.597,      58.862,  -147.82, "ret_b2b_wp_168"),
    (   306.779,      57.718,  -154.24, "ret_b2b_wp_167"),
    (   304.777,      56.752,  -162.13, "ret_b2b_wp_166"),
    (   302.683,      56.077,  -177.22, "ret_b2b_wp_165"),
    (   300.644,      55.978,  -172.33, "ret_b2b_wp_164"),
    (   298.594,      55.702,  -160.23, "ret_b2b_wp_163"),
    (   296.521,      54.957,  -138.41, "ret_b2b_wp_162"),
    (   291.054,      50.105,  -123.28, "ret_b2b_wp_161"),
    (   289.732,      48.091,  -123.82, "ret_b2b_wp_160"),
    (   288.457,      46.188,  -122.01, "ret_b2b_wp_159"),
    (   287.196,      44.171,  -119.66, "ret_b2b_wp_158"),
    (   286.000,      42.071,  -120.25, "ret_b2b_wp_157"),
    (   284.790,      39.996,  -119.74, "ret_b2b_wp_156"),
    (   283.582,      37.882,  -118.44, "ret_b2b_wp_155"),
    (   282.414,      35.725,  -116.95, "ret_b2b_wp_154"),
    (   281.256,      33.447,  -112.73, "ret_b2b_wp_153"),
    (   280.259,      31.067,  -110.47, "ret_b2b_wp_152"),
    (   279.366,      28.675,  -110.50, "ret_b2b_wp_151"),
    (   278.519,      26.410,  -111.46, "ret_b2b_wp_150"),
    (   277.644,      24.184,  -112.54, "ret_b2b_wp_149"),
    (   276.730,      21.982,  -114.60, "ret_b2b_wp_148"),
    (   275.747,      19.835,  -115.27, "ret_b2b_wp_147"),
    (   274.732,      17.685,  -115.57, "ret_b2b_wp_146"),
    (   273.681,      15.488,  -114.61, "ret_b2b_wp_145"),
    (   272.684,      13.311,  -114.29, "ret_b2b_wp_144"),
    (   271.678,      11.082,  -114.27, "ret_b2b_wp_143"),
    (   270.668,       8.842,  -114.51, "ret_b2b_wp_142"),
    (   269.667,       6.647,  -115.75, "ret_b2b_wp_141"),
    (   268.603,       4.441,  -116.23, "ret_b2b_wp_140"),
    (   267.557,       2.318,  -117.27, "ret_b2b_wp_139"),
    (   266.440,       0.151,  -116.16, "ret_b2b_wp_138"),
    (   265.351,      -2.066,  -116.25, "ret_b2b_wp_137"),
    (   264.287,      -4.224,  -115.65, "ret_b2b_wp_136"),
    (   263.219,      -6.448,  -114.72, "ret_b2b_wp_135"),
    (   262.207,      -8.646,  -114.83, "ret_b2b_wp_134"),
    (   261.196,     -10.831,  -113.61, "ret_b2b_wp_133"),
    (   260.217,     -13.071,  -113.68, "ret_b2b_wp_132"),
    (   259.223,     -15.338,  -116.18, "ret_b2b_wp_131"),
    (   258.143,     -17.535,  -115.89, "ret_b2b_wp_130"),
    (   257.054,     -19.779,  -116.79, "ret_b2b_wp_129"),
    (   255.991,     -21.884,  -125.49, "ret_b2b_wp_128"),
    (   254.739,     -23.640,  -149.47, "ret_b2b_wp_127"),
    (   252.750,     -24.813,  -174.38, "ret_b2b_wp_126"),
    (   250.727,     -25.012,   178.76, "ret_b2b_wp_125"),
    (   248.413,     -24.962,  -179.24, "ret_b2b_wp_124"),
    (   246.065,     -24.993,  -172.90, "ret_b2b_wp_123"),
    (   243.674,     -25.291,  -174.60, "ret_b2b_wp_122"),
    (   241.264,     -25.519,  -178.30, "ret_b2b_wp_121"),
    (   238.833,     -25.591,  -178.19, "ret_b2b_wp_120"),
    (   236.400,     -25.668,  -178.53, "ret_b2b_wp_119"),
    (   234.024,     -25.729,  -175.36, "ret_b2b_wp_118"),
    (   231.586,     -25.927,  -173.85, "ret_b2b_wp_117"),
    (   229.153,     -26.189,  -174.07, "ret_b2b_wp_116"),
    (   226.690,     -26.445,  -170.20, "ret_b2b_wp_115"),
    (   224.327,     -26.853,  -167.21, "ret_b2b_wp_114"),
    (   222.002,     -27.381,  -163.14, "ret_b2b_wp_113"),
    (   219.649,     -28.094,  -157.60, "ret_b2b_wp_112"),
    (   217.242,     -29.086,  -155.81, "ret_b2b_wp_111"),
    (   215.410,     -29.909,  -152.95, "ret_b2b_wp_110"),
    (   213.583,     -30.842,  -151.30, "ret_b2b_wp_109"),
    (   211.313,     -32.085,  -153.07, "ret_b2b_wp_108"),
    (   209.256,     -33.130,  -157.29, "ret_b2b_wp_107"),
    (   207.118,     -34.025,  -159.33, "ret_b2b_wp_106"),
    (   204.854,     -34.879,  -155.24, "ret_b2b_wp_105"),
    (   202.556,     -35.939,  -144.62, "ret_b2b_wp_104"),
    (   200.469,     -37.421,  -128.40, "ret_b2b_wp_103"),
    (   198.964,     -39.320,  -114.95, "ret_b2b_wp_102"),
    (   197.949,     -41.502,  -107.98, "ret_b2b_wp_101"),
    (   197.179,     -43.875,  -107.22, "ret_b2b_wp_100"),
    (   196.481,     -46.127,  -112.93, "ret_b2b_wp_099"),
    (   195.582,     -48.252,  -120.29, "ret_b2b_wp_098"),
    (   194.444,     -50.200,  -127.84, "ret_b2b_wp_097"),
    (   193.104,     -51.925,  -130.24, "ret_b2b_wp_096"),
    (   191.645,     -53.649,  -132.81, "ret_b2b_wp_095"),
    (   190.088,     -55.330,  -140.80, "ret_b2b_wp_094"),
    (   188.304,     -56.785,  -156.61, "ret_b2b_wp_093"),
    (   186.304,     -57.650,  -175.39, "ret_b2b_wp_092"),
    (   184.059,     -57.831,   167.12, "ret_b2b_wp_091"),
    (   181.784,     -57.311,   158.64, "ret_b2b_wp_090"),
    (   180.001,     -55.161,   134.53, "ret_b2b_wp_089"),
    (   177.823,     -53.284,   137.05, "ret_b2b_wp_088"),
    (   176.100,     -51.529,   129.64, "ret_b2b_wp_087"),
    (   174.400,     -49.152,   123.18, "ret_b2b_wp_086"),
    (   172.750,     -46.700,   135.48, "ret_b2b_wp_085"),
    (   170.900,     -45.500,   166.17, "ret_b2b_wp_084"),
    (   168.750,     -45.700,  -162.78, "ret_b2b_wp_083"),
    (   166.641,     -46.875,  -149.77, "ret_b2b_wp_082"),
    (   164.403,     -48.242,  -148.44, "ret_b2b_wp_081"),
    (   161.950,     -50.050,  -158.09, "ret_b2b_wp_080"),
    (   159.650,     -50.350,   168.78, "ret_b2b_wp_079"),
    (   157.300,     -49.000,   138.78, "ret_b2b_wp_078"),
    (   155.552,     -46.717,   123.11, "ret_b2b_wp_077"),
    (   154.217,     -44.132,   120.50, "ret_b2b_wp_076"),
    (   152.903,     -42.217,   125.09, "ret_b2b_wp_075"),
    (   151.200,     -39.837,   123.66, "ret_b2b_wp_074"),
    (   149.653,     -37.338,   123.88, "ret_b2b_wp_073"),
    (   147.925,     -34.960,   124.52, "ret_b2b_wp_072"),
    (   146.322,     -32.495,   122.79, "ret_b2b_wp_071"),
    (   144.740,     -30.016,   126.28, "ret_b2b_wp_070"),
    (   143.366,     -28.144,   123.70, "ret_b2b_wp_069"),
    (   142.038,     -26.153,   123.28, "ret_b2b_wp_068"),
    (   140.684,     -24.090,   123.66, "ret_b2b_wp_067"),
    (   139.313,     -22.031,   124.23, "ret_b2b_wp_066"),
    (   137.899,     -19.953,   125.85, "ret_b2b_wp_065"),
    (   136.431,     -17.921,   124.96, "ret_b2b_wp_064"),
    (   135.034,     -15.923,   123.97, "ret_b2b_wp_063"),
    (   133.687,     -13.924,   122.96, "ret_b2b_wp_062"),
    (   132.378,     -11.905,   122.56, "ret_b2b_wp_061"),
    (   131.067,      -9.852,   124.35, "ret_b2b_wp_060"),
    (   129.690,      -7.837,   124.91, "ret_b2b_wp_059"),
    (   128.283,      -5.821,   125.92, "ret_b2b_wp_058"),
    (   126.834,      -3.821,   129.86, "ret_b2b_wp_057"),
    (   125.265,      -1.942,   136.51, "ret_b2b_wp_056"),
    (   123.513,      -0.280,   146.00, "ret_b2b_wp_055"),
    (   121.439,       1.119,   157.09, "ret_b2b_wp_054"),
    (   119.047,       2.130,   164.07, "ret_b2b_wp_053"),
    (   116.560,       2.840,   168.47, "ret_b2b_wp_052"),
    (   114.085,       3.345,   172.77, "ret_b2b_wp_051"),
    (   111.658,       3.653,   176.36, "ret_b2b_wp_050"),
    (   109.251,       3.806,  -177.75, "ret_b2b_wp_049"),
    (   106.838,       3.711,  -172.14, "ret_b2b_wp_048"),
    (   104.345,       3.367,  -168.40, "ret_b2b_wp_047"),
    (   101.953,       2.876,  -169.02, "ret_b2b_wp_046"),
    (    99.593,       2.418,  -171.98, "ret_b2b_wp_045"),
    (    97.215,       2.083,  -175.05, "ret_b2b_wp_044"),
    (    94.790,       1.873,  -177.05, "ret_b2b_wp_043"),
    (    92.341,       1.747,  -174.89, "ret_b2b_wp_042"),
    (    89.960,       1.534,  -172.13, "ret_b2b_wp_041"),
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
