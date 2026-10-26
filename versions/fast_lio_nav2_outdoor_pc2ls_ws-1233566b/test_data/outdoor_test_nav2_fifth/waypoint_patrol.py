#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_patrol.py — 直线往返巡航节点
=========================================
从当前定位起点出发，向机器人正前方生成 N 段直线航点，到达终点后原路返回，循环往复。

航点由参数自动计算，无需手动填写地图坐标：
  start_x, start_y  : 地图坐标系起点（来自 ICP 定位结果）
  start_yaw_deg     : 出发朝向（度），向前方向
  path_length       : 直线总长度（m）
  num_segments      : 均分段数（航点数 = num_segments，不含起点）

使用示例:
  # 使用默认参数（从当前定位起点出发，直行10m，4段2.5m间隔）
  ros2 run my_nav2_config waypoint_patrol.py

  # 自定义参数
  ros2 run my_nav2_config waypoint_patrol.py --ros-args \
      -p start_x:=2.07 -p start_y:=2.42 \
      -p start_yaw_deg:=89.08 \
      -p path_length:=10.0 \
      -p num_segments:=4 \
      -p end_dwell:=5.0 \
      -p max_loops:=0
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


class WaypointPatrol(Node):
    """直线往返巡航控制器 — 航点自动生成"""

    # ──────────────────────────────────────────────────────────────────
    # 初始化
    # ──────────────────────────────────────────────────────────────────
    def __init__(self):
        super().__init__('waypoint_patrol')

        # ── ROS2 参数声明 ──────────────────────────────────────────────
        # 起始位姿（来自 ICP 定位结果，单位：m / 度）
        self.declare_parameter('start_x',        2.07)   # 地图 X（m）
        self.declare_parameter('start_y',        2.42)   # 地图 Y（m）
        self.declare_parameter('start_yaw_deg',  85.08)  # 前进朝向（deg，map 坐标系）

        # 路径参数
        self.declare_parameter('path_length',    10.0)   # 直线总长度（m）
        self.declare_parameter('num_segments',   9)      # 均分段数
        # ┌─────────────────────────────────────────────────────┐
        # │  num_segments 选取建议（10m 直线）                   │
        # │  4  → 间距 2.5m，5 个航点 ← 推荐（适合 MPPI 视野）  │
        # │  10 → 间距 1.0m，11 个航点（路径更密但易抖动）       │
        # └─────────────────────────────────────────────────────┘

        # 行为参数
        self.declare_parameter('end_dwell',       5.0)   # 到达两端后停留秒数
        self.declare_parameter('max_loops',       0)     # 最大循环次数，0=无限
        self.declare_parameter('skip_on_failure', True)  # 失败时跳过该点
        self.declare_parameter('max_retries',     2)     # 每点最大重试次数

        # 读取参数
        start_x        = self.get_parameter('start_x').value
        start_y        = self.get_parameter('start_y').value
        start_yaw_deg  = self.get_parameter('start_yaw_deg').value
        path_length    = self.get_parameter('path_length').value
        self.num_segs  = self.get_parameter('num_segments').value
        end_dwell      = self.get_parameter('end_dwell').value
        self.max_loops        = self.get_parameter('max_loops').value
        self.skip_on_failure  = self.get_parameter('skip_on_failure').value
        self.max_retries      = self.get_parameter('max_retries').value

        # ── Navigator 初始化 ──────────────────────────────────────────
        self.navigator = BasicNavigator()

        # ── 自动生成直线往返航点 ──────────────────────────────────────
        self.waypoints_data = self._generate_straight_waypoints(
            start_x, start_y, start_yaw_deg,
            path_length, self.num_segs, end_dwell
        )

        self._log_waypoints(start_x, start_y, start_yaw_deg,
                             path_length, self.num_segs, end_dwell)

    # ──────────────────────────────────────────────────────────────────
    # 航点生成
    # ──────────────────────────────────────────────────────────────────
    def _generate_straight_waypoints(
            self,
            start_x: float, start_y: float, start_yaw_deg: float,
            path_length: float, num_segments: int, end_dwell: float
    ) -> list:
        """
        生成直线往返航点列表。格式: [(x, y, yaw_rad, dwell_sec), ...]

        去程: 从 start 沿 yaw_fwd 方向每 seg_len 设一个航点，共 num_segments 个。
              只在最后一个（终点）停留 end_dwell 秒。
        返程: 原路反向，每个中间点不停留，回到起点停留 end_dwell 秒。
        """
        yaw_fwd_rad = math.radians(start_yaw_deg)
        yaw_ret_rad = math.radians(start_yaw_deg - 180.0)
        cos_f = math.cos(yaw_fwd_rad)
        sin_f = math.sin(yaw_fwd_rad)
        seg = path_length / num_segments

        waypoints = []

        # ── 去程：i = 1 … num_segments ──────────────────────────────
        for i in range(1, num_segments + 1):
            d = i * seg
            wx = start_x + d * cos_f
            wy = start_y + d * sin_f
            dwell = end_dwell if i == num_segments else 0.0
            waypoints.append((wx, wy, yaw_fwd_rad, dwell))

        # ── 返程：i = num_segments-1 … 0（含起点）───────────────────
        for i in range(num_segments - 1, -1, -1):
            d = i * seg
            wx = start_x + d * cos_f
            wy = start_y + d * sin_f
            dwell = end_dwell if i == 0 else 0.0
            waypoints.append((wx, wy, yaw_ret_rad, dwell))

        return waypoints

    def _log_waypoints(self, start_x, start_y, start_yaw_deg,
                        path_length, num_segments, end_dwell):
        """启动时打印所有航点坐标，方便核查。"""
        seg = path_length / num_segments
        total = len(self.waypoints_data)
        self.get_logger().info(
            f'\n{"="*62}\n'
            f'  直线往返巡航 配置汇总\n'
            f'  起点:      ({start_x:.3f}, {start_y:.3f}), Yaw={start_yaw_deg:.2f}°\n'
            f'  路径长度:  {path_length:.1f} m\n'
            f'  段数:      {num_segments} 段（间距 {seg:.2f} m/段）\n'
            f'  端点停留:  {end_dwell:.1f} s\n'
            f'  总航点数:  {total}（{num_segments} 去程 + {num_segments} 返程）\n'
            f'  最大循环:  {"无限" if self.max_loops == 0 else self.max_loops}\n'
            f'{"="*62}'
        )
        for idx, (x, y, yaw, dwell) in enumerate(self.waypoints_data):
            phase = '→去程' if idx < num_segments else '←返程'
            dist  = (idx + 1) * seg if idx < num_segments else (num_segments * 2 - idx) * seg
            self.get_logger().info(
                f'  WP-{idx+1:02d}/{total} {phase} '
                f'({x:.3f}, {y:.3f})  Yaw={math.degrees(yaw):.1f}°  '
                f'd={dist:.1f}m  停留={dwell:.0f}s'
            )

    # ──────────────────────────────────────────────────────────────────
    # 工具方法
    # ──────────────────────────────────────────────────────────────────
    def create_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        """构造 PoseStamped 消息（仅绕 Z 轴旋转）。"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def navigate_to_point(self, x: float, y: float, yaw: float,
                          wp_name: str) -> bool:
        """导航至单个目标点，含重试逻辑与实时距离反馈。"""
        for attempt in range(1, self.max_retries + 1):
            suffix = (f' (尝试 {attempt}/{self.max_retries})'
                      if self.max_retries > 1 else '')
            self.get_logger().info(
                f'[{wp_name}] 导航至 ({x:.3f}, {y:.3f}), '
                f'Yaw={math.degrees(yaw):.1f}°{suffix}')

            self.navigator.goToPose(self.create_pose(x, y, yaw))

            while not self.navigator.isTaskComplete():
                fb = self.navigator.getFeedback()
                if fb and hasattr(fb, 'distance_remaining'):
                    self.get_logger().info(
                        f'[{wp_name}] 剩余: {fb.distance_remaining:.2f} m',
                        throttle_duration_sec=3.0)
                time.sleep(0.1)

            result = self.navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'[{wp_name}] ✅ 到达')
                return True
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f'[{wp_name}] ⚠️  被外部取消')
                return False
            else:  # FAILED
                self.get_logger().error(f'[{wp_name}] ❌ 失败')
                if attempt < self.max_retries:
                    self.get_logger().info(f'[{wp_name}] 2s 后重试...')
                    time.sleep(2.0)

        return False

    # ──────────────────────────────────────────────────────────────────
    # 主循环
    # ──────────────────────────────────────────────────────────────────
    def run_patrol(self):
        """主巡航循环。"""
        self.get_logger().info('等待 Nav2 激活（bt_navigator）...')
        self.navigator._waitForNodeToActivate('bt_navigator')
        self.get_logger().info('Nav2 已激活 ✔  开始直线往返巡航...')

        total_wps  = len(self.waypoints_data)
        loop_count = 0

        while rclpy.ok():
            loop_count += 1

            if self.max_loops > 0 and loop_count > self.max_loops:
                self.get_logger().info(f'已完成 {self.max_loops} 次循环，结束。')
                break

            self.get_logger().info(
                f'\n{"="*50}\n'
                f'  第 #{loop_count} 圈'
                f'{f"/{self.max_loops}" if self.max_loops > 0 else " (无限循环)"} 启动\n'
                f'{"="*50}')

            for idx, (x, y, yaw, dwell) in enumerate(self.waypoints_data):
                phase = '→去程' if idx < self.num_segs else '←返程'
                wp_name = f'WP-{idx+1:02d}/{total_wps} {phase}'

                ok = self.navigate_to_point(x, y, yaw, wp_name)

                if ok:
                    if dwell > 0:
                        self.get_logger().info(
                            f'[{wp_name}] 🕐 端点停留 {dwell:.0f} s...')
                        time.sleep(dwell)
                else:
                    if not rclpy.ok():
                        return
                    if self.skip_on_failure:
                        self.get_logger().warn(f'[{wp_name}] 跳过，继续下一点')
                        continue
                    else:
                        self.get_logger().error(f'[{wp_name}] 停止巡航')
                        return

            self.get_logger().info(f'第 #{loop_count} 圈完成 ✔')

        self.get_logger().info('巡航结束。')


# ─────────────────────────────────────────────────────────────────────
# 入口
# ─────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = WaypointPatrol()
    try:
        node.run_patrol()
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C 中断巡航')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
