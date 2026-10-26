#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_patrol.py — 曲线巡航节点 (十一测大圈轨迹)
=========================================
从当前定位起点出发，沿着十一测提取的参考轨迹生成 N 段航点，到达终点 (-55, -18) 后原路返回，循环往复。

航点由参考轨迹自动插值生成：
  num_segments : 单程的航点数量（可调，默认 20）
  end_dwell    : 端点停留时间（s）
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


# 从十一测轨迹中提取的平滑参考关键点 (起点 -> 终点)
REFERENCE_PATH = [
    (2.070, 2.420),      # 起点
    (2.111, 5.579),
    (2.108, 11.163),
    (1.763, 16.665),
    (0.070, 21.997),
    (-4.050, 25.667),
    (-9.507, 25.637),
    (-15.005, 24.678),
    (-20.623, 24.374),
    (-26.237, 24.265),
    (-31.806, 23.938),
    (-37.178, 23.468),
    (-42.254, 23.865),
    (-47.334, 23.754),
    (-52.960, 22.336),
    (-56.446, 18.665),
    (-57.504, 13.094),
    (-57.511, 8.093),
    (-57.335, 2.995),
    (-56.870, -2.049),
    (-56.465, -7.505),
    (-55.858, -12.593),
    (-55.258, -17.961)   # 终点附近 (-55, -18)
]


class WaypointPatrol(Node):
    """曲线往返巡航控制器 — 航点自动生成"""

    # ──────────────────────────────────────────────────────────────────
    # 初始化
    # ──────────────────────────────────────────────────────────────────
    def __init__(self):
        super().__init__('waypoint_patrol')

        # ── ROS2 参数声明 ──────────────────────────────────────────────
        # 保留原有的部分参数声明以兼容可能存在的 Launch 参数
        self.declare_parameter('start_x',        2.07)
        self.declare_parameter('start_y',        2.42)
        self.declare_parameter('start_yaw_deg',  89.08)
        self.declare_parameter('path_length',    10.0) 

        # 核心控制参数
        self.declare_parameter('num_segments',   40)     # 单程航点数量（可调参数）
        self.declare_parameter('end_dwell',      5.0)    # 到达两端后停留秒数
        self.declare_parameter('max_loops',      0)      # 最大循环次数，0=无限
        self.declare_parameter('skip_on_failure', True)  # 失败时跳过该点
        self.declare_parameter('max_retries',    2)      # 每点最大重试次数

        # 读取参数
        self.num_segs         = self.get_parameter('num_segments').value
        self.end_dwell        = self.get_parameter('end_dwell').value
        self.max_loops        = self.get_parameter('max_loops').value
        self.skip_on_failure  = self.get_parameter('skip_on_failure').value
        self.max_retries      = self.get_parameter('max_retries').value

        # ── Navigator 初始化 ──────────────────────────────────────────
        self.navigator = BasicNavigator()

        # ── 自动生成曲线往返航点 ──────────────────────────────────────
        self.waypoints_data = self._generate_loop_waypoints(self.num_segs, self.end_dwell)

        self._log_waypoints()

    # ──────────────────────────────────────────────────────────────────
    # 航点生成与插值算法
    # ──────────────────────────────────────────────────────────────────
    def _generate_loop_waypoints(self, num_segments: int, end_dwell: float) -> list:
        """
        根据 REFERENCE_PATH 自动插值生成往返航点。
        格式: [(x, y, yaw_rad, dwell_sec), ...]
        """
        # 1. 计算参考路径的累积长度
        distances = [0.0]
        for i in range(1, len(REFERENCE_PATH)):
            dx = REFERENCE_PATH[i][0] - REFERENCE_PATH[i-1][0]
            dy = REFERENCE_PATH[i][1] - REFERENCE_PATH[i-1][1]
            distances.append(distances[-1] + math.hypot(dx, dy))
        
        total_length = distances[-1]
        step = total_length / num_segments
        
        # 2. 生成去程插值点 (1 到 num_segments)
        fwd_points = []
        for i in range(1, num_segments + 1):
            target_dist = i * step
            # 找到对应线段
            for j in range(1, len(distances)):
                if distances[j] >= target_dist or j == len(distances) - 1:
                    segment_len = distances[j] - distances[j-1]
                    t = 0.0 if segment_len == 0 else (target_dist - distances[j-1]) / segment_len
                    t = max(0.0, min(1.0, t))  # Clamp
                    
                    x = REFERENCE_PATH[j-1][0] + t * (REFERENCE_PATH[j][0] - REFERENCE_PATH[j-1][0])
                    y = REFERENCE_PATH[j-1][1] + t * (REFERENCE_PATH[j][1] - REFERENCE_PATH[j-1][1])
                    
                    # 朝向与当前线段切向对齐
                    yaw = math.atan2(REFERENCE_PATH[j][1] - REFERENCE_PATH[j-1][1], 
                                     REFERENCE_PATH[j][0] - REFERENCE_PATH[j-1][0])
                    
                    dwell = end_dwell if i == num_segments else 0.0
                    fwd_points.append((x, y, yaw, dwell))
                    break

        waypoints = []
        waypoints.extend(fwd_points)

        # 3. 生成返程插值点 (倒序返回，要求车头朝向行驶方向)
        # 返程行驶方向是朝向起点的，因此偏航角是原来的反向
        for i in range(num_segments - 1, -1, -1):
            x, y, orig_yaw, _ = fwd_points[i] if i > 0 else (REFERENCE_PATH[0][0], REFERENCE_PATH[0][1], fwd_points[0][2], 0.0)
            # 返程朝向是相反的
            ret_yaw = orig_yaw - math.pi 
            # 规范化角度到 [-pi, pi]
            ret_yaw = (ret_yaw + math.pi) % (2 * math.pi) - math.pi
            
            dwell = end_dwell if i == 0 else 0.0
            waypoints.append((x, y, ret_yaw, dwell))

        return waypoints

    def _log_waypoints(self):
        """启动时打印所有航点坐标，方便核查。"""
        total = len(self.waypoints_data)
        
        # 计算粗略的总长度
        dist = 0
        for i in range(1, len(REFERENCE_PATH)):
            dist += math.hypot(REFERENCE_PATH[i][0] - REFERENCE_PATH[i-1][0], 
                               REFERENCE_PATH[i][1] - REFERENCE_PATH[i-1][1])

        self.get_logger().info(
            f'\n{"="*62}\n'
            f'  十一测大圈曲线巡航 配置汇总\n'
            f'  起点:      ({REFERENCE_PATH[0][0]:.3f}, {REFERENCE_PATH[0][1]:.3f})\n'
            f'  终点:      ({REFERENCE_PATH[-1][0]:.3f}, {REFERENCE_PATH[-1][1]:.3f})\n'
            f'  单程总长:  约 {dist:.1f} m\n'
            f'  段数:      {self.num_segs} 段\n'
            f'  端点停留:  {self.end_dwell:.1f} s\n'
            f'  总航点数:  {total}（{self.num_segs} 去程 + {self.num_segs} 返程）\n'
            f'  最大循环:  {"无限" if self.max_loops == 0 else self.max_loops}\n'
            f'{"="*62}'
        )
        for idx, (x, y, yaw, dwell) in enumerate(self.waypoints_data):
            phase = '→去程' if idx < self.num_segs else '←返程'
            self.get_logger().info(
                f'  WP-{idx+1:02d}/{total} {phase} '
                f'({x:.3f}, {y:.3f})  Yaw={math.degrees(yaw):.1f}°  '
                f'停留={dwell:.0f}s'
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
        self.get_logger().info('Nav2 已激活 ✔  开始十一测大圈巡航...')

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
