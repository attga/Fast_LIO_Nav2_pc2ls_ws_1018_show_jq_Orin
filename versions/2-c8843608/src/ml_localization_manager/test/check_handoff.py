#!/usr/bin/env python3
"""独立 ROS 边界回归；需先 source 工作区，在未使用的 ROS_DOMAIN_ID 下运行。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from robot_localization.srv import SetPose


def run_case(executable, output, mode):
    rclpy.init()
    node = rclpy.create_node('manager_handoff_probe')
    counts = dict(trigger=0, set_pose=0, start=0)
    states, events, services = [], [], []
    began = time.monotonic()
    sent = False
    first_trigger_stamp = None
    first_trigger_time = None
    acknowledged = False
    duplicate_stamp = None
    duplicate_until = None
    early_start = False
    local = node.create_publisher(Odometry, '/local_odometry/filter', 10)
    global_odom = node.create_publisher(Odometry, '/global_odometry/filter', 10)
    reloc = node.create_publisher(PoseWithCovarianceStamped, '/relocalized_pose', 10)
    diag = node.create_publisher(DiagnosticStatus, '/ml_localization/status_diagnostic', 10)
    subscription = node.create_subscription(String, '/ml_localization/manager_state',
                                            lambda m: states.append(m.data), 10)

    def trigger(request, response):
        nonlocal sent, first_trigger_stamp, first_trigger_time
        counts['trigger'] += 1
        if mode == 'stale_then_fresh':
            sent = False
            if first_trigger_stamp is None:
                first_trigger_stamp = node.get_clock().now().to_msg()
                first_trigger_time = time.monotonic()
        response.success = True
        return response

    def set_pose(request, response):
        counts['set_pose'] += 1
        return response

    def start(request, response):
        nonlocal acknowledged, early_start
        counts['start'] += 1
        if duplicate_until is not None and time.monotonic() < duplicate_until:
            early_start = True
        response.success = mode != 'reject_once' or counts['start'] > 1
        acknowledged = response.success
        events.append(dict(start_success=response.success, elapsed=time.monotonic() - began))
        return response

    if mode != 'sc_unavailable':
        services.append(node.create_service(Trigger, '/trigger_relocalization', trigger))
    if mode != 'set_pose_unavailable':
        services.append(node.create_service(SetPose, '/global/set_pose', set_pose))
    if mode != 'start_unavailable':
        services.append(node.create_service(Trigger, '/ml_localization/start_tracking', start))

    def tick():
        nonlocal sent, duplicate_stamp, duplicate_until
        stamp = node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.0
        local.publish(odom)
        odom.header.frame_id = 'map'
        if mode == 'global_mismatch':
            odom.pose.pose.position.x = 10.0
        if mode == 'duplicate_global' and counts['set_pose']:
            if duplicate_stamp is None:
                duplicate_stamp = stamp
                duplicate_until = time.monotonic() + 1.0
            if time.monotonic() < duplicate_until:
                odom.header.stamp = duplicate_stamp
        global_odom.publish(odom)
        waiting_to_expire = (mode == 'stale_then_fresh' and counts['trigger'] == 1 and
                             time.monotonic() - first_trigger_time < 1.8)
        if counts['trigger'] and not sent and not waiting_to_expire:
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = (first_trigger_stamp if mode == 'stale_then_fresh' and
                                 counts['trigger'] == 1 else stamp)
            pose.header.frame_id = 'map'
            pose.pose.pose.orientation.w = 1.0
            for i in (0, 7, 14, 21, 28, 35):
                pose.pose.covariance[i] = 0.1
            reloc.publish(pose)
            sent = True
        failed = any(s.startswith('INITIALIZATION_FAILED') for s in states)
        tracking = failed or (acknowledged and mode != 'no_fresh_scan')
        status = DiagnosticStatus()
        status.values = [KeyValue(key='state', value='TRACKING' if tracking else 'INIT'),
                         KeyValue(key='scan_stamp', value=str(node.get_clock().now().nanoseconds * 1e-9))]
        # 正常交接后持续发送旧文本/非法状态，不能掩盖真实心跳超时。
        if mode in ('text_only_status', 'malformed_state') and any(
                s.startswith('NORMAL_TRACKING') and 'startup_complete=1' in s for s in states):
            status.message = 'TRACKING nvtl=3.0'
            status.values = [] if mode == 'text_only_status' else [
                KeyValue(key='state', value='TRACKING_FAILED')]
        diag.publish(status)

    timer = node.create_timer(0.05, tick)
    # 只缩短本测试实例的交接窗口和 SC 重试间隔，不修改正式参数。
    command = [executable, '--ros-args', '-p', 'recovery_timeout:=3.0',
               '-p', 'reloc_request_interval:=1.0']
    if mode == 'stale_then_fresh':
        command = [executable, '--ros-args', '-p', 'recovery_timeout:=3.0',
                   '-p', 'reloc_request_interval:=120.0', '-p', 'reloc_rejection_retry_delay:=0.5',
                   '-p', 'max_reloc_age:=1.5']  # 固定短超时，保留本例的过期/重试时序。
    if mode in ('text_only_status', 'malformed_state'):
        command += ['-p', 'icp_status_timeout:=1.0']
    process = None
    try:
        with (output / (mode + '.log')).open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            while time.monotonic() - began < 8.0:
                rclpy.spin_once(node, timeout_sec=0.05)
            alive = process.poll() is None
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
    failed = next((i for i, s in enumerate(states) if s.startswith('INITIALIZATION_FAILED')), None)
    normal = any(s.startswith('NORMAL_TRACKING') and 'startup_complete=1' in s for s in states)
    if mode == 'sc_unavailable':
        passed = counts == dict(trigger=0, set_pose=0, start=0) and not normal and failed is None
        passed = passed and (output / (mode + '.log')).read_text().count('SC service unavailable') >= 2
    elif mode in ('text_only_status', 'malformed_state'):
        passed = normal and failed is None and counts == dict(trigger=1, set_pose=1, start=1)
        passed = passed and any(s.startswith('LOST') and 'icp_heartbeat_stale=1' in s for s in states)
    elif mode == 'stale_then_fresh':
        passed = normal and failed is None and counts == dict(trigger=2, set_pose=1, start=1)
        passed = passed and 'stale reloc pose' in (output / (mode + '.log')).read_text()
    elif mode in ('reject_once', 'duplicate_global'):
        passed = normal and failed is None and counts == dict(trigger=1, set_pose=1,
                    start=2 if mode == 'reject_once' else 1) and not early_start
    else:
        passed = failed is not None and not normal and not any(
            s.startswith('NORMAL_TRACKING') for s in states[failed:])
        passed = passed and counts == dict(trigger=1,
            set_pose=0 if mode == 'set_pose_unavailable' else 1,
            start=1 if mode == 'no_fresh_scan' else 0)
        passed = passed and 'confirmation timed out' in states[failed]
    return dict(case=mode, passed=passed and alive, counts=counts, states=states,
                events=events, start_before_distinct_global=early_start)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', choices=['sc_unavailable', 'set_pose_unavailable', 'start_unavailable',
        'global_mismatch', 'no_fresh_scan', 'reject_once', 'duplicate_global', 'stale_then_fresh', 'text_only_status', 'malformed_state'])
    args = parser.parse_args()
    # 显式要求隔离域，避免默认域测试服务误接正在运行的机器人。
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) == 0:
        parser.error('请设置未使用的非零 ROS_DOMAIN_ID')
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(args.output / 'ros_logs')
    executable = str(Path(get_package_prefix('ml_localization_manager')) /
                     'lib/ml_localization_manager/ml_localization_manager')
    results = []
    modes = [args.case] if args.case else ['sc_unavailable', 'set_pose_unavailable', 'start_unavailable',
                 'global_mismatch', 'no_fresh_scan', 'reject_once', 'duplicate_global', 'stale_then_fresh', 'text_only_status', 'malformed_state']
    for mode in modes:
        result = run_case(executable, args.output, mode)
        results.append(result)
        (args.output / 'results.json').write_text(json.dumps(results, indent=2))
        print(json.dumps(result), flush=True)
    if not all(r['passed'] for r in results):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
