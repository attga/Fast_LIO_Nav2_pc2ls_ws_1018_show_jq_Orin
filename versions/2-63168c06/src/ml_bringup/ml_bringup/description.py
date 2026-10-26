"""校验安装参数并展开 Xacro；不估计安装外参，不发布动态 TF。"""
import ast
import math
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory


def finite_vector(value, size, name):
    if isinstance(value, str):
        value = ast.literal_eval(value)
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f'{name} requires {size} values')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
           not math.isfinite(v) for v in value):
        raise ValueError(f'{name}: fill all measured values with finite numbers; null is unknown')
    return [float(v) for v in value]


def robot_description(mounts_file, enable_yesense=True, imu_mount=None,
                      wheel_mount=None, share_directory=None):
    share = Path(share_directory or get_package_share_directory('ml_bringup'))
    mounts_file = Path(mounts_file).expanduser().resolve()
    with mounts_file.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or config.get('base_reference') not in ('livox_imu', 'rear_axle'):
        raise ValueError('base_reference must be livox_imu or rear_axle')
    mounts = config['mounts']
    if not isinstance(mounts, dict) or set(mounts) != {'wheel', 'yesense', 'livox_imu', 'livox_lidar'}:
        raise ValueError('mounts must define wheel, yesense, livox_imu and livox_lidar')
    mappings = {'mounts_file': str(mounts_file),
                'enable_yesense': str(enable_yesense).lower()}
    # 旧命令行的六元列表转为空格分隔；Xacro 负责展开，安装数值只来自 YAML。
    for key, argument, override in (('yesense', 'imu_mount', imu_mount),
                                    ('wheel', 'wheel_mount', wheel_mount)):
        if override not in (None, ''):
            values = finite_vector(override, 6, key)
            mounts[key] = {'xyz': values[:3], 'rpy': values[3:]}
            mappings[argument] = ' '.join(str(v) for v in values)
    for key, mount in mounts.items():
        if key == 'yesense' and not enable_yesense:
            continue
        for field in ('xyz', 'rpy'):
            finite_vector(mount[field], 3, key + '.' + field)
    return xacro.process_file(str(share / 'urdf' / 'robot.urdf.xacro'),
                              mappings=mappings).toxml()
