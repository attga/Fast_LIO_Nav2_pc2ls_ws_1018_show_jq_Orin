"""验证固定关节拓扑、安装方向、旧入口兼容及未知外参拒绝。"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
import xacro
from ml_bringup.description import robot_description

SHARE = Path(__file__).resolve().parents[1]


def model(path=None, **kwargs):
    return ET.fromstring(robot_description(path or SHARE / 'config/robot/rear_axle.yaml',
                                          share_directory=SHARE, **kwargs))


def joint(tree, child):
    return next(j for j in tree.findall('joint') if j.find('child').get('link') == child)


def test_rear_axle_tree_has_one_parent_and_no_dynamic_edges():
    tree = model()
    children = [j.find('child').get('link') for j in tree.findall('joint')]
    assert len(children) == len(set(children)) == 4
    links = {link.get('name') for link in tree.findall('link')}
    assert links - set(children) == {'base_link'}
    assert not {'map', 'odom'} & links
    assert all(j.get('type') == 'fixed' for j in tree.findall('joint'))
    for child in ('body', 'gyro_link', 'livox_imu_link'):
        mount = joint(tree, child)
        assert mount.find('parent').get('link') == 'base_link'
        assert [float(v) for v in mount.find('origin').get('xyz').split()] == [0., 0., 0.]
        expected_yaw = math.pi / 2 if child == 'livox_imu_link' else 0.
        assert [float(v) for v in mount.find('origin').get('rpy').split()] == pytest.approx([0., 0., expected_yaw])


def test_yesense_can_be_disabled():
    assert 'gyro_link' not in {link.get('name') for link in model(enable_yesense=False).findall('link')}


def test_legacy_launch_overrides_keep_units_and_axis_order():
    tree = model(imu_mount='[1,2,3,0.1,0.2,0.3]', wheel_mount='[4,5,6,0.4,0.5,0.6]')
    for child, xyz, rpy in [('gyro_link', [1,2,3], [.1,.2,.3]), ('body',[4,5,6],[.4,.5,.6])]:
        origin = joint(tree, child).find('origin')
        assert list(map(float, origin.get('xyz').split())) == xyz
        assert list(map(float, origin.get('rpy').split())) == rpy


def test_explicit_unknown_mounts_are_rejected(tmp_path):
    config = yaml.safe_load((SHARE / 'config/robot/rear_axle.yaml').read_text())
    config['mounts']['livox_imu']['xyz'] = [None, None, None]
    path = tmp_path / 'unknown.yaml'
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match='measured'):
        model(path)


def test_rear_axle_temporary_zero_translation_keeps_known_yaw():
    tree = model(SHARE / 'config/robot/rear_axle.yaml')
    origin = joint(tree, 'livox_imu_link').find('origin')
    assert list(map(float, origin.get('xyz').split())) == [0., 0., 0.]
    assert list(map(float, origin.get('rpy').split())) == pytest.approx([0., 0., math.pi/2])


def test_rear_axle_yaw_maps_sensor_forward_to_vehicle_left(tmp_path):
    config = yaml.safe_load((SHARE / 'config/robot/rear_axle.yaml').read_text())
    # 仅测试用的虚构安装位置，不代表实车测量。
    config['mounts']['livox_imu']['xyz'] = [1.0, .2, .8]
    config['mounts']['yesense']['xyz'] = [.3, .1, .5]
    path = tmp_path / 'mounts.yaml'
    path.write_text(yaml.safe_dump(config))
    tree = model(path)
    origin = joint(tree, 'livox_imu_link').find('origin')
    roll, pitch, yaw = map(float, origin.get('rpy').split())
    assert roll == pitch == 0
    assert math.cos(yaw) == pytest.approx(0, abs=1e-12)
    assert math.sin(yaw) == pytest.approx(1)
    assert list(map(float, origin.get('xyz').split())) == [1., .2, .8]
    assert joint(tree, 'livox_lidar_link').find('parent').get('link') == 'livox_imu_link'


@pytest.mark.parametrize('override', ['[0,0,0]', '[0,0,0,0,0,null]', '[0,0,0,0,0,True]', '[0,0,0,0,0,1e999]'])
def test_invalid_legacy_override_is_rejected(override):
    with pytest.raises((ValueError, SyntaxError)):
        model(imu_mount=override)


def test_xacro_can_expand_directly_with_rear_axle_defaults():
    # 直接使用 Xacro 也应读到同一份 YAML，不能依赖 Python 注入安装数值。
    direct = ET.fromstring(xacro.process_file(str(SHARE / 'urdf/robot.urdf.xacro')).toxml())
    assert ET.tostring(direct) == ET.tostring(model())


def test_direct_xacro_uses_selected_yaml_and_sensor_switch():
    path = SHARE / 'config/robot/rear_axle.yaml'
    direct = ET.fromstring(xacro.process_file(str(SHARE / 'urdf/robot.urdf.xacro'), mappings={
        'mounts_file': str(path), 'enable_yesense': 'false'}).toxml())
    assert ET.tostring(direct) == ET.tostring(model(path, enable_yesense=False))


def test_legacy_yaw_override_preserves_other_mount_axes(tmp_path):
    config = yaml.safe_load((SHARE / 'config/robot/rear_axle.yaml').read_text())
    config['mounts']['livox_imu'] = {'xyz': [1., .2, .8], 'rpy': [.1, .2, .3]}
    path = tmp_path / 'mounts.yaml'
    path.write_text(yaml.safe_dump(config))
    origin = joint(model(path, lidar_mount_yaw='0.0'), 'livox_imu_link').find('origin')
    assert list(map(float, origin.get('xyz').split())) == [1., .2, .8]
    assert list(map(float, origin.get('rpy').split())) == [.1, .2, 0.]


@pytest.mark.parametrize('yaw', ['nan', 'inf', '-inf', 'bad'])
def test_invalid_lidar_yaw_is_rejected(yaw):
    with pytest.raises(ValueError):
        model(lidar_mount_yaw=yaw)
