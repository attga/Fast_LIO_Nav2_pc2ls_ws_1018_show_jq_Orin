"""Check the effective sensor inputs assembled by the localization launch."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
SHARE = ROOT / 'ml_bringup'


def parameters(node):
    result = {}
    for item in node['parameters']:
        if isinstance(item, dict):
            result.update(item)
        else:
            for settings in yaml.safe_load(Path(item).read_text()).values():
                result.update(settings['ros__parameters'])
    return result


def prepare_launch(enabled, monkeypatch, tmp_path, delay='0'):
    spec = importlib.util.spec_from_file_location(
        'localization_launch', SHARE / 'launch/localization.launch.py')
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    monkeypatch.setattr(launch, 'get_package_share_directory', lambda name: str(ROOT / name))
    monkeypatch.setattr(launch, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(launch, 'robot_description', lambda *args, **kwargs: '<robot name="test"/>')
    options = dict(ekf_config_directory=str(SHARE / 'config/ekf'), use_sim_time='true',
                   map_directory=str(tmp_path), pose_file='isam_poses.txt',
                   fastlio_log_directory=str(tmp_path / 'logs'),
                   manager_config=str(ROOT / 'ml_localization_manager/config/manager.yaml'),
                   robot_mounts_file=str(SHARE / 'config/robot/rear_axle.yaml'),
                   enable_yesense=str(enabled).lower(), imu_mount='', wheel_mount='',
                   fastlio_config=str(ROOT / 'fast_lio/config/mid360.yaml'),
                   ndt_config=str(ROOT / 'ml_ndt_localization/config/localization.yaml'),
                   relocalization_config=str(ROOT / 'ml_relocalization/config/relocalization.yaml'),
                   rviz='false', manager_start_delay=delay)
    return launch, options


@pytest.mark.parametrize('enabled', [True, False])
def test_dual_ekf_sensor_contract(enabled, monkeypatch, tmp_path):
    launch, options = prepare_launch(enabled, monkeypatch, tmp_path)
    nodes = launch.launch_nodes(SimpleNamespace(launch_configurations=options))
    ekfs = [parameters(n) for n in nodes if n['executable'] == 'ekf_node']
    assert len(ekfs) == 2
    for params in ekfs:
        assert '/wheel_odom' not in params.values()
        assert not any(key.startswith('odom1') for key in params)
        assert params['two_d_mode'] is True
        if enabled:
            assert params['imu0'] == '/imu/measurements'
            assert [i for i, flag in enumerate(params['imu0_config']) if flag] == [9, 10, 11]
        else:
            assert 'imu0' not in params
    assert ekfs[0]['odom0'] == '/lidar_odometry'
    assert ekfs[0]['odom0_differential'] is False
    assert ekfs[1]['odom0'] == '/local_odometry/filter'
    assert ekfs[1]['odom0_differential'] is True
    assert ekfs[1]['pose0'] == '/ml_localization/ndt_pose'
    assert sum(n['executable'] == 'imu_measurements.py' for n in nodes) == int(enabled)


@pytest.mark.parametrize('delay', ['-1', 'nan', 'inf', 'invalid'])
def test_invalid_manager_delay(delay, monkeypatch, tmp_path):
    launch, options = prepare_launch(True, monkeypatch, tmp_path, delay)
    with pytest.raises(ValueError):
        launch.launch_nodes(SimpleNamespace(launch_configurations=options))


@pytest.mark.parametrize('delay', [0.0, 10.0])
def test_manager_wall_time_delay_without_ros_clock(delay, monkeypatch, tmp_path):
    import time
    from launch import LaunchDescription, LaunchService
    from launch.actions import OpaqueFunction

    launch, options = prepare_launch(True, monkeypatch, tmp_path, str(delay))
    started = {}

    def node_action(**kwargs):
        def record(context):
            started[kwargs['executable']] = time.monotonic()
            return []
        return OpaqueFunction(function=record)

    monkeypatch.setattr(launch, 'Node', node_action)
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'launch_logs'))
    service = LaunchService()
    service.include_launch_description(LaunchDescription(
        launch.launch_nodes(SimpleNamespace(launch_configurations=options))))
    assert service.run() == 0
    elapsed = started['ml_localization_manager'] - started['relocalization_node']
    assert elapsed >= delay - 0.05
    assert elapsed < delay + 2.0
    # Other nodes must not wait for the Manager delay.
    assert abs(started['fastlio_mapping'] - started['relocalization_node']) < 1.0
    arguments = launch.generate_launch_description().get_launch_arguments()
    argument = next(a for a in arguments if a.name == 'manager_start_delay')
    assert ''.join(part.text for part in argument.default_value) == '6.0'
