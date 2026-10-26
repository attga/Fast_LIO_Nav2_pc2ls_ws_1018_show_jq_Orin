"""Offline startup regression tests; no ROS nodes or hardware are started."""

import copy
import os
from pathlib import Path
import runpy
import shlex
import shutil
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import yaml

from prepare_nav2 import prepare


SOURCE = Path(__file__).resolve().parents[1]


def bash_path():
    if os.name == 'nt':
        candidate = Path('C:/Program Files/Git/bin/bash.exe')
        if candidate.exists():
            return str(candidate)
    return shutil.which('bash')


def native_path(value):
    # The fake ROS command captures the Linux-style paths used by Git Bash.
    if os.name == 'nt' and len(value) > 3 and value[0] == '/' and value[2] == '/':
        value = value[1].upper() + ':' + value[2:]
    return Path(value)


class StartupTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='nav startup ', dir=SOURCE.parent)
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / 'src/tools').mkdir(parents=True)
        for name in ('start_nav2.sh', 'start_robot.sh', 'tools/prepare_nav2.py'):
            shutil.copyfile(SOURCE / name, self.root / 'src' / name)
        config = self.root / 'src/my_nav2_config/config'
        config.mkdir(parents=True)
        self.params = yaml.safe_load((SOURCE / 'my_nav2_config/config/nav2_params.yaml')
                                     .read_text(encoding='utf-8'))
        (config / 'nav2_params.yaml').write_text(yaml.safe_dump(self.params), encoding='utf-8')
        self.map_file = self.make_map('custom maps', 'floor.yaml', 'floor image.png')
        self.mask_file = self.make_map('custom maps', 'mask.yaml', 'restricted.png')
        self.capture = self.root / 'captured.bin'
        mock_bin = self.root / 'mock-bin'
        mock_bin.mkdir()
        commands = {
            'python3': f'#!/usr/bin/env bash\nexec {shlex.quote(Path(sys.executable).as_posix())} "$@"\n',
            'ros2': '#!/usr/bin/env bash\nprintf "%s\\0" "$@" > "$CAPTURE_FILE"\n',
        }
        for name, contents in commands.items():
            path = mock_bin / name
            path.write_text(contents, encoding='utf-8', newline='\n')
            path.chmod(0o755)
        self.setup = self.root / 'setup.bash'
        self.setup.write_text('export PATH="$WORKSPACE/mock-bin:$PATH"\n', encoding='utf-8')

    def make_map(self, directory, name, image):
        folder = self.root / directory
        folder.mkdir(parents=True, exist_ok=True)
        (folder / image).write_bytes(b'mock image; only readability is checked')
        path = folder / name
        path.write_text(yaml.safe_dump({'image': image}), encoding='utf-8')
        return path

    def run_script(self, *args, script='start_nav2.sh', **env_overrides):
        bash = bash_path()
        if not bash:
            self.skipTest('Bash is required for shell integration tests')
        env = dict(os.environ, WORKSPACE_SETUP=self.setup.as_posix(),
                   CAPTURE_FILE=self.capture.as_posix(), USE_KEEPOUT='auto',
                   USE_SIM_TIME='false')
        env.update(env_overrides)
        return subprocess.run([bash, str(self.root / 'src' / script), *args],
                              cwd=self.root, env=env, capture_output=True,
                              encoding='utf-8', errors='replace', timeout=20)

    def captured(self):
        args = self.capture.read_bytes().decode('utf-8').rstrip('\0').split('\0')
        configs = {}
        for arg in args:
            if ':=' in arg:
                name, value = arg.split(':=', 1)
                self.assertNotIn(name, configs, f'Duplicate launch argument: {name}')
                configs[name] = value
        return configs

    def test_custom_map_without_default_map_and_last_override_wins(self):
        result = self.run_script('map:=missing.yaml',
                                 f'map:={self.map_file.as_posix()}',
                                 'use_keepout:=true', 'use_keepout:=false',
                                 'autostart:=false')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        configs = self.captured()
        self.assertEqual(configs['use_keepout'], 'false')
        self.assertEqual(configs['autostart'], 'false')
        params = yaml.safe_load(native_path(configs['params_file']).read_text(encoding='utf-8'))
        for name in ('local_costmap', 'global_costmap'):
            self.assertNotIn('filters', params[name][name]['ros__parameters'])

    def test_cli_keepout_overrides_environment_and_uses_custom_params(self):
        params = copy.deepcopy(self.params)
        params['controller_server']['ros__parameters']['controller_frequency'] = 7.0
        for name in ('local_costmap', 'global_costmap'):
            config = params[name][name]['ros__parameters']
            config['keepout_filter']['enabled'] = False
        custom = self.root / 'custom params.yaml'
        custom.write_text(yaml.safe_dump(params), encoding='utf-8')
        result = self.run_script(f'map:={self.map_file.as_posix()}',
                                 f'mask:={self.mask_file.as_posix()}',
                                 f'params_file:={custom.as_posix()}',
                                 'use_keepout:=true', USE_KEEPOUT='false')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        configs = self.captured()
        self.assertEqual(configs['use_keepout'], 'true')
        effective = yaml.safe_load(native_path(configs['params_file']).read_text(encoding='utf-8'))
        self.assertEqual(effective['controller_server']['ros__parameters']['controller_frequency'], 7.0)
        for name in ('local_costmap', 'global_costmap'):
            config = effective[name][name]['ros__parameters']
            self.assertIn('keepout_filter', config['filters'])
            self.assertTrue(config['keepout_filter']['enabled'])
        self.assertEqual(yaml.safe_load(custom.read_text(encoding='utf-8')), params)

    def test_invalid_map_or_explicit_mask_fails_before_ros(self):
        for args in (
            ['map:=does-not-exist.yaml'],
            [f'map:={self.map_file.as_posix()}', 'use_keepout:=true', 'mask:=missing.yaml'],
        ):
            with self.subTest(args=args):
                result = self.run_script(*args)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(self.capture.exists())

    def test_auto_validates_selected_mask_image(self):
        self.assertEqual(prepare(self.map_file, self.mask_file, self.root /
                                'src/my_nav2_config/config/nav2_params.yaml',
                                'auto', self.root / 'effective.yaml'), 'true')
        (self.mask_file.parent / 'restricted.png').unlink()
        with self.assertRaises(OSError):
            prepare(self.map_file, self.mask_file, self.root /
                    'src/my_nav2_config/config/nav2_params.yaml',
                    'auto', self.root / 'effective.yaml')

    def test_auto_without_optional_mask_disables_keepout(self):
        result = self.run_script(f'map:={self.map_file.as_posix()}')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.captured()['use_keepout'], 'false')

    def test_enabling_keepout_adds_missing_filter(self):
        for name in ('local_costmap', 'global_costmap'):
            config = self.params[name][name]['ros__parameters']
            config.pop('filters')
            config.pop('keepout_filter')
        custom = self.root / 'custom.yaml'
        custom.write_text(yaml.safe_dump(self.params), encoding='utf-8')
        output = self.root / 'effective.yaml'
        prepare(self.map_file, self.mask_file, custom, 'true', output)
        effective = yaml.safe_load(output.read_text(encoding='utf-8'))
        for name in ('local_costmap', 'global_costmap'):
            config = effective[name][name]['ros__parameters']
            self.assertIn('keepout_filter', config['filters'])
            self.assertTrue(config['keepout_filter']['enabled'])

    def test_disabling_keepout_preserves_other_filters(self):
        for name in ('local_costmap', 'global_costmap'):
            config = self.params[name][name]['ros__parameters']
            config['filters'].append('speed_filter')
            config['speed_filter'] = {'plugin': 'nav2_costmap_2d::SpeedFilter'}
        custom = self.root / 'custom.yaml'
        custom.write_text(yaml.safe_dump(self.params), encoding='utf-8')
        output = self.root / 'effective.yaml'
        prepare(self.map_file, 'unused-mask.yaml', custom, 'false', output)
        effective = yaml.safe_load(output.read_text(encoding='utf-8'))
        for name in ('local_costmap', 'global_costmap'):
            self.assertEqual(effective[name][name]['ros__parameters']['filters'], ['speed_filter'])

    def test_robot_script_passes_environment_clock(self):
        result = self.run_script(script='start_robot.sh', USE_SIM_TIME='true')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.captured()['use_sim_time'], 'true')


class SourceLaunchTests(unittest.TestCase):
    def test_source_launches_do_not_load_installed_configuration(self):
        # Minimal launch records exercise path selection without ROS installed.
        class Record:
            def __init__(self, *args, **kwargs):
                self.args = args
                self.kwargs = kwargs

        class Description(list):
            pass

        def installed_path(package):
            raise AssertionError(f'Unexpected installed package lookup: {package}')

        definitions = {
            'ament_index_python.packages': {'get_package_share_directory': installed_path},
            'launch': {'LaunchDescription': Description},
            'launch.actions': {'DeclareLaunchArgument': Record, 'IncludeLaunchDescription': Record},
            'launch.conditions': {'IfCondition': Record},
            'launch.launch_description_sources': {'PythonLaunchDescriptionSource': Record},
            'launch.substitutions': {'LaunchConfiguration': Record, 'PythonExpression': Record,
                                     'Command': Record},
            'launch_ros.actions': {'Node': Record},
        }
        modules = {}
        for name, attributes in definitions.items():
            module = types.ModuleType(name)
            module.__dict__.update(attributes)
            modules[name] = module
        with patch.dict(sys.modules, modules):
            robot = runpy.run_path(str(SOURCE / 'LIO-SAM_MID360_ROS2/launch/robot.launch.py'))
            description = robot['generate_launch_description']()
            includes = [r for r in description if r.args and isinstance(r.args[0], Record)]
            expected = ['chassis_driver/launch/chassis_driver.launch.py',
                        'livox_ros_driver2/launch_ROS2/msg_MID360s_launch.py',
                        'yesense_std_ros2/launch/yesense_node.launch.py',
                        'LIO-SAM_MID360_ROS2/launch/relocalization.launch.py']
            self.assertEqual([Path(r.args[0].args[0]).resolve() for r in includes],
                             [(SOURCE / p).resolve() for p in expected])
            defaults = {r.args[0]: r.kwargs['default_value'] for r in description
                        if r.args and isinstance(r.args[0], str)}
            self.assertEqual(Path(defaults['map_data_dir']), SOURCE / 'LIO-SAM_MID360_ROS2/data')
            self.assertEqual(Path(defaults['params_file']),
                             SOURCE / 'LIO-SAM_MID360_ROS2/config/params.yaml')
            for relative in ('LIO-SAM_MID360_ROS2/launch/relocalization.launch.py',
                             'yesense_std_ros2/launch/yesense_node.launch.py'):
                launch = runpy.run_path(str(SOURCE / relative))
                self.assertTrue(launch['generate_launch_description']())


if __name__ == '__main__':
    unittest.main()
