"""Run the unchanged R2 scenario with an additional passive fusion node."""

from importlib.util import module_from_spec, spec_from_file_location
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration


def load_capture():
    """Resolve the optional Gazebo capture supplied by the caller."""
    directory = os.environ.get('R2_DEMO')
    if not directory:
        return None
    path = Path(directory) / 'r2_capture.launch.py'
    if not path.is_file():
        raise RuntimeError('R2_DEMO must contain r2_capture.launch.py')
    spec = spec_from_file_location('r2_capture_launch', path)
    capture = module_from_spec(spec)
    spec.loader.exec_module(capture)
    return capture


r2 = load_capture()


def fusion(context):
    """Add the state publisher and stop the run if it unexpectedly exits."""
    run = Path(LaunchConfiguration('run').perform(context))
    prefix = Path(get_package_prefix('my_dwa_controller'))
    executable = prefix / 'lib' / 'my_dwa_controller' / 'pedestrian_fusion'
    estimator = LaunchConfiguration('position_estimator').perform(context)
    state_estimator = LaunchConfiguration('state_estimator').perform(context)
    kf_config = LaunchConfiguration('kf_config').perform(context)
    params = ['-p', f'state_estimator:={state_estimator}']
    if state_estimator == 'kf':
        params.extend(['-p', f'kf_config:={kf_config}'])
    process = ExecuteProcess(
        cmd=[str(executable), '--ros-args',
             '-p', 'use_sim_time:=true', '-p',
             f'diagnostic_file:={run / "pedestrians.jsonl"}',
             '-p', f'position_estimator:={estimator}', *params],
        output='screen')
    handler = RegisterEventHandler(OnProcessExit(
        target_action=process, on_exit=r2.base.base.stop_if_unexpected))
    return [handler, process]


def generate_launch_description():
    """Reuse the R2 capture without altering the world or motion driver."""
    if r2 is None:
        raise RuntimeError('Set R2_DEMO for capture or use pedestrian_fusion.launch.py')
    description = r2.generate_launch_description()
    description.add_action(
        DeclareLaunchArgument('position_estimator', default_value='trimmed_centroid'))
    description.add_action(DeclareLaunchArgument('state_estimator', default_value='kf'))
    config = Path(get_package_share_directory('my_dwa_controller')) / 'config/kf_frozen.json'
    description.add_action(DeclareLaunchArgument('kf_config', default_value=str(config)))
    description.add_action(OpaqueFunction(function=fusion))
    return description
