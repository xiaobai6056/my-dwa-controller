"""Start a standard MyDWA static Gazebo world with the installed Nav2 configuration."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
import yaml


def start_scenario(context, package, bringup, scenes):
    """Resolve the selected standard world, map and initial robot pose."""
    name = LaunchConfiguration('scene').perform(context)
    scene = scenes[name]
    x, y, yaw = map(str, scene['start'])
    if scene['world'] == 'stock':
        world, map_yaml = (bringup / 'worlds/world_only.model',
                           bringup / 'maps/turtlebot3_world.yaml')
    else:
        assets = package / 'scenarios' / 'static' / 'assets' / name
        world, map_yaml = assets / 'world.sdf', assets / 'map.yaml'
    models = [str(Path(get_package_share_directory('turtlebot3_gazebo')) / 'models'),
              '/usr/share/gazebo-11/models']
    if os.environ.get('GAZEBO_MODEL_PATH'):
        models.append(os.environ['GAZEBO_MODEL_PATH'])
    # Give Humble's AMCL the declared start pose during lifecycle activation.
    return [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(models)),
        SetParameter(name='set_initial_pose', value=True),
        SetParameter(name='initial_pose.x', value=float(x)),
        SetParameter(name='initial_pose.y', value=float(y)),
        SetParameter(name='initial_pose.yaw', value=float(yaw)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/tb3_simulation_launch.py')),
            launch_arguments={
                'world': str(world), 'map': str(map_yaml),
                'params_file': str(package / 'config/nav2_params.yaml'),
                'x_pose': x, 'y_pose': y, 'yaw': yaw,
                'use_sim_time': 'true', 'use_composition': 'False',
                'headless': LaunchConfiguration('headless'),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items()),
    ]


def generate_launch_description():
    """Expose scene selection without checkout or home-directory assumptions."""
    package = Path(get_package_share_directory('my_dwa_controller'))
    bringup = Path(get_package_share_directory('nav2_bringup'))
    scenes = yaml.safe_load(
        (package / 'scenarios/static/scenarios.yaml').read_text(encoding='utf-8'))['scenarios']
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='S01', choices=list(scenes)),
        DeclareLaunchArgument('headless', default_value='True'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        OpaqueFunction(function=start_scenario, args=[package, bringup, scenes]),
    ])
