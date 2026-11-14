"""Launch an interactive MyDWA demonstration from installed scene assets."""

import json
import os
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
import yaml

SCENES = ('M1', 'same-direction', 'PERSON', 'S02')


def static_demo(share, run):
    """Embed the stock Waffle and bring up all of Nav2 on the S02 map."""
    bringup = Path(get_package_share_directory('nav2_bringup'))
    assets = share / 'scenarios/static/assets/S02'
    scenes = yaml.safe_load((assets.parent.parent / 'scenarios.yaml').read_text())
    x, y, yaw = scenes['scenarios']['S02']['start']
    model = ET.parse(bringup / 'worlds/waffle.model').getroot().find('model')
    model.find('pose').text = f'{x} {y} 0.01 0 0 {yaw}'
    world = ET.parse(assets / 'world.sdf')
    world.getroot().find('world').append(model)
    world_file = run / 's02.world'
    world.write(world_file, encoding='utf-8', xml_declaration=True)
    wait_for_clock = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '/clock', '--once', '--qos-reliability', 'best_effort'],
        name='demo_wait_for_gazebo')
    gui = ExecuteProcess(cmd=['gzclient'], name='demo_gzclient', output='screen')
    models = [str(Path(get_package_share_directory('turtlebot3_gazebo')) / 'models'),
              '/usr/share/gazebo-11/models']
    if os.environ.get('GAZEBO_MODEL_PATH'):
        models.append(os.environ['GAZEBO_MODEL_PATH'])
    urdf = (bringup / 'urdf/turtlebot3_waffle.urdf').read_text()
    return [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(models)),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetParameter(name='FollowPath.diagnostics.visualization', value=True),
        SetParameter(name='set_initial_pose', value=True),
        SetParameter(name='initial_pose.x', value=float(x)),
        SetParameter(name='initial_pose.y', value=float(y)),
        SetParameter(name='initial_pose.yaw', value=float(yaw)),
        RegisterEventHandler(OnProcessExit(
            target_action=wait_for_clock,
            on_exit=lambda event, context: [gui] if event.returncode == 0
            and not context.is_shutdown else [])),
        ExecuteProcess(cmd=['gzserver', '-s', 'libgazebo_ros_init.so', str(world_file)],
                       name='demo_gzserver', output='screen'),
        wait_for_clock,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True, 'robot_description': urdf}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/bringup_launch.py')),
            launch_arguments={'map': str(assets / 'map.yaml'),
                              'params_file': str(share / 'config/nav2_params.yaml'),
                              'use_sim_time': 'true', 'use_composition': 'False'}.items()),
    ]


def dynamic_demo(share, name, run):
    """Reproduce the isolated FollowPath stack with visible Gazebo and live fusion."""
    scene = share / 'scenarios/dynamic' / name
    for name_in_scene in ('v01.world', 'config.json', 'trial.json'):
        shutil.copyfile(scene / name_in_scene, run / name_in_scene)
    trial = json.loads((run / 'trial.json').read_text())
    x, y, yaw = trial.get('transform', (trial['start'], 0., 0.))
    (run / 'empty.pgm').write_bytes(b'P5\n200 120\n255\n' + bytes([254]) * 24000)
    map_file = run / 'empty.yaml'
    map_file.write_text(
        'image: empty.pgm\nresolution: 0.05\norigin: [-3.0, -3.0, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    assets = Path(os.environ['MYDWA_VISUAL_ASSETS'])
    visual = share / 'visual_prototype'
    sort = Path(os.environ['SORT_DEMO'])
    state = Path(os.environ['STATE_DEMO'])
    sort_python = os.environ['SORT_PYTHON']
    sim_time = {'use_sim_time': True}
    sim_args = ['--ros-args', '-p', 'use_sim_time:=true']
    motion = (sort / 'multi_motion.py' if name == 'M1' else visual / 'v01_scenario.py')
    urdf = (assets / '.cache/waffle.urdf').read_text()
    models = [str(assets / 'models'),
              str(Path(get_package_share_directory('turtlebot3_gazebo')) / 'models'),
              '/usr/share/gazebo-11/models']
    plugins = [str(assets / '.cache/v01-gazebo-build')]
    for paths, environment in ((models, 'GAZEBO_MODEL_PATH'), (plugins, 'GAZEBO_PLUGIN_PATH')):
        if os.environ.get(environment):
            paths.append(os.environ[environment])
    return [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(models)),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', os.pathsep.join(plugins)),
        ExecuteProcess(cmd=['gzserver', '-s', 'libgazebo_ros_init.so', str(run / 'v01.world')],
                       name='demo_gzserver', output='screen'),
        ExecuteProcess(cmd=['gzclient'], name='demo_gzclient', output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim_time, {'robot_description': urdf}]),
        ExecuteProcess(cmd=[str(assets / '.venv/bin/python'), '-u',
                            str(visual / 'person_detector.py'), *sim_args],
                       name='demo_yolo', output='screen'),
        ExecuteProcess(cmd=[sort_python, '-u', str(sort / 'sort_node.py'), *sim_args],
                       name='demo_sort', output='screen'),
        ExecuteProcess(cmd=[sort_python, '-u', str(state / 'vision_state.py'), *sim_args],
                       name='demo_ipm', output='screen'),
        Node(package='my_dwa_controller', executable='pedestrian_fusion',
             parameters=[sim_time, {'visualize_clusters': True}], output='screen'),
        ExecuteProcess(cmd=[sys.executable, '-u', str(motion), str(run), *sim_args,
                            '-r', '/cmd_vel:=/my_dwa/demo/motion_cmd'],
                       name='demo_actor_motion', output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'world', '--child-frame-id', 'odom',
                        '--x', str(x), '--y', str(y), '--yaw', str(yaw)]),
        Node(package='nav2_map_server', executable='map_server',
             parameters=[sim_time, {'yaml_filename': str(map_file), 'frame_id': 'odom'}]),
        Node(package='nav2_controller', executable='controller_server',
             parameters=[str(share / 'scenarios/dynamic/nav2_params.yaml'),
                         {'FollowPath.diagnostics.visualization': True,
                          'FollowPath.diagnostics.control_timing': False}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_demo',
             parameters=[sim_time, {'autostart': True,
                                    'node_names': ['map_server', 'controller_server']}]),
    ]


def start_demo(context, share):
    """Start the simulator, demo goal and RViz without an end-of-run timer."""
    scene = LaunchConfiguration('scene').perform(context)
    run = Path(LaunchConfiguration('run').perform(context))
    run.mkdir(parents=True, exist_ok=True)
    actions = static_demo(share, run) if scene == 'S02' else dynamic_demo(share, scene, run)
    frame = 'map' if scene == 'S02' else 'odom'
    target = share / 'scenarios/static/scenarios.yaml' if scene == 'S02' else run / 'trial.json'
    goal_args = [scene, str(target)]
    if LaunchConfiguration('wait_for_input').perform(context) == 'true':
        goal_args.extend(['--start-file', str(run / 'start')])
    actions.extend([
        Node(package='my_dwa_controller', executable='demo_goal',
             arguments=goal_args, parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'config/demo_ui.rviz'), '-f', frame],
             parameters=[{'use_sim_time': True}], output='screen'),
    ])
    return actions


def generate_launch_description():
    """Declare the scenario and scratch directory used by the shell supervisor."""
    share = Path(get_package_share_directory('my_dwa_controller'))
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='M1', choices=SCENES),
        DeclareLaunchArgument('run'),
        DeclareLaunchArgument('wait_for_input', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=start_demo, args=[share]),
    ])
