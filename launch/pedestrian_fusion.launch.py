"""Start the installed fusion node with existing ROS sensor and SORT topics."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Resolve the deployed KF configuration from the package share."""
    share = Path(get_package_share_directory('my_dwa_controller'))
    defaults = {
        'use_sim_time': 'false',
        'position_estimator': 'trimmed_centroid',
        'state_estimator': 'kf',
        'kf_config': str(share / 'config' / 'kf_frozen.json'),
        'diagnostic_file': '',
    }
    params = {
        name: ParameterValue(LaunchConfiguration(name),
                             value_type=bool if name == 'use_sim_time' else str)
        for name in defaults
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        Node(package='my_dwa_controller', executable='pedestrian_fusion',
             parameters=[params], output='screen'),
    ])
