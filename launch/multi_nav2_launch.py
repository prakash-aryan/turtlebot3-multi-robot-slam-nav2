import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushROSNamespace, SetParameter


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('namespace'),
        DeclareLaunchArgument('params_file'),
        GroupAction([
            PushROSNamespace(namespace),
            SetParameter('bond_timeout', 30.0),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={'namespace': namespace,
                                  'params_file': params_file,
                                  'use_sim_time': 'false',
                                  'autostart': 'true'}.items(),
            ),
        ]),
    ])
