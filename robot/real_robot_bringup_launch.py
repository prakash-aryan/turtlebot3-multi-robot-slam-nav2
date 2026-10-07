import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def _patched_tb3_params(robot, model):
    src = os.path.join(
        get_package_share_directory('turtlebot3_node'), 'param', f'{model}.yaml')
    with open(src) as f:
        cfg = yaml.safe_load(f)

    odom = cfg.setdefault('diff_drive_controller', {}).setdefault('ros__parameters', {}) \
              .setdefault('odometry', {})
    odom['frame_id'] = f'{robot}/odom'
    odom['child_frame_id'] = f'{robot}/base_footprint'

    cfg.setdefault('turtlebot3_node', {}).setdefault('ros__parameters', {})['enable_stamped_cmd_vel'] = True

    # Bare upstream keys match nothing once the nodes live under /<robot>.
    cfg = {f'/{robot}/{node}': params for node, params in cfg.items()}

    dst = f'/tmp/tb3_{robot}_{model}.yaml'
    with open(dst, 'w') as f:
        yaml.safe_dump(cfg, f, default_flow_style=False, sort_keys=False)
    return dst


def generate_launch_description():
    robot = os.environ['ROBOT_NAME']
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    usb_port = os.environ.get('OPENCR_PORT', '/dev/ttyACM0')

    bringup_dir = get_package_share_directory('turtlebot3_bringup')
    lidar_dir = get_package_share_directory('ld08_driver')
    tb3_param = _patched_tb3_params(robot, model)

    return LaunchDescription([
        GroupAction([
            PushRosNamespace(robot),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch',
                                 'turtlebot3_state_publisher.launch.py')),
                launch_arguments={'use_sim_time': 'false',
                                  'namespace': robot}.items(),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(lidar_dir, 'launch', 'ld08.launch.py')),
                # ld08_driver prepends its namespace itself, so the frame stays bare.
                launch_arguments={'frame_id': 'base_scan',
                                  'namespace': robot}.items(),
            ),

            Node(
                package='turtlebot3_node',
                executable='turtlebot3_ros',
                parameters=[tb3_param, {'namespace': robot}],
                arguments=['-i', usb_port],
                output='screen',
            ),
        ]),
    ])
