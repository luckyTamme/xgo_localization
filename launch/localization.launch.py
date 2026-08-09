"""Bring up localisation with a selectable SLAM backend.

    ros2 launch xgo_localization localization.launch.py backend:=cartographer
    ros2 launch xgo_localization localization.launch.py backend:=rtabmap

Everything lands under ``/localization`` except ``/diagnostics``, which stays at
the root because standard tooling subscribes to it absolutely.

``/tf`` and ``/tf_static`` also stay at the root, but that needs no help from
here: tf2's broadcasters and listeners use those absolute names directly, so the
namespace push never touches them.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    package_share = get_package_share_directory('xgo_localization')
    default_params = os.path.join(package_share, 'config', 'localization.yaml')
    cartographer_config_dir = os.path.join(package_share, 'config', 'cartographer')
    default_rtabmap_params = os.path.join(package_share, 'config', 'rtabmap.yaml')

    backend = LaunchConfiguration('backend')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    rtabmap_params_file = LaunchConfiguration('rtabmap_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')

    use_cartographer = IfCondition(PythonExpression(["'", backend, "' == 'cartographer'"]))
    use_rtabmap = IfCondition(PythonExpression(["'", backend, "' == 'rtabmap'"]))

    # Absolute names for anything that must survive the namespace push. These
    # assume `namespace` is bare (no leading slash), matching the argument's
    # documented form; a leading slash would produce '//localization/map'.
    map_topic = ['/', namespace, '/map']
    odom_topic = ['/', namespace, '/odom']

    return LaunchDescription([
        DeclareLaunchArgument(
            'backend', default_value='cartographer',
            choices=['cartographer', 'rtabmap'],
            description='Which SLAM backend corrects map -> odom.'),
        DeclareLaunchArgument(
            'namespace', default_value='localization',
            description='Namespace for every topic this stack owns. Give it '
                        'bare, without a leading slash.'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Parameters for the localisation node.'),
        DeclareLaunchArgument(
            'rtabmap_params_file', default_value=default_rtabmap_params,
            description='Parameters for rtabmap. Ignored for cartographer.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='True when replaying a bag with --clock. Must match the '
                        'clock the data was recorded against.'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='LaserScan input. Absolute, so it resolves at the root.'),

        LogInfo(msg=['[xgo_localization] backend=', backend,
                     '  namespace=/', namespace,
                     '  use_sim_time=', use_sim_time]),

        GroupAction([
            PushRosNamespace(namespace),

            Node(
                package='xgo_localization',
                executable='localization_node',
                name='localization_node',
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
            ),

            # -- cartographer ------------------------------------------------
            # published_frame=odom + provide_odom_frame=false means it emits
            # only map -> odom, matching rtabmap, so the two are interchangeable
            # and this node keeps sole ownership of odom -> base_link.
            Node(
                condition=use_cartographer,
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                namespace='cartographer',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    '-configuration_directory', cartographer_config_dir,
                    '-configuration_basename', 'xgo_2d.lua',
                ],
                remappings=[('scan', scan_topic), ('odom', odom_topic)],
            ),
            Node(
                condition=use_cartographer,
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                name='cartographer_occupancy_grid_node',
                namespace='cartographer',
                output='screen',
                # resolution and publish_period_sec are gflags in this node, not
                # ROS parameters. Passed as parameters they are accepted, never
                # read, and never warned about.
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
                remappings=[('map', map_topic)],
            ),

            # -- rtabmap -----------------------------------------------------
            # odom_frame_id makes rtabmap take odometry from TF, so no odom
            # topic subscription and no icp_odometry: this node already
            # publishes odom -> base_link.
            Node(
                condition=use_rtabmap,
                package='rtabmap_slam',
                executable='rtabmap',
                name='rtabmap',
                namespace='rtabmap',
                output='screen',
                arguments=['--delete_db_on_start'],
                parameters=[rtabmap_params_file, {'use_sim_time': use_sim_time}],
                remappings=[('scan', scan_topic), ('map', map_topic)],
            ),
        ]),
    ])
