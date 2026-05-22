"""
simulation_static_filter.launch.py

Full navigation simulation in gz sim — TWO-planner + COSTMAP-FILTER variant.
Mirrors simulation_static.launch.py but additionally launches the real-robot
costmap filter stack (filter_mask_server + costmap_filter_info_server +
lifecycle_manager_costmap_filters via amr_bringup/launch/costmap_filter.launch.py).

For the no-filter version use simulation_static.launch.py.
For the param-overrides wrapper (common_ammr.yaml rewrite) use
run_simulation_static_filter.launch.py.

Starts:
  1. gz sim world (depot world)
  2. robot_state_publisher + ros_gz_bridge + robot spawn
  3. Localization: nav2_bringup localization_launch (map_server + AMCL)
  4. Costmap filters: filter_mask_server + costmap_filter_info_server (+ lifecycle mgr)
  5. Navigation: navigation_launch_gz_static.py (nav stack + static_planner_server)
  6. RViz2 (optional)

Usage:
  ros2 launch mobile_robot_gz_sim simulation_static_filter.launch.py
  ros2 launch mobile_robot_gz_sim simulation_static_filter.launch.py \
      filter_mask_file:=/path/to/depot_keepout.yaml
  ros2 launch mobile_robot_gz_sim simulation_static_filter.launch.py use_filters:=False
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    pkg_gz_sim = get_package_share_directory('mobile_robot_gz_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_amr_bringup = get_package_share_directory('amr_bringup')

    # ---------- Launch arguments ----------
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    use_filters = LaunchConfiguration('use_filters')
    filter_mask_file = LaunchConfiguration('filter_mask_file')

    # Spawn at (5.86, 0.17): an open, well-clear spot on the depot warehouse
    # floor. Must be free in the depot map; AMCL initial_pose must match.
    pose = {
        'x': LaunchConfiguration('x_pose', default='5.86'),
        'y': LaunchConfiguration('y_pose', default='0.17'),
        'z': LaunchConfiguration('z_pose', default='0.12'),
        'yaw': LaunchConfiguration('yaw', default='0.0'),
    }

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_gz_sim, 'maps', 'depot.yaml'),
        description='Full path to map YAML')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_gz_sim, 'params', 'nav2_params_gz_static_filter.yaml'),
        description='Full path to Nav2 parameters file (filter variant)')

    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically start Nav2 lifecycle nodes')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Launch RViz2')

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to RViz config file')

    declare_headless = DeclareLaunchArgument(
        'headless', default_value='True',
        description='Run gz sim without GUI. Set False to show gz sim GUI.')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_gz_sim, 'worlds', 'depot.sdf'),
        description='Full path to gz sim world SDF (xacro)')

    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='mobile_robot',
        description='Name for the robot model in gz sim')

    declare_use_filters = DeclareLaunchArgument(
        'use_filters', default_value='True',
        description='Whether to launch the costmap filter stack')

    # No depot keepout mask ships with the package yet — default to the depot
    # map itself so the launch never fails on a missing file. Override with a
    # real keepout yaml for actual filtering.
    declare_filter_mask_file = DeclareLaunchArgument(
        'filter_mask_file',
        default_value=os.path.join(pkg_gz_sim, 'maps', 'depot.yaml'),
        description='Full path to costmap filter mask YAML')

    # ---------- GZ_SIM_RESOURCE_PATH ----------
    set_gz_resource_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_gz_sim, 'models'))

    # ---------- gz sim world ----------
    world_sdf = tempfile.mktemp(prefix='mobile_robot_nav_', suffix='.sdf')

    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world])

    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_sdf],
        output='screen',
    )

    gz_server_after_xacro = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=world_sdf_xacro,
            on_exit=[gz_server],
        )
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        condition=IfCondition(PythonExpression(['not ', headless])),
        launch_arguments={'gz_args': '-v4 -g'}.items(),
    )

    remove_temp_sdf = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf))]
        )
    )

    # ---------- Robot spawn ----------
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_sim, 'launch', 'spawn_robot.launch.py')),
        launch_arguments={
            'robot_name': robot_name,
            'x_pose': pose['x'],
            'y_pose': pose['y'],
            'z_pose': pose['z'],
            'yaw': pose['yaw'],
        }.items(),
    )

    # ---------- Scan merger (f_scan + b_scan -> /scan) ----------
    scan_merger = Node(
        package='three_scan_merger_ros2',
        executable='three_scan_merger_node',
        name='three_scan_merger_node',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('three_scan_merger_ros2'),
                'params', 'scan_merger_ros2_real.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # ---------- Nav2 bringup ----------
    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
            'use_respawn': 'False',
            'namespace': '',
        }.items(),
    )

    # Costmap filter servers (filter_mask_server + costmap_filter_info_server +
    # lifecycle_manager_costmap_filters). Reuses amr_bringup/costmap_filter.launch.py;
    # use_composition=False -> standalone nodes (matches the rest of the gz sim stack).
    nav2_costmap_filters = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_bringup, 'launch', 'costmap_filter.launch.py')),
        condition=IfCondition(use_filters),
        launch_arguments={
            'namespace': '',
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'mask': filter_mask_file,
            'use_composition': 'False',
            'container_name': 'nav2_container',
        }.items(),
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_sim, 'launch', 'navigation_launch_gz_static.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
            'use_respawn': 'False',
            'namespace': '',
            'log_level': 'info',
        }.items(),
    )

    # Localization + filter servers at t+10s, navigation 6s later (t+16s):
    # AMCL must publish map->odom and the filter_info_topic must be up before
    # the global_costmap's lane_filter layer activates.
    nav2_bringup_loc = TimerAction(
        period=10.0,
        actions=[nav2_localization, nav2_costmap_filters],
    )
    nav2_bringup_nav = TimerAction(
        period=16.0,
        actions=[nav2_navigation],
    )

    # ---------- RViz2 ----------
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config_file,
            'namespace': '',
            'use_namespace': 'false',
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params_file,
        declare_autostart,
        declare_use_rviz,
        declare_rviz_config,
        declare_headless,
        declare_world,
        declare_robot_name,
        declare_use_filters,
        declare_filter_mask_file,
        set_gz_resource_models,
        world_sdf_xacro,
        remove_temp_sdf,
        gz_server_after_xacro,
        gz_client,
        spawn_robot,
        scan_merger,
        nav2_bringup_loc,
        nav2_bringup_nav,
        rviz,
    ])
