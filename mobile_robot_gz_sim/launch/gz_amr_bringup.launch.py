"""
gz_amr_bringup.launch.py

Single-file gz sim entry point that mirrors the real-robot launch chain
`run_amr_bringup.launch.py` → `bringup_launch.py` while reusing amr_bringup
and nav2_bringup launches AS-IS — no logic duplication.

Sim-specific responsibilities (everything outside the amr_bringup chain):
  1. LD_PRELOAD env (workspace amr_controller vs system nav2_controller
     libcontroller_server_core.so / libposition_goal_checker.so clash)
  2. gz sim world spawn + robot URDF spawn + ros_gz_bridge
     (three_scan_merger_ros2 is launched separately by
     mobile_robot_gz_sim/scripts/mobile_robot_sim.sh — mirrors the real
     robot's mobile_robot_localization.sh, which also runs scan_merger
     as its own process. It must come up before AMCL activates at t+10s.)
  3. AMCL+map_server via nav2_bringup/localization_launch.py (sim's
     Cartographer replacement)
  4. Staged timing (localization t+10s, run_amr_bringup t+16s)

Delegated to amr_bringup AS-IS via `run_amr_bringup.launch.py`:
  - common_ammr.yaml overlay logic (uses Option C `params_file_source` arg
    to read the sim params YAML instead of the real-robot one)
  - costmap_filter.launch.py + navigation_launch.py (via bringup_launch.py)
  - amr_bringup/localization_launch.py is SKIPPED via Option D
    `use_localization=False` since sim provides AMCL externally

Real-robot ↔ sim divergence is now limited to:
  (a) the sim-orchestration block above
  (b) localization source (Cartographer real / AMCL sim — natural split)
  (c) LD_PRELOAD env var

Existing duplicated sim launches (simulation*.launch.py,
navigation_launch_gz*.py, run_simulation_static_filter.launch.py) are
left untouched as a fallback path.

Usage:
  ros2 launch mobile_robot_gz_sim gz_amr_bringup.launch.py
  ros2 launch mobile_robot_gz_sim gz_amr_bringup.launch.py \
      headless:=False use_rviz:=True
  ros2 launch mobile_robot_gz_sim gz_amr_bringup.launch.py \
      map:=/path/to/my_map.yaml \
      filter_mask_file:=/path/to/my_keepout.yaml
  ros2 launch mobile_robot_gz_sim gz_amr_bringup.launch.py use_filters:=False
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

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_gz_sim = get_package_share_directory('mobile_robot_gz_sim')
    pkg_amr_bringup = get_package_share_directory('amr_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # NOTE: no LD_PRELOAD needed. The workspace nav2 binaries
    # (controller_server, planner_server, etc.) have DT_RUNPATH set by
    # colcon pointing at the correct workspace lib directories (e.g.
    # /root/work_ws/install/nav2_controller/lib, .../nav2_costmap_2d/lib).
    # RUNPATH takes precedence over LD_LIBRARY_PATH so amr_controller's
    # same-soname libs (which use a different namespace) don't get loaded
    # accidentally. Verify with:
    #   ldd /root/work_ws/install/nav2_controller/lib/nav2_controller/controller_server

    # =====================================================================
    # 2. Launch arguments
    # =====================================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    use_filters = LaunchConfiguration('use_filters')
    filter_mask_file = LaunchConfiguration('filter_mask_file')
    # Source params YAML — overlaid with common_ammr.yaml by
    # run_amr_bringup.launch.py. Default = sim's filter variant; override
    # with another sim YAML (e.g. nav2_params_gz_static_filter2.yaml).
    params_file_source = LaunchConfiguration('params_file_source')

    # Spawn pose; must match amcl.initial_pose in the params YAML so AMCL
    # publishes map->odom immediately (no manual RViz 2D Pose Estimate).
    pose = {
        'x': LaunchConfiguration('x_pose', default='5.86'),
        'y': LaunchConfiguration('y_pose', default='0.17'),
        'z': LaunchConfiguration('z_pose', default='0.12'),
        'yaw': LaunchConfiguration('yaw', default='0.0'),
    }

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')

    declare_params_file_source = DeclareLaunchArgument(
        'params_file_source',
        default_value=os.path.join(
            pkg_gz_sim, 'params', 'nav2_params_gz_static_filter.yaml'),
        description='Absolute path to source params YAML. '
                    'run_amr_bringup.launch.py overlays common_ammr.yaml '
                    'onto this file; the same file is also used by the sim '
                    "AMCL include. Override to swap to a different sim "
                    'params variant (e.g. nav2_params_gz_static_filter2.yaml).')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_gz_sim, 'maps', 'depot.yaml'),
        description='Absolute path to map YAML')

    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically start Nav2 lifecycle nodes')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Launch RViz2')

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz'),
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

    declare_filter_mask_file = DeclareLaunchArgument(
        'filter_mask_file',
        default_value=os.path.join(pkg_gz_sim, 'maps', 'depot.yaml'),
        description='Absolute path to costmap filter mask YAML '
                    '(placeholder default = depot map itself; override '
                    'with a real keepout mask for actual filtering)')

    # =====================================================================
    # 3. gz sim world + robot + bridge + scan merger (sim-specific)
    # =====================================================================
    set_gz_resource_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_gz_sim, 'models'))

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
            on_shutdown=[OpaqueFunction(
                function=lambda _: os.remove(world_sdf))]
        )
    )

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

    # NOTE: three_scan_merger_ros2 (two-corner-lidar → single /scan fusion)
    # is intentionally NOT launched here. It is started as a separate launch
    # in mobile_robot_gz_sim/scripts/mobile_robot_sim.sh — same pattern as
    # the real robot's mobile_robot_localization.sh. It must come up before
    # AMCL activates at t+10s (see staged timing below).

    # =====================================================================
    # 4. Sim AMCL — nav2_bringup's localization_launch.py AS-IS
    #    (real robot uses Cartographer in this slot, sim swaps to AMCL —
    #    natural real/sim divergence)
    #
    # NOTE on params: we pre-rewrite the sim params YAML so that
    # `map_server.yaml_filename` becomes the actual map path before handing
    # it to nav2_bringup/localization_launch.py. Reason: workspace's
    # nav2_bringup fork applies its `{'yaml_filename': map_yaml_file}`
    # override to map_server as a wildcard /**: parameter, which loses to
    # any specific node entry (e.g. `map_server: { yaml_filename: "" }`)
    # present in the source YAML. Pre-rewriting puts the resolved path
    # under the specific node entry so it wins.
    # =====================================================================
    localization_params = RewrittenYaml(
        source_file=params_file_source,
        root_key='',
        param_rewrites={'yaml_filename': map_yaml_file},
        convert_types=True,
    )

    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': localization_params,
            'autostart': autostart,
            'use_composition': 'False',
            'use_respawn': 'False',
            'namespace': '',
        }.items(),
    )

    # =====================================================================
    # 5. Real-robot nav stack — amr_bringup/run_amr_bringup.launch.py AS-IS
    #    (does common_ammr.yaml overlay onto sim_params_file then chains
    #    into bringup_launch.py which runs costmap_filter + navigation;
    #    amr_bringup's own localization is suppressed via use_localization=False
    #    because we provide AMCL externally above)
    # =====================================================================
    amr_bringup_chain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_bringup, 'launch',
                         'run_amr_bringup.launch.py')),
        launch_arguments={
            'params_file_source': params_file_source,
            'use_localization': 'False',
            'use_composition': 'False',
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': map_yaml_file,
            'use_filters': use_filters,
            'filter_mask_file': filter_mask_file,
        }.items(),
    )

    # Sim AMCL at t+10s, amr_bringup nav stack at t+16s.
    # AMCL must publish map->odom before navigation's global_costmap
    # activates, otherwise the costmap times out on the 'map' frame.
    nav2_bringup_loc = TimerAction(period=10.0, actions=[nav2_localization])
    nav2_bringup_nav = TimerAction(period=16.0, actions=[amr_bringup_chain])

    # =====================================================================
    # 6. RViz
    # =====================================================================
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
        # Launch arguments
        declare_use_sim_time,
        declare_params_file_source,
        declare_map,
        declare_autostart,
        declare_use_rviz,
        declare_rviz_config,
        declare_headless,
        declare_world,
        declare_robot_name,
        declare_use_filters,
        declare_filter_mask_file,
        # sim-specific
        set_gz_resource_models,
        world_sdf_xacro,
        remove_temp_sdf,
        gz_server_after_xacro,
        gz_client,
        spawn_robot,
        # delegated to amr_bringup / nav2_bringup
        nav2_bringup_loc,
        nav2_bringup_nav,
        rviz,
    ])
