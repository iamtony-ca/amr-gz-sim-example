"""
run_simulation_static_filter.launch.py

Gz sim counterpart of amr_bringup/launch/run_amr_bringup.launch.py — the
top-level wrapper that overlays ammr_common_config/common_ammr.yaml's
common_settings (self_machine_id, self_type_id, robot_ids, per-bot entries)
onto nav2_params_gz_static_filter.yaml, writes the merged result to a temp
file, and chains into simulation_static_filter.launch.py.

This is the canonical entry point for the sim — it mirrors how the real
robot is brought up on the real platform (via run_amr_bringup.launch.py).

Usage:
  ros2 launch mobile_robot_gz_sim run_simulation_static_filter.launch.py
  ros2 launch mobile_robot_gz_sim run_simulation_static_filter.launch.py \
      map:=/path/to/my_map.yaml \
      filter_mask_file:=/path/to/my_keepout.yaml \
      headless:=False use_rviz:=True
"""

import copy
import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def replace_keys_recursively(data, rewrites):
    """Walk a YAML tree and replace any key present in `rewrites` with the
    rewrite value (deep-copied). Matches the real-robot run_amr_bringup
    behavior — overwriting wherever the key appears, at any depth.
    """
    if isinstance(data, dict):
        for k, v in data.items():
            if k in rewrites and rewrites[k] is not None:
                data[k] = copy.deepcopy(rewrites[k])
            elif isinstance(v, dict):
                replace_keys_recursively(v, rewrites)


def generate_launch_description():
    pkg_gz_sim = get_package_share_directory('mobile_robot_gz_sim')
    fleet_config_dir = get_package_share_directory('ammr_common_config')

    # 1. Read common_ammr.yaml
    common_yaml_path = os.path.join(fleet_config_dir, 'config', 'common_ammr.yaml')
    with open(common_yaml_path, 'r') as f:
        common_data = yaml.safe_load(f)['common_settings']

    # 2. Collect keys to overwrite (matches run_amr_bringup.launch.py).
    fleet_rewrites = {
        'self_machine_id': common_data['my_machine_id'],
        'self_type_id': common_data['my_type_id'],
        'robot_ids': common_data['robot_ids'],
    }
    for bot_id in common_data.get('robot_ids', []):
        if bot_id in common_data:
            fleet_rewrites[bot_id] = common_data[bot_id]

    # 3. Read the gz-sim filter params file (default overlay target).
    original_yaml_path = os.path.join(
        pkg_gz_sim, 'params', 'nav2_params_gz_static_filter.yaml')
    with open(original_yaml_path, 'r') as f:
        yaml_data = yaml.safe_load(f)

    if yaml_data is None:
        raise ValueError(f"{original_yaml_path} is empty or invalid")

    # 4. In-memory merge.
    replace_keys_recursively(yaml_data, fleet_rewrites)

    # 5. Write the merged result to a temp file (default_flow_style=False so
    #    the ROS 2 YAML parser reads it as block style — matches the safety
    #    note in run_amr_bringup.launch.py).
    temp_yaml_file = tempfile.NamedTemporaryFile(
        mode='w', delete=False, suffix='.yaml')
    yaml.dump(yaml_data, temp_yaml_file, default_flow_style=False)
    temp_yaml_file.close()

    # 6. Forward user-tweakable args to simulation_static_filter.launch.py.
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_gz_sim, 'maps', 'depot.yaml'),
        description='Full path to map YAML')

    declare_filter_mask_file = DeclareLaunchArgument(
        'filter_mask_file',
        default_value=os.path.join(pkg_gz_sim, 'maps', 'depot.yaml'),
        description='Full path to costmap filter mask YAML')

    declare_use_filters = DeclareLaunchArgument(
        'use_filters', default_value='True',
        description='Whether to launch the costmap filter stack')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Launch RViz2')

    declare_headless = DeclareLaunchArgument(
        'headless', default_value='True',
        description='Run gz sim without GUI')

    run_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_sim, 'launch', 'simulation_static_filter.launch.py')
        ),
        launch_arguments=[
            ('params_file', temp_yaml_file.name),
            ('map', LaunchConfiguration('map')),
            ('filter_mask_file', LaunchConfiguration('filter_mask_file')),
            ('use_filters', LaunchConfiguration('use_filters')),
            ('use_rviz', LaunchConfiguration('use_rviz')),
            ('headless', LaunchConfiguration('headless')),
        ],
    )

    return LaunchDescription([
        declare_map,
        declare_filter_mask_file,
        declare_use_filters,
        declare_use_rviz,
        declare_headless,
        run_cmd,
    ])
