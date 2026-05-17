"""
spawn_robot.launch.py

Spawns the mobile robot in a running gz sim world.
Starts:
  - robot_state_publisher  (publishes TF from URDF)
  - ros_gz_bridge          (bridges gz ↔ ROS topics)
  - ros_gz_sim create      (spawns the URDF model into gz sim)
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_mobile_robot_gz_sim = get_package_share_directory('mobile_robot_gz_sim')
    pkg_mobile_robot_desc = get_package_share_directory('mobile_robot_description')

    robot_urdf = os.path.join(
        pkg_mobile_robot_gz_sim, 'urdf', 'mobile_robot_gz.urdf.xacro')

    bridge_config = os.path.join(
        pkg_mobile_robot_gz_sim, 'configs', 'mobile_robot_bridge.yaml')

    # ---------- Launch arguments ----------
    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_z = DeclareLaunchArgument('z_pose', default_value='0.12',
                                      description='Spawn height — 0.12 m so wheels clear ground')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='mobile_robot')

    # ---------- Robot description (xacro → URDF string) ----------
    robot_description = Command([FindExecutable(name='xacro'), ' ', robot_urdf])

    # ---------- Robot state publisher ----------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': True,
        }],
    )

    # ---------- ros_gz_bridge ----------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # ---------- Spawn robot from robot_description topic ----------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_mobile_robot',
        arguments=[
            '-name',  LaunchConfiguration('robot_name'),
            '-topic', '/robot_description',
            '-x',     LaunchConfiguration('x_pose'),
            '-y',     LaunchConfiguration('y_pose'),
            '-z',     LaunchConfiguration('z_pose'),
            '-Y',     LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    # gz sim needs mobile_robot_description's share parent so it can resolve
    # model://mobile_robot_description/meshes/... URIs (converted from package://)
    set_gz_resource_desc = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(pkg_mobile_robot_desc).parent))

    return LaunchDescription([
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        declare_robot_name,
        set_gz_resource_desc,
        robot_state_publisher,
        bridge,
        spawn_robot,
    ])
