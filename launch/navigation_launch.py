"""
launch/navigation_launch.py — ROS 2 launch file that brings up:

  1. slam_toolbox  (online_async_launch.py with custom params)
  2. sensor_fusion_node
  3. navigation_node
  4. recovery_node

Usage
-----
    ros2 launch launch/navigation_launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node as LaunchNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── Paths ──────────────────────────────────
    project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    slam_params_file = os.path.join(project_dir, "config", "slam_toolbox_params.yaml")

    # ── Launch arguments ──────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock (set true for Gazebo).",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── 1. SLAM Toolbox ──────────────────────
    slam_toolbox_launch = None
    try:
        slam_share = get_package_share_directory("slam_toolbox")
        slam_toolbox_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, "launch", "online_async_launch.py")
            ),
            launch_arguments={
                "slam_params_file": slam_params_file,
                "use_sim_time": use_sim_time,
            }.items(),
        )
    except Exception:
        # slam_toolbox may not be installed — warn at runtime
        pass

    # ── 2. Sensor Fusion Node ────────────────
    sensor_fusion = LaunchNode(
        package=None,                       # standalone script, not a ROS pkg
        executable=None,
        name="sensor_fusion_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        # Launch as a Python script
        prefix=f"python3 {os.path.join(project_dir, 'nodes', 'sensor_fusion_node.py')}",
    )

    # For standalone Python scripts we use ExecuteProcess instead:
    from launch.actions import ExecuteProcess

    sensor_fusion_proc = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(project_dir, "nodes", "sensor_fusion_node.py"),
        ],
        name="sensor_fusion_node",
        output="screen",
    )

    # ── 3. Navigation Node ───────────────────
    navigation_proc = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(project_dir, "nodes", "navigation_node.py"),
        ],
        name="navigation_node",
        output="screen",
    )

    # ── 4. Recovery Node ─────────────────────
    recovery_proc = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(project_dir, "nodes", "recovery_node.py"),
        ],
        name="recovery_node",
        output="screen",
    )

    # ── 5. Cmd Vel Mux ───────────────────────
    cmd_vel_mux_proc = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(project_dir, "nodes", "cmd_vel_mux_node.py"),
        ],
        name="cmd_vel_mux_node",
        output="screen",
    )

    # ── Assemble ─────────────────────────────
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)

    if slam_toolbox_launch is not None:
        ld.add_action(slam_toolbox_launch)

    ld.add_action(sensor_fusion_proc)
    ld.add_action(navigation_proc)
    ld.add_action(recovery_proc)
    ld.add_action(cmd_vel_mux_proc)

    return ld
