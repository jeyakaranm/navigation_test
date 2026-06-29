#!/usr/bin/env python3
"""End-to-end docking demo.

Brings up the Gazebo dock world + TurtleBot3, the docking controller, and
(optionally) the disturbance injector and RViz, plus a metrics node that
reports the final ground-truth pose error.

Examples:
  ros2 launch docking_bringup demo.launch.py
  ros2 launch docking_bringup demo.launch.py disturbances:=true rviz:=true
  ros2 launch docking_bringup demo.launch.py headless:=true disturbances:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("docking_sim")
    ctrl_share = get_package_share_directory("docking_controller")
    dist_share = get_package_share_directory("docking_disturbance")

    ctrl_params = os.path.join(ctrl_share, "config", "docking_params.yaml")
    dist_params = os.path.join(dist_share, "config", "disturbance_params.yaml")

    headless = LaunchConfiguration("headless")
    disturbances = LaunchConfiguration("disturbances")
    disturbance_case = LaunchConfiguration("disturbance_case")
    rviz = LaunchConfiguration("rviz")
    start_delay = LaunchConfiguration("start_delay")
    auto_start = LaunchConfiguration("auto_start")

    declare_args = [
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("disturbances", default_value="false"),
        DeclareLaunchArgument(
            "disturbance_case",
            default_value="1",
            description="1=corruption(5s)+pose+obstacle, 2=dropout(5s)+pose+obstacle.",
        ),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "start_delay",
            default_value="8.0",
            description="Seconds to wait (sim + RViz settle) before the "
            "controller starts driving.",
        ),
        DeclareLaunchArgument(
            "auto_start",
            default_value="false",
            description="If true, the controller begins docking automatically; "
            "otherwise call /docking/start.",
        ),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, "launch", "docking_sim.launch.py")
        ),
        launch_arguments={"headless": headless}.items(),
    )

    # Baseline: controller consumes the raw sim topics.
    controller_clean = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_share, "launch", "docking.launch.py")
        ),
        launch_arguments={
            "params_file": ctrl_params,
            "scan_topic": "/scan",
            "odom_topic": "/odom",
            "auto_start": auto_start,
        }.items(),
        condition=UnlessCondition(disturbances),
    )

    # Disturbed: injector republishes to *_dock, controller consumes those.
    disturbance = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dist_share, "launch", "disturbance.launch.py")
        ),
        launch_arguments={
            "params_file": dist_params,
            "disturbance_case": disturbance_case,
        }.items(),
        condition=IfCondition(disturbances),
    )
    controller_disturbed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_share, "launch", "docking.launch.py")
        ),
        launch_arguments={
            "params_file": ctrl_params,
            "scan_topic": "/scan_dock",
            "odom_topic": "/odom_dock",
            "auto_start": auto_start,
        }.items(),
        condition=IfCondition(disturbances),
    )

    metrics = Node(
        package="docking_bringup",
        executable="dock_metrics.py",
        name="dock_metrics",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(sim_share, "rviz", "docking.rviz")],
        condition=IfCondition(rviz),
    )

    # Give Gazebo (and RViz) time to settle, and let the robot sit at its
    # offset start pose, before the controller starts driving.
    delayed = TimerAction(
        period=start_delay,
        actions=[controller_clean, disturbance, controller_disturbed, metrics],
    )

    return LaunchDescription(declare_args + [sim, rviz_node, delayed])
