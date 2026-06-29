#!/usr/bin/env python3
"""Launch the LiDAR docking controller node.

Remaps scan/odom topics so the disturbance injector can be inserted in
front of the controller without code changes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("docking_controller")
    default_params = os.path.join(pkg_share, "config", "docking_params.yaml")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    auto_start = LaunchConfiguration("auto_start")

    declare_args = [
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("auto_start", default_value="false"),
    ]

    docking_node = Node(
        package="docking_controller",
        executable="docking_node",
        name="docking_node",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "scan_topic": scan_topic,
                "odom_topic": odom_topic,
                "auto_start": auto_start,
            },
        ],
    )

    return LaunchDescription(declare_args + [docking_node])
