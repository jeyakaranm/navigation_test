#!/usr/bin/env python3
"""Launch the disturbance injector node.

A ``disturbance_case`` argument selects which subset of disturbances is active
so each scenario can be demonstrated / recorded independently:

  * case 1: full scenario without scan dropout (corruption burst held 5s) +
            pose noise + moving obstacle.
  * case 2: full scenario without scan corruption (stale/dropout scan held 5s) +
            pose noise + moving obstacle.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Which disturbances each case enables. Both cases are derived from the full
# staged scenario; case 1 drops the scan dropout, case 2 drops the corruption.
_CASES = {
    "1": {
        "enable_scan_corruption": True,
        "enable_pose_noise": True,
        "enable_moving_obstacle": True,
        "enable_scan_dropout": False,
        "corruption_duration": 5.0,
    },
    "2": {
        "enable_scan_corruption": False,
        "enable_pose_noise": True,
        "enable_moving_obstacle": True,
        "enable_scan_dropout": True,
        "dropout_delay": 0.0,
        "dropout_duration": 5.0,
    },
}


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    case = LaunchConfiguration("disturbance_case").perform(context)

    overrides = {"use_sim_time": use_sim_time}
    # Unrecognised cases keep the yaml enables (all on).
    overrides.update(_CASES.get(case, {}))

    node = Node(
        package="docking_disturbance",
        executable="disturbance_node",
        name="disturbance_node",
        output="screen",
        parameters=[params_file, overrides],
    )
    return [node]


def generate_launch_description():
    pkg_share = get_package_share_directory("docking_disturbance")
    default_params = os.path.join(pkg_share, "config", "disturbance_params.yaml")

    declare_args = [
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "disturbance_case",
            default_value="1",
            description="1=corruption(5s)+pose+obstacle, 2=dropout(5s)+pose+obstacle.",
        ),
    ]

    return LaunchDescription(declare_args + [OpaqueFunction(function=_launch_setup)])

