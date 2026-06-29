#!/usr/bin/env python3
"""Bring up the Gazebo docking world with a TurtleBot3 burger.

Starts gzserver (and gzclient unless headless), publishes the robot
description, and spawns the robot at the docking approach pose.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("docking_sim")
    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    headless = LaunchConfiguration("headless")
    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    yaw = LaunchConfiguration("yaw")
    world = LaunchConfiguration("world")

    default_world = os.path.join(sim_share, "worlds", "docking_world.world")
    # Local copy of the burger model with the LiDAR ray visualization disabled
    # (no blue scan beams in Gazebo); meshes still resolve via GAZEBO_MODEL_PATH.
    robot_sdf = os.path.join(
        sim_share, "models", "turtlebot3_burger", "model.sdf"
    )

    declare_args = [
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("x_pose", default_value="0.5"),
        DeclareLaunchArgument("y_pose", default_value="1.0"),
        DeclareLaunchArgument("yaw", default_value="0.7"),
        DeclareLaunchArgument("world", default_value=default_world),
    ]

    # Ensure Gazebo finds the bundled sun/ground_plane and TB3 models offline
    # (otherwise gzserver tries to download them and hangs).
    model_paths = [
        AppendEnvironmentVariable(
            "GAZEBO_MODEL_PATH", "/usr/share/gazebo-11/models"
        ),
        AppendEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            os.path.join(tb3_gazebo_share, "models"),
        ),
    ]

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world, "verbose": "false"}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")
        ),
        condition=UnlessCondition(headless),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                tb3_gazebo_share, "launch", "robot_state_publisher.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_turtlebot3",
        output="screen",
        arguments=[
            "-entity", "burger",
            "-file", robot_sdf,
            "-x", x_pose,
            "-y", y_pose,
            "-z", "0.01",
            "-Y", yaw,
        ],
    )

    return LaunchDescription(
        declare_args
        + model_paths
        + [gzserver, gzclient, robot_state_publisher, spawn_robot]
    )
