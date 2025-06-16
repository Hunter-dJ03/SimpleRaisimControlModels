#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # paths
    robot_description_dir = get_package_share_directory("simple_quadruped_leg_description")
    foxglove_pkg_share     = get_package_share_directory("foxglove_bridge")
    foxglove_launch_file   = os.path.join(foxglove_pkg_share, "launch", "foxglove_bridge_launch.xml")

    # Foxglove include (runs immediately)
    foxglove = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(foxglove_launch_file),
        launch_arguments={"port": "8765"}.items()
    )

    # Your nodes, but delayed until after Foxglove
    raisim_node = Node(
        package="simple_quadruped_leg_simulation",
        executable="raisim_bridge",
        name="raisim_bridge",
        output="screen",
        parameters=[
            {"time_step_ms": LaunchConfiguration("time_step_ms")},
            {"robot_description_path": robot_description_dir}
        ]
    )
    control_node = Node(
        package="simple_quadruped_leg_control",
        executable="control_node",
        name="pendulum_controller",
        output="screen"
    )

    return LaunchDescription([
        # still declare your argument up front
        DeclareLaunchArgument(
            "time_step_ms",
            default_value="1.0",
            description="Simulation time step in milliseconds"
        ),

        # 1) start Foxglove immediately
        foxglove,

        # 2) wait 2s, then fire off your two nodes
        TimerAction(
            period=2.0,
            actions=[raisim_node, control_node]
        ),
    ])
