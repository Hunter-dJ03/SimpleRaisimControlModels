#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set up paths to the necessary configuration files and packages
    robot_description_path = get_package_share_directory("quadruped_description")
    operation_path = get_package_share_directory("quadruped_bringup")
    simulation_path = get_package_share_directory("quadruped_simulation")
    
    # Load config for leg configuration
    leg_config = os.path.join(
        robot_description_path,
        'config',
        'leg_config.yaml'
        )
    
    # Load config for operationparameters
    operation_params = os.path.join(
        operation_path,
        'config',
        'operation.yaml'
        )
    
    # Load config for simulation parameters
    simulation_params = os.path.join(
        simulation_path,
        'config',
        'simulation.yaml'
        )
    
    # Include the Foxglove launch file
    foxglove_pkg_share     = get_package_share_directory("foxglove_bridge")
    foxglove_launch_file   = os.path.join(foxglove_pkg_share, "launch", "foxglove_bridge_launch.xml")

    # Foxglove include (runs immediately)
    foxglove = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(foxglove_launch_file),
        launch_arguments={"port": "8765"}.items()
    )

    # Setup raisim node
    raisim_node = Node(
        package="quadruped_simulation",
        executable="raisim_bridge",
        name="raisim_bridge",
        output="screen",
        parameters=[
            {"robot_description_path": robot_description_path},
            leg_config,
            operation_params,
            simulation_params
        ]
    )

    # Setup control node
    control_node = Node(
        package="quadruped_control",
        executable="control_node",
        name="pendulum_controller",
        output="screen",
        parameters=[
            leg_config, 
            operation_params,   
        ]
    )

    return LaunchDescription([
        # Start Foxglove immediately for visualization
        foxglove,

        # After a delay, start the raisim node and control node
        TimerAction(
            period=2.0,
            actions=[raisim_node, control_node]
        ),
    ])
