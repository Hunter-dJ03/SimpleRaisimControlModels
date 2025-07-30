from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

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
    
    # Load config for operation parameters
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

    # Declare launch arguments for the configurations
    return LaunchDescription([
        # Setup raisim node
        Node(
            package='quadruped_simulation',
            executable='raisim_bridge',
            name='raisim_bridge',
            output='screen',
            parameters=[
                {
                "robot_description_path": robot_description_path,
                }, 
                leg_config,
                operation_params,
                simulation_params
            ]
        )
    ])
