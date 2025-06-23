from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robot_description_path = get_package_share_directory("quadruped_description")
    operation_path = get_package_share_directory("quadruped_bringup")
    simulation_path = get_package_share_directory("quadruped_simulation")

    
    leg_config = os.path.join(
        robot_description_path,
        'config',
        'leg_config.yaml'
        )
    
    operation_params = os.path.join(
        operation_path,
        'config',
        'operation.yaml'
        )
    
    simulation_params = os.path.join(
        simulation_path,
        'config',
        'simulation.yaml'
        )

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     "time_step_ms",
        #     default_value="1.0",
        #     description="Simulation time step in milliseconds"
        # ),
        
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
