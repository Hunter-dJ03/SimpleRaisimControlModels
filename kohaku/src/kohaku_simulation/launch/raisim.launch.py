from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_description_path = get_package_share_directory("kohaku_description")
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "time_step_ms",
            default_value="1.0",
            description="Simulation time step in milliseconds"
        ),
        
        Node(
            package='kohaku_simulation',
            executable='raisim_bridge',
            name='raisim_bridge',
            output='screen',
            parameters=[{
                "time_step_ms": LaunchConfiguration("time_step_ms"),
                "robot_description_path": robot_description_path,
            }]
        )
    ])
