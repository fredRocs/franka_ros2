from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    bag_path = 'src/trajectory_replayer/recording/recording'  # relative to workspace root

    # Delete existing bag file/folder if it exists
    remove_bag = ExecuteProcess(
        cmd=['rm', '-rf', bag_path],
        output='screen'
    )

    # Include franka_bringup with hardcoded arguments
    franka_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('franka_bringup'),
                'launch',
                'example.launch.py'
            ])
        ]),
        launch_arguments={
            'controller_name': 'gravity_compensation_example_controller'
        }.items()
    )

    # Start ros2 bag recorder for joint states
    record_bag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', bag_path, '/franka3/joint_states'
        ],
        output='screen'
    )

    return LaunchDescription([
        remove_bag,
        franka_bringup,
        record_bag
    ])
