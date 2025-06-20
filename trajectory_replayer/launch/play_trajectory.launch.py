from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import yaml
import os

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    trajectory_replayer_pkg = get_package_share_directory("trajectory_replayer")

    # Launch args
    robot_ip = LaunchConfiguration("robot_ip")

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        trajectory_replayer_pkg, 'config/fr3_controllers.yaml'
    )

    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }
    
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="0.0.0.0"),

        # Launch franka_bringup (bringup includes ros2_control node)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("franka_bringup"),
                    "launch",
                    "example.launch.py"
                ])
            ]),
            launch_arguments={
                "robot_ip": robot_ip,
                "controller_name": "fr3_arm_controller"  # initial controller
            }.items()
        ),

        # Spawn necessary controllers (joint_state + fr3_arm)
        ExecuteProcess(
            cmd=[
                "ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster",
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "60"
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "controller_manager", "spawner", "fr3_arm_controller",
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "60"
            ],
            output="screen"
        ),

        # Launch the replay node (FollowJointTrajectory client)
        Node(
            package="trajectory_replayer",
            executable="replay",
            name="replay",
            output="screen"
        )
    ])
