from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def check_bag_exists(context, *args, **kwargs):
    """Expects the parents path of bag_name as arg."""
    #bag_name = LaunchConfiguration('bag_name').perform(context)
    bag_name = LaunchConfiguration("bag_name").perform(context)
    bag_file = os.path.join(args[0], bag_name)
    if os.path.exists(bag_file):
        print(f"\nBag path '{bag_file}' already exists. Please remove it or choose a different name.\n")
        exit(1)
    return []

def generate_launch_description():
    # check and get bag name
    bag_path = 'src/trajectory_replayer/recording'  # relative to workspace root
    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='recording',
        description='Path for the output ros2 bag'
    )
    check_bag = OpaqueFunction(function=check_bag_exists, args=[bag_path])

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
            'ros2', 'bag', 'record', '-o', 
            PathJoinSubstitution([bag_path, LaunchConfiguration("bag_name")]),
            '/franka3/franka_robot_state_broadcaster/measured_joint_states'
        ],
        output='screen'
    )

    return LaunchDescription([
        bag_name_arg,
        check_bag,
        franka_bringup,
        record_bag
    ])
