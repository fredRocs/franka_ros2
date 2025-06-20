1. Move to initial position 
ros2 launch franka_bringup example.launch.py controller_name:=move_to_start_example_controller

2. Start the recording
ros2 launch trajectory_replayer record_trajectory.launch.py 