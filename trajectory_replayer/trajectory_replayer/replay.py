# trajectory_replayer/replay.py

import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

bag_path = 'src/trajectory_replayer/recording/recording'  # relative to workspace root

class TrajectoryReplayer(Node):
    def __init__(self):
        super().__init__('replay')
        # declare and read bag path parameter
        joint_names, points = self.load_trajectory(bag_path)

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/fr3_arm_controller/follow_joint_trajectory'
        )
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()
        self.send_goal(joint_names, points)

    def load_trajectory(self, bag_path: str):
        """Read JointState messages from a ros2 bag and convert to trajectory points."""
        storage_opts = StorageOptions(uri=bag_path, storage_id='sqlite3')
        conv_opts = ConverterOptions('', '')
        reader = SequentialReader()
        reader.open(storage_opts, conv_opts)
        reader.set_filter_topic_names(['/franka3/joint_states'])

        base_stamp = None
        joint_names = []
        points = []
        while reader.has_next():
            topic, data, t = reader.read_next()
            msg = deserialize_message(data, JointState)
            if not joint_names:
                joint_names = list(msg.name)

            sec = msg.header.stamp.sec
            nsec = msg.header.stamp.nanosec
            if base_stamp is None:
                base_stamp = msg.header.stamp
                elapsed_sec, elapsed_nsec = 0, 0
            else:
                elapsed_sec = sec - base_stamp.sec
                elapsed_nsec = nsec - base_stamp.nanosec
                if elapsed_nsec < 0:
                    elapsed_sec -= 1
                    elapsed_nsec += 1_000_000_000

            elapsed = Duration(sec=elapsed_sec, nanosec=elapsed_nsec)
            pt = JointTrajectoryPoint()
            pt.positions = list(msg.position)
            pt.velocities = list(msg.velocity)
            pt.effort = list(msg.effort)
            pt.time_from_start = elapsed
            points.append(pt)

        self.get_logger().info(f"Loaded {len(points)} points from bag '{bag_path}'")
        return joint_names, points

    def send_goal(self, joint_names, points):
        """Send the trajectory built from bag file to the action server."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.goal_time_tolerance = Duration(sec=1)
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self.get_logger().info('Sending trajectory...')
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryReplayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
