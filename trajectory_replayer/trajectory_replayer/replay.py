# trajectory_replayer/replay.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.duration import Duration as RclpyDuration
from builtin_interfaces.msg import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message


BAG_PATH = 'src/trajectory_replayer/recording/recording' 
JOINT_STATES_TOPIC = '/franka3/franka_robot_state_broadcaster/measured_joint_states'
ACTION_TOPIC = '/fr3_arm_controller/follow_joint_trajectory'
ALPHA_FILTER = 0.02  # Smoothing factor for exponential smoothing


class TrajectoryReplayer(Node):
    def __init__(self):
        super().__init__('replay')

        self.joint_names, self.points = self.load_trajectory(BAG_PATH)

        self.get_logger().info(f"Loaded {len(self.points)} trajectory points")
        for idx, pt in enumerate(self.points[:50]):
            self.get_logger().info(
                f"Point {idx}: "
                f"time={pt.time_from_start.sec}s{pt.time_from_start.nanosec}ns, "
                f"pos={pt.positions}, vel={pt.velocities}, acc={pt.accelerations}"
            )

        self._action_client = ActionClient(self, FollowJointTrajectory, ACTION_TOPIC)
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server is ready.")

        self.send_goal(self.joint_names, self.points)

    def load_trajectory(self, bag_path: str):
        """Read JointState messages from a ros2 bag and convert to trajectory points."""
        reader = SequentialReader()
        reader.open(StorageOptions(uri=bag_path, storage_id='sqlite3'), ConverterOptions('', ''))

        base_stamp = None
        joint_names = []
        raw_times, raw_positions, raw_vels = [], [], []
        points = []

        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic != JOINT_STATES_TOPIC:
                continue

            msg = deserialize_message(data, JointState)

            if not joint_names:
                joint_names = list(msg.name)

            stamp = msg.header.stamp
            if base_stamp is None:
                base_stamp = stamp
                elapsed_sec, elapsed_nsec = 0, 0
            else:
                elapsed_sec = stamp.sec - base_stamp.sec
                elapsed_nsec = stamp.nanosec - base_stamp.nanosec
                if elapsed_nsec < 0:
                    elapsed_sec -= 1
                    elapsed_nsec += 1_000_000_000

            elapsed = Duration(sec=elapsed_sec, nanosec=elapsed_nsec)
            t_float = elapsed_sec + elapsed_nsec * 1e-9

            pt = JointTrajectoryPoint()
            pt.positions = list(msg.position)
            pt.velocities = list(msg.velocity)
            pt.time_from_start = elapsed

            raw_times.append(t_float)
            raw_positions.append(pt.positions)
            raw_vels.append(pt.velocities)
            points.append(pt)

        # Filter invalid points (e.g., duplicate timestamps or incorrect lengths)
        filtered = []
        last_time = -1.0
        for t, pos, vel, pt in zip(raw_times, raw_positions, raw_vels, points):
            if t > last_time and len(pos) == len(joint_names) and len(vel) == len(joint_names):
                filtered.append((t, pos, vel, pt))
                last_time = t
            else:
                self.get_logger().warn(f"Skipping invalid point at time {t:.6f}")

        raw_times, raw_positions, raw_vels, points = map(list, zip(*filtered))

        # Exponential smoothing
        smoothed_pos = [raw_positions[0]]
        smoothed_vel = [raw_vels[0]]
        dof = len(joint_names)

        for i in range(1, len(raw_positions)):
            new_pos = [
                ALPHA_FILTER * raw_positions[i][j] + (1 - ALPHA_FILTER) * smoothed_pos[-1][j]
                for j in range(dof)
            ]
            new_vel = [
                ALPHA_FILTER * raw_vels[i][j] + (1 - ALPHA_FILTER) * smoothed_vel[-1][j]
                for j in range(dof)
            ]
            smoothed_pos.append(new_pos)
            smoothed_vel.append(new_vel)

        for i, pt in enumerate(points):
            pt.positions = smoothed_pos[i]
            pt.velocities = smoothed_vel[i]

        # Compute accelerations
        for i, pt in enumerate(points):
            if i == 0:
                pt.accelerations = [0.0] * dof
            else:
                dt = raw_times[i] - raw_times[i - 1]
                if dt <= 1e-9:
                    self.get_logger().warn(f"At point {i}: dt={dt:.9f} too small, zeroing acceleration")
                    pt.accelerations = [0.0] * dof
                else:
                    pt.accelerations = [
                        (raw_vels[i][j] - raw_vels[i - 1][j]) / dt
                        for j in range(dof)
                    ]

        return joint_names, points

    def send_goal(self, joint_names, points):
        """Send the trajectory to the action server."""
        goal_msg = FollowJointTrajectory.Goal()
        delay = RclpyDuration(nanoseconds=10_000_000)
        goal_msg.trajectory.header.stamp = (self.get_clock().now() + delay).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self.get_logger().info("Sending trajectory...")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            # feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.error_string}")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Received feedback: {feedback_msg.feedback.actual}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryReplayer()
    rclpy.spin(node)
    node.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
