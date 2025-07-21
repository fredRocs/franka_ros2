import numpy as np
import math

class MotionGenerator:
    def __init__(self, speed_factor, q_start, q_goal):
        assert 0 < speed_factor <= 1.0

        self.kJoints = 7
        self.q_start = np.array(q_start)
        self.q_goal = np.array(q_goal)
        self.delta_q = self.q_goal - self.q_start

        # Max values
        self.dq_max = np.array([2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5]) * speed_factor  # m/s
        self.ddq_max_start = np.array([5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0]) * speed_factor  # m/s^2
        self.ddq_max_goal = np.array([5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0]) * speed_factor  # m/s^2

        self.kDeltaQMotionFinished = 1e-6

        # Outputs
        self.dq_max_sync = np.zeros(self.kJoints)
        self.t_1_sync = np.zeros(self.kJoints)
        self.t_2_sync = np.zeros(self.kJoints)
        self.t_f_sync = np.zeros(self.kJoints)
        self.q_1 = np.zeros(self.kJoints)

        self.calculate_synchronized_values()

    def calculate_synchronized_values(self):
        dq_max_reach = self.dq_max.copy()
        t_f = np.zeros(self.kJoints)
        delta_t_2 = np.zeros(self.kJoints)
        t_1 = np.zeros(self.kJoints)
        delta_t_2_sync = np.zeros(self.kJoints)
        sign_delta_q = np.sign(self.delta_q)
        dq = abs(self.delta_q)

        for i in range(self.kJoints):
            if dq[i] < self.kDeltaQMotionFinished:
                continue

            min_dist = (3/4) * (self.dq_max[i] ** 2 / self.ddq_max_start[i]) + \
                       (3/4) * (self.dq_max[i] ** 2 / self.ddq_max_goal[i])

            if dq[i] < min_dist:
                dq_max_reach[i] = np.sqrt(
                    4.0 / 3.0 * dq[i] * (self.ddq_max_start[i] * self.ddq_max_goal[i]) /
                    (self.ddq_max_start[i] + self.ddq_max_goal[i])
                )

            t_1[i] = 1.5 * dq_max_reach[i] / self.ddq_max_start[i]
            delta_t_2[i] = 1.5 * dq_max_reach[i] / self.ddq_max_goal[i]
            t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + dq[i] / dq_max_reach[i]

        max_t_f = np.max(t_f)

        for i in range(self.kJoints):
            if dq[i] < self.kDeltaQMotionFinished:
                continue

            a = 1.5 / 2.0 * (self.ddq_max_goal[i] + self.ddq_max_start[i])
            b = -max_t_f * self.ddq_max_goal[i] * self.ddq_max_start[i]
            c = dq[i] * self.ddq_max_goal[i] * self.ddq_max_start[i]
            delta = max(0.0, b**2 - 4.0 * a * c)

            self.dq_max_sync[i] = (-b - np.sqrt(delta)) / (2.0 * a)
            self.t_1_sync[i] = 1.5 * self.dq_max_sync[i] / self.ddq_max_start[i]
            delta_t_2_sync[i] = 1.5 * self.dq_max_sync[i] / self.ddq_max_goal[i]
            self.t_f_sync[i] = self.t_1_sync[i] / 2.0 + delta_t_2_sync[i] / 2.0 + dq[i] / self.dq_max_sync[i]
            self.t_2_sync[i] = self.t_f_sync[i] - delta_t_2_sync[i]
            self.q_1[i] = self.dq_max_sync[i] * sign_delta_q[i] * 0.5 * self.t_1_sync[i]

    def get_desired_joint_positions(self, dt=0.01):
        positions = []
        velocities = []
        accelerations = []
        t = 0.0
        finished = False

        while not finished:
            delta_q_d = np.zeros(self.kJoints)
            q_dot = np.zeros(self.kJoints)
            q_ddot = np.zeros(self.kJoints)
            sign_delta_q = np.sign(self.delta_q)
            t_d = self.t_2_sync - self.t_1_sync
            delta_t_2_sync = self.t_f_sync - self.t_2_sync
            joint_motion_finished = [False] * self.kJoints

            for i in range(self.kJoints):
                if abs(self.delta_q[i]) < self.kDeltaQMotionFinished:
                    joint_motion_finished[i] = True
                    continue

                if t < self.t_1_sync[i]:
                    delta_q_d[i] = -1.0 / self.t_1_sync[i] ** 3 * self.dq_max_sync[i] * sign_delta_q[i] * \
                                   (0.5 * t - self.t_1_sync[i]) * t ** 3
                    q_dot[i] = - sign_delta_q[i] * self.dq_max_sync[i] / self.t_1_sync[i]**3 * \
                                    (2*t - 3*self.t_1_sync[i]) * t**2
                    q_ddot[i] = - sign_delta_q[i] * self.dq_max_sync[i] / self.t_1_sync[i]**3 * \
                                    (6*t**2 - 6*self.t_1_sync[i]*t)
                    
                elif self.t_1_sync[i] <= t < self.t_2_sync[i]:
                    delta_q_d[i] = self.q_1[i] + (t - self.t_1_sync[i]) * self.dq_max_sync[i] * sign_delta_q[i]
                    q_dot[i] = sign_delta_q[i] * self.dq_max_sync[i]
                    q_ddot[i] = 0

                elif self.t_2_sync[i] <= t < self.t_f_sync[i]:
                    delta_q_d[i] = self.delta_q[i] + 0.5 * (
                        1.0 / delta_t_2_sync[i] ** 3 *
                        (t - self.t_1_sync[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                        (t - self.t_1_sync[i] - t_d[i]) ** 3 +
                        (2.0 * t - 2.0 * self.t_1_sync[i] - delta_t_2_sync[i] - 2.0 * t_d[i])
                    ) * self.dq_max_sync[i] * sign_delta_q[i]
                    q_dot[i] = sign_delta_q[i] * self.dq_max_sync[i] / 2 * \
                                        (
                                            1 / delta_t_2_sync[i]**3 * (
                                                (t - self.t_2_sync[i])**3 + 3*(t - 2*delta_t_2_sync[i] - self.t_2_sync[i]) * \
                                                (t - self.t_2_sync[i])**2
                                            ) + 2
                                        )
                    q_ddot[i] = 3 * sign_delta_q[i] * self.dq_max_sync[i] / delta_t_2_sync[i]**3 * \
                                        (
                                            (t - self.t_2_sync[i])**2 + (t - 2*delta_t_2_sync[i] - self.t_2_sync[i]) * \
                                            (t - self.t_2_sync[i])
                                        )
                else:
                    delta_q_d[i] = self.delta_q[i]
                    q_dot[i] = 0
                    q_ddot[i] = 0
                    joint_motion_finished[i] = True

            positions.append(self.q_start + delta_q_d)
            velocities.append(q_dot)
            accelerations.append(q_ddot)
            finished = all(joint_motion_finished)
            t += dt

        return positions, velocities, accelerations
    

if __name__ == '__main__':
    q_start = [0,0,0,0,0,0,0]
    q_goal = [0, -math.pi/4, 0, -3/4 * math.pi, 0, math.pi/2, math.pi/4]

    speed_factor = 0.5
    motion_generator = MotionGenerator(speed_factor=speed_factor, q_start=q_start, q_goal=q_goal)
    dt = 0.01
    pos, vel, acc = motion_generator.get_desired_joint_positions(dt=dt)
    pos = np.array(pos).transpose()
    vel = np.array(vel).transpose()
    acc = np.array(acc).transpose()

    t = np.arange(len(pos[1])) * dt

    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(7, 3, figsize=(15, 2.5 * 7), sharex=True)
    fig.suptitle("Joint Trajectories: Position, Velocity, Acceleration", fontsize=16)

    for i in range(7):
        axs[i, 0].plot(t, pos[i])
        axs[i, 0].set_ylabel(f"Joint {i+1}")
        axs[i, 0].set_title("Position")

        axs[i, 1].plot(t, vel[i])
        axs[i, 1].set_title("Velocity")

        axs[i, 2].plot(t, acc[i])
        axs[i, 2].set_title("Acceleration")

    for ax in axs[-1, :]:
        ax.set_xlabel("Time [s]")

    plt.tight_layout(rect=[0, 0.03, 1, 0.97])
    plt.show()

