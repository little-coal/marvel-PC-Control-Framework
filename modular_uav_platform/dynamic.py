import time

import numpy as np
from utils import rpy2quat


class Dynamic():
    def __init__(self):
        self.v_acc = np.array([0.0, 0.0, 0.0])
        self.omega_acc = np.array([0.0, 0.0, 0.0])

        self.update_time = 0.01  # 100Hz

        self.m = 0.13
        self.Lw = 0.185  # from system center to qc center
        self.IBxy = 2 * 0.036 * self.Lw ** 2
        self.IBz = 4 * 0.036 * self.Lw ** 2
        self.IB = np.diag([self.IBxy, self.IBxy, self.IBz])  # total inertia

        self.controller_init_time = time.time()
        self.last_loop_time = time.time()
        self.current_time = self.last_loop_time

        self.alpha = np.array([0.0, 0.0, 0.0])
        self.beta = np.array([0.0, 0.0, 0.0])
        self.thrust = self.m * 9.81 / 3 * np.array([1, 1, 1])  # 42598

        self.Ar = np.array([])
        self.wrench_mapper_init()

        # feedback
        self.R_fb = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        self.pos_fb = np.array([0.0, 0.0, 0.2])
        self.vel_fb = np.array([0.0, 0.0, 0.0])
        self.quat_fb = np.array([1.0, 0.0, 0.0, 0.0])
        self.omega_fb = np.array([0.0, 0.0, 0.0])
        self.rpy_fb = np.array([0.0, 0.0, 0.0])

        # controller states
        self.u1 = np.array([0.0, 0.0, self.m * 9.81])
        self.u2 = np.array([0.0, 0.0, 0.0])
        self.epi = np.array([0.0, 0.0, 0.0])  # integral of pos error
        self.eri = np.array([0.0, 0.0, 0.0])  # integral of ang error
        self.q = self.m * 9.81 / 3 * np.array([0, 0, 1, 0, 0, 1, 0, 0, 1])  # thrust vector
        self.qprev = self.q  # previous thrust vector

    def wrench_mapper_init(self):
        # 偏移量矩阵（d1, d2, d3）
        BP = np.array([[-1 / 3 * self.Lw, 2 / 3 * self.Lw, 0], [-1 / 3 * self.Lw, -1 / 3 * self.Lw, 0],
                       [-2 / 3 * self.Lw, -1 / 3 * self.Lw, 0]])
        BPx = np.array([[], [], []])  # 变成叉乘矩阵
        for i in range(3):
            BPix = np.array([[0, -BP[i][2], BP[i][1]], [BP[i][2], 0, -BP[i][0]], [-BP[i][1], BP[i][0], 0]])
            BPx = np.concatenate((BPx, BPix), axis=1)

        # A: 叉乘矩阵 6x9
        A = np.concatenate(
            (np.concatenate((np.identity(3), np.identity(3), np.identity(3)), axis=1), BPx), axis=0)
        B1R = np.identity(3)
        B2R = np.identity(3)
        B3R = np.identity(3)

        # Ar:旋转矩阵[R1 0 0]
        #           [0 R2 0]        x A     6x9
        #           [0 0 R3] 9x9
        self.Ar = np.dot(A, np.concatenate((np.concatenate((B1R, np.zeros((3, 6))), axis=1),
                                            np.concatenate((np.zeros((3, 3)), B2R, np.zeros((3, 3))), axis=1),
                                            np.concatenate((np.zeros((3, 6)), B3R), axis=1)), axis=0))

    def push_states(self, pos_shared, vel_shared,
                    quat_shared, omega_shared):
        pos_shared[:] = self.pos_fb
        vel_shared[:] = self.vel_fb
        quat_shared[:] = self.quat_fb
        omega_shared[:] = self.omega_fb

    def pull_states(self, alpha_shared, beta_shared, thrust_shared):
        self.alpha = np.array(alpha_shared[:])
        self.beta = np.array(beta_shared[:])
        self.thrust = np.array(thrust_shared[:])

    def kinematics(self):
        # q_i = np.array([0.0, 0.0, 0.0])
        # for i in range(3):
        #     # fiz
        #     # q_i[i] = np.array([np.sin(self.beta[i]), -1.0*np.sin(self.alpha[i])*np.cos(self.beta[i]),
        #     #                  np.cos(self.alpha[i])*np.cos(self.beta[i])]) * self.thrust[i]
        #     q_i[i] = self.thrust[i] * np.cos(self.alpha[i]) * np.cos(self.beta[i])

        self.q = np.concatenate((self.thrust[0]*np.array([0.0, 0.0, 1.0]), self.thrust[1]*np.array([0.0, 0.0, 1.0]),
                                 self.thrust[2]*np.array([0.0, 0.0, 1.0])), axis=0)

        u = np.dot(self.Ar, self.q)
        self.u1 = u[0:3]
        self.u2 = u[3:6]
        # self.v_acc = u[0:3] / self.m + np.array([0.0, 0.0, 1.0]) * 9.81
        self.v_acc = u[0:3] / self.m
        IBT = np.dot(self.IB.T, np.linalg.inv(np.dot(self.IB, self.IB.T)))
        self.omega_acc = np.dot(IBT, u[3:6])

    def position_update(self):
        temp10 = self.current_time - self.last_loop_time
        self.pos_fb += self.vel_fb * temp10
        # quat?
        rpy = self.rpy_YZX(self.quat_fb[0], self.quat_fb[1], self.quat_fb[2], self.quat_fb[3])
        rpy += self.omega_fb * temp10
        self.quat_fb = rpy2quat(rpy[0], rpy[1], rpy[2])

    def vel_update(self):
        temp10 = self.current_time - self.last_loop_time
        self.vel_fb += self.v_acc * temp10
        self.omega_fb += self.omega_acc * temp10

    def rpy_YZX(self, quat0, quat1, quat2, quat3):
        rpy = np.array([0.0, 0.0, 0.0])
        roll = np.arctan2(-2 * (quat2 * quat3 - quat0 * quat1), 1 - 2 * (quat1 ** 2 + quat3 ** 2))
        pitch = np.arctan2(-2 * (quat1 * quat3 - quat0 * quat2), 1 - 2 * (quat2 ** 2 + quat3 ** 2))
        yaw = np.arcsin(2 * (quat1 * quat2 + quat0 * quat3))
        rpy = [roll, pitch, yaw]
        return rpy

    def update_states(self):
        # self.
        pass

    def step(self):
        self.kinematics()
        self.vel_update()
        self.position_update()

    def run(self, pos_shared, vel_shared, quat_shared, omega_shared,
            alpha_shared, beta_shared, thrust_shared, stop_shared):
        self.controller_start_time = time.time()
        self.last_loop_time = time.time()
        while 1:
            if stop_shared.value == 1:
                break
            self.current_time = time.time()
            if self.current_time - self.last_loop_time > self.update_time:
                self.pull_states(alpha_shared, beta_shared, thrust_shared)
                self.step()
                # take off
                if self.current_time - self.controller_start_time < 3:
                    self.thrust = self.thrust * (self.current_time - self.controller_start_time) / 3

                self.push_states(pos_shared, vel_shared, quat_shared, omega_shared)
                self.last_loop_time = self.current_time
                # self.log.append(self.dt)

            # time.sleep(0.001)
            else:
                time.sleep(0.0001)
