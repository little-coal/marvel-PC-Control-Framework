"""
Controller
"""
import numpy as np
import time

class Controller:
    def __init__(self):
        self.compensation_enabled = False
        # controller gains

        # smaller bandwidth
        self.Kpp = np.array([4.5, 4.5, 5])
        self.Kpi = np.array([2, 2, 2.5])
        self.Kpv = np.array([2.7, 2.5, 3])  # position pid

        self.Krr = np.array([17, 15, 8])
        self.Kri = np.array([6, 5.5, 3])
        self.Krw = np.array([5.5, 4.5, 3])  # attitude pid

        self.Krr2 = self.Krr
        self.Kri2 = self.Kri
        self.Krw2 = self.Krw

        self.reduced_alpha_margin1 = 1.04
        self.reduced_alpha_margin2 = 1.04

        # TODO: need modify
        # system parameters
        self.m = 0.13  # 0.036*4 + 0.012 # total mass
        self.Lh = 0.00  # vertical offset of CoM in negative z
        self.Lw = 0.185  # from system center to qc center
        self.IBxy = 2 * 0.036 * self.Lw ** 2
        self.IBz = 4 * 0.036 * self.Lw ** 2
        self.IB = np.diag([self.IBxy, self.IBxy, self.IBz])  # total inertia

        # other constants
        self.ArT = np.array([])
        self.ArTr = np.array([])
        self.wrench_mapper_init()

        # reference
        self.pos_ref = np.array([0.0, 0.0, 0.2])
        self.rpy_ref = np.array([0.0, 0.0, 0.0])
        self.vel_ref = np.array([0.0, 0.0, 0.0])
        self.agv_ref = np.array([0.0, 0.0, 0.0])

        # feedback
        self.pos_fb = np.array([0.0, 0.0, 0.2])
        self.vel_fb = np.array([0.0, 0.0, 0.0])
        self.quat_fb = np.array([1.0, 0.0, 0.0, 0.0])
        self.omega_fb = np.array([0.0, 0.0, 0.0])

        # system states
        # self.rpy_fb = np.array([0,0,0]) # roll pitch yaw
        self.R_fb = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])  # rotation matrix feedback
        self.R_ref = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])  # rotation matrix reference
        self.the = 0.0

        # controller states
        self.u1 = np.array([0.0, 0.0, self.m * 9.81])
        self.u2 = np.array([0.0, 0.0, 0.0])
        self.epi = np.array([0.0, 0.0, 0.0])  # integral of pos error
        self.eri = np.array([0.0, 0.0, 0.0])  # integral of ang error
        self.q = self.m * 9.81 / 3 * np.array([0, 0, 1, 0, 0, 1, 0, 0, 1])  # thrust vector
        self.qprev = self.q  # previous thrust vector

        # controller outputs
        self.alpha = np.array([0.0, 0.0, 0.0])
        self.beta = np.array([0.0, 0.0, 0.0])
        self.thrust = self.m * 9.81 / 3 * np.array([1, 1, 1])  # 42598

        # controller properties
        self.controller_rate = 0.01

        self.controller_init_time = time.time()
        self.last_loop_time = time.time()
        self.current_time = self.last_loop_time
        self.dt = 0
        self.log = []
        self.controller_start_time = 0.0

        self.debug_local = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                            # u1           u2              rpy          er           eri

        print("controller init time = %s" % self.controller_init_time)

    def wrench_mapper_init(self):
        # 偏移量矩阵（d1, d2, d3）
        BP = np.array([[-1/3 * self.Lw, 2/3 * self.Lw, 0], [-1/3 * self.Lw, -1/3 * self.Lw, 0], [-2/3 * self.Lw, -1/3 * self.Lw, 0]])
        BPx = np.array([[], [], []])    # 变成叉乘矩阵
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
        Ar = np.dot(A, np.concatenate((np.concatenate((B1R, np.zeros((3, 6))), axis=1),
                                       np.concatenate((np.zeros((3, 3)), B2R, np.zeros((3, 3))), axis=1),
                                       np.concatenate((np.zeros((3, 6)), B3R), axis=1)), axis=0))
        # 9x6, 广义逆， f = Ar-1 dot u
        self.ArT = np.dot(Ar.T, np.linalg.inv(np.dot(Ar, Ar.T)))

        # # TODO :为了去奇点去掉了Fiy列   7x6
        # Arr = np.concatenate((Ar[:, 0:1], Ar[:, 2:7], Ar[:, 8:9]), axis=1)
        # self.ArTr = np.dot(Arr.T, np.linalg.inv(np.dot(Arr, Arr.T)))

    def rpy_YZX(self, quat0, quat1, quat2, quat3):
        rpy = np.array([0.0, 0.0, 0.0])
        roll = np.arctan2(-2 * (quat2 * quat3 - quat0 * quat1), 1 - 2 * (quat1 ** 2 + quat3 ** 2))
        pitch = np.arctan2(-2 * (quat1 * quat3 - quat0 * quat2), 1 - 2 * (quat2 ** 2 + quat3 ** 2))
        yaw = np.arcsin(2 * (quat1 * quat2 + quat0 * quat3))
        rpy = [roll, pitch, yaw]
        return rpy

    def feedforward_downwash_compensation(self):
        maxForce = 0.08
        Lw = self.Lw
        angleZero = 21 * np.pi / 180
        angleFull = 5.5 * np.pi / 180
        dist = abs(2 * Lw * np.cos(self.the))
        distZero = Lw * 2 * np.sin(angleZero)
        distFull = Lw * 2 * np.sin(angleFull)
        if dist > distZero:
            fz = 0
        elif dist < distFull:
            fz = maxForce
        else:
            phase = (distZero - dist) / (distZero - distFull) * np.pi
            fz = -maxForce / 2 * np.cos(phase) + maxForce / 2
        return fz

    def pull_states(self, pos_ref_shared, rpy_ref_shared, vel_ref_shared, agv_ref_shared, pos_shared, vel_shared,
                    quat_shared, omega_shared):
        self.pos_ref = np.array(pos_ref_shared[:])
        self.rpy_ref = np.array(rpy_ref_shared[:])
        self.vel_ref = np.array(vel_ref_shared[:])
        self.agv_ref = np.array(agv_ref_shared[:])
        self.pos_fb = np.array(pos_shared[:])
        self.vel_fb = np.array(vel_shared[:])
        self.quat_fb = np.array(quat_shared[:])
        self.omega_fb = np.array(omega_shared[:])

    # print('pitch ref: %s agv ref: %s' % (self.rpy_ref[1], self.agv_ref[1]))

    def update_states(self):
        self.R_fb = np.array([[1 - 2 * (self.quat_fb[2] ** 2 + self.quat_fb[3] ** 2),
                               2 * (self.quat_fb[1] * self.quat_fb[2] - self.quat_fb[0] * self.quat_fb[3]),
                               2 * (self.quat_fb[0] * self.quat_fb[2] + self.quat_fb[1] * self.quat_fb[3])],
                              [2 * (self.quat_fb[1] * self.quat_fb[2] + self.quat_fb[0] * self.quat_fb[3]),
                               1 - 2 * (self.quat_fb[1] ** 2 + self.quat_fb[3] ** 2),
                               2 * (self.quat_fb[2] * self.quat_fb[3] - self.quat_fb[0] * self.quat_fb[1])],
                              [2 * (self.quat_fb[1] * self.quat_fb[3] - self.quat_fb[0] * self.quat_fb[2]),
                               2 * (self.quat_fb[0] * self.quat_fb[1] + self.quat_fb[2] * self.quat_fb[3]),
                               1 - 2 * (self.quat_fb[1] ** 2 + self.quat_fb[2] ** 2)]])
        Rx = np.array([[1, 0, 0], [0, np.cos(self.rpy_ref[0]), -np.sin(self.rpy_ref[0])],
                       [0, np.sin(self.rpy_ref[0]), np.cos(self.rpy_ref[0])]])
        Ry = np.array([[np.cos(self.rpy_ref[1]), 0, np.sin(self.rpy_ref[1])], [0, 1, 0],
                       [-np.sin(self.rpy_ref[1]), 0, np.cos(self.rpy_ref[1])]])
        Rz = np.array([[np.cos(self.rpy_ref[2]), -np.sin(self.rpy_ref[2]), 0],
                       [np.sin(self.rpy_ref[2]), np.cos(self.rpy_ref[2]), 0], [0, 0, 1]])
        self.R_ref = np.dot(np.dot(Rz, Ry), Rx)

        # phi = np.arctan2(self.R_fb[2,1], self.R_fb[2,2])
        # the = np.arcsin(-self.R_fb[2,0])
        # psi = np.arctan2(self.R_fb[1,0], self.R_fb[0,0])
        # self.the = the
        rpy = self.rpy_YZX(self.quat_fb[0], self.quat_fb[1], self.quat_fb[2], self.quat_fb[3])
        phi = rpy[0]
        the = rpy[1]
        psi = rpy[2]
        self.the = the

        self.debug_local[6:9] = [phi, the, psi]

    def position_controller(self):
        ep = self.pos_fb - self.pos_ref
        ev = self.vel_fb - self.vel_ref
        self.epi = self.epi + ep * 0.01
        fr = self.m * (9.81 * np.array([0, 0, 1]) - self.Kpp * ep - self.Kpv * ev - self.Kpi * self.epi)
        self.debug_local[0:3] = fr

        if self.compensation_enabled:
            fz = self.feedforward_downwash_compensation()
            fr += np.array([0, 0, fz])
            print(fz)

        self.u1 = np.dot(self.R_fb.T, fr)

    # print('epi = %s ' % self.epi)
    # print('error p = %s ' % ep)
    # print('u1 = %s' % self.u1)

    def attitude_controller(self):
        E = 1 / 2 * (np.dot(self.R_ref.T, self.R_fb) - np.dot(self.R_fb.T, self.R_ref))
        er = np.array([E[2][1], E[0][2], E[1][0]])
        ew = self.omega_fb - np.dot(self.R_fb.T, np.dot(self.R_ref, self.agv_ref))
        self.eri = self.eri + er * 0.01
        rho = np.array([0, 0, -self.Lh])
        h = np.cross(self.omega_fb, np.dot(self.IB, self.omega_fb))
        tau_g = -self.m * 9.81 * np.cross(rho, np.dot(self.R_fb.T, np.array([0, 0, 1])))
        self.u2 = h - tau_g - np.dot(self.IB, (self.Krr * er + self.Krw * ew + self.Kri * self.eri))
        # print('u2 = %s' % self.u2)

        self.debug_local[3:6] = self.u2
        self.debug_local[9:12] = er
        self.debug_local[12:15] = self.eri

    def wrench_mapper(self):
        self.qprev = self.q[:]
        pitch_judge = abs(self.rpy_ref[1])
        if pitch_judge > np.pi:
            pitch_judge -= np.pi
        self.q = np.dot(self.ArT, np.concatenate([self.u1, self.u2]))

        # TODO: 奇点先不考虑
        # if pitch_judge > (np.pi / 2 - self.reduced_alpha_margin1) and pitch_judge < (
        #         np.pi / 2 + self.reduced_alpha_margin2):
        #     qr = np.dot(self.ArTr, np.concatenate([self.u1, self.u2]))
        #     self.q = np.array([qr[0], 0,     qr[1],
        #                        qr[2], qr[3], qr[4],
        #                        qr[5], 0,     qr[6]])
        #     self.Krr = self.Krr2
        #     self.Kri = self.Kri2
        #     self.Krw = self.Krw2
        # else:
        #     self.q = np.dot(self.ArT, np.concatenate([self.u1, self.u2]))

    # print('value of q = %s' % self.q)

    def inverse_kinematics(self):
        temp10 = self.current_time - self.controller_start_time
        # print('===curr_time = %s========' % temp10)
        Xhat = np.array([1.0, 0.0, 0.0])
        Yhat = np.array([0.0, 1.0, 0.0])
        for i in range(3):
            aprev = self.alpha[i]
            bprev = self.beta[i]
            fdi = self.q[(3 * i):(3 * i + 3)]
            fdi_prev = self.qprev[(3 * i):(3 * i + 3)]
            self.thrust[i] = np.linalg.norm(fdi)

            tstar = fdi / np.linalg.norm(fdi)
            tprev = fdi_prev / np.linalg.norm(fdi)
            sp = np.array([0.0, -tprev[2], tprev[1]])
            sc = np.array([0.0, -tstar[2], tstar[1]])
            ainc = np.arctan2(np.linalg.norm(np.cross(sp, sc)), np.dot(sp, sc))
            asign = np.sign(np.dot(np.cross(sp, sc), Xhat))
            '''
            if ainc > 3.14: # flip condition
                ainc = np.pi - ainc
                asign = -asign
            '''
            Rx = np.array([[1, 0, 0], [0, np.cos(aprev), np.sin(aprev)], [0, -np.sin(aprev), np.cos(aprev)]])
            tp = np.dot(Rx, tprev)
            acurr = aprev + asign * ainc
            # print('i = %s ainc = %s asign = %s' % (i,ainc,asign))
            # print('alpha %s = %s' % (i,alpha))

            Rxn = np.array([[1, 0, 0], [0, np.cos(acurr), np.sin(acurr)], [0, -np.sin(acurr), np.cos(acurr)]])
            tn = np.dot(Rxn, tstar)
            binc = np.arctan2(np.linalg.norm(np.cross(tp, tn)), np.dot(tp, tn))
            bsign = np.sign(np.dot(np.cross(tp, tn), Yhat))
            temp1 = np.dot(np.cross(tp, tn), Yhat)
            temp2 = np.cross(tp, tn)
            bcurr = bprev + bsign * binc
            # print('i = %s binc = %s bsign = %s' % (i,binc,bsign))

            self.alpha[i] = acurr
            self.beta[i] = bcurr
        self.thrust = np.clip(self.thrust, 0, 0.6)

    def push_states(self, alpha_shared, beta_shared, thrust_shared):
        alpha_shared[:] = self.alpha
        beta_shared[:] = self.beta
        thrust_shared[:] = self.thrust

    # print('CMD: alpha = %s beta = %s thrust = %s' % (self.alpha,self.beta,self.thrust))

    def step(self):
        self.update_states()
        self.position_controller()
        self.attitude_controller()
        self.wrench_mapper()
        self.inverse_kinematics()

    def run(self,
            pos_ref_shared, rpy_ref_shared, vel_ref_shared, agv_ref_shared,
            pos_shared, vel_shared, quat_shared, omega_shared,
            alpha_shared, beta_shared, thrust_shared,
            debug1_shared, stop_shared):
        self.controller_start_time = time.time()
        self.last_loop_time = time.time()
        while 1:
            if stop_shared.value == 1:
                break
            self.current_time = time.time()
            if self.current_time - self.last_loop_time > self.controller_rate:
                self.pull_states(pos_ref_shared, rpy_ref_shared, vel_ref_shared, agv_ref_shared, pos_shared, vel_shared,
                                 quat_shared, omega_shared)
                self.step()

                # if self.current_time - self.controller_start_time > 11:
                # 	if self.current_time - self.controller_start_time < 11.8:
                # 		self.thrust = self.thrust + np.array([0.13, 0.15, -0.14, -0.13])

                # if self.current_time - self.controller_start_time > 11:
                # 	if self.current_time - self.controller_start_time < 11.3:
                # 		self.thrust = self.thrust + np.array([0.09, 0.11, -0.10, -0.09])

                # if self.current_time - self.controller_start_time > 11:
                # 	if self.current_time - self.controller_start_time < 11.4:
                # 		self.thrust = self.thrust + np.array([0.07, 0.09, -0.08, -0.07])

                # take off
                if self.current_time - self.controller_start_time < 3:
                    self.thrust = self.thrust * (self.current_time - self.controller_start_time) / 3

                self.push_states(alpha_shared, beta_shared, thrust_shared)
                temptime = self.current_time - self.last_loop_time
                # print('========================= %s' % temptime)
                # self.dt = self.current_time - self.last_loop_time
                self.last_loop_time = self.current_time
                # self.log.append(self.dt)
                debug1_shared[:] = self.debug_local

            # time.sleep(0.001)
            else:
                time.sleep(0.0001)

    # print(self.log)
