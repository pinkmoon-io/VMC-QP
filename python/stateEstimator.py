import numpy as np

class StateEstimator:
    def __init__(self, robot):
        self._robot = robot
        self._A = np.mat(np.eye(18))

        self._dt = 0.005
        self._A[0:3, 3:6] = np.mat(np.eye(3)) * self._dt

        self._C = np.mat(np.zeros([24, 18]))

        for leg_id in range(4):
            self._C[3*leg_id: 3*leg_id+3, 0:3] = -np.mat(np.eye(3))
            self._C[3*leg_id:3*leg_id+3, 6+3*leg_id:9+3*leg_id] = np.mat(np.eye(3))
            self._C[3*leg_id+12:3*leg_id+15, 3:6] = -np.mat(np.eye(3))

        self._Xs = np.mat(np.zeros([18, 1]))

        self._Xs[2, 0] = 0.3
        self._Xs[6:18, 0] = np.mat([[0.17, -0.13, 0.0,
                                     0.17, 0.13, 0.0,
                                     -0.17, -0.13, 0.0,
                                     -0.17, 0.13, 0.0]]).transpose()

        self._Yo = np.mat(np.zeros([24, 1]))

        self._Xvar = np.mat(np.eye(18)) * 0.001
        self._Yvar = np.mat(np.eye(24)) * 0.001

        self._P = np.mat(np.eye(18)) * 0.001

    def run(self, current_time):
        foot_pos =self._robot.getFootPosInGravityFrame()

        foot_vel = self._robot.getFootVelocityInGravityFrame()


        angular_vel = self._robot.getRobotAngulurVelocity()

        leg_contact_state = self._robot.getFootContactState()

        for leg_id in range(4):
            rot_vel = np.cross(angular_vel, foot_pos[leg_id])

            for i in range(3):
                self._Yo[3*leg_id + i, 0] = foot_pos[leg_id][i]
                self._Yo[12+3*leg_id+i, 0] = foot_vel[3*leg_id+i] + rot_vel[i]



            if leg_contact_state[leg_id] == 0:
                self._Yvar[12+3*leg_id: 15+3*leg_id, 12+3*leg_id: 15+3*leg_id] = np.eye(3) * 100.0
            else:
                self._Yvar[12 + 3 * leg_id: 15 + 3 * leg_id, 12 + 3 * leg_id: 15 + 3 * leg_id] = np.eye(3) * 0.001

        x_k_k_1 = self._A * self._Xs
        z_k_k_1 = self._C * x_k_k_1
        P_k_k_1 = self._A * self._P * self._A.transpose() + self._Xvar
        K_k = P_k_k_1 * self._C.transpose()
        self._Xs = x_k_k_1 + K_k * np.linalg.solve(self._C * P_k_k_1 * self._C.transpose() + self._Yvar, self._Yo - z_k_k_1)
        self._P = (np.eye(18) - K_k * self._C) * P_k_k_1

    def getEstimatedVelocity(self):
        estimated_velocity = [self._Xs[3, 0], self._Xs[4, 0], self._Xs[5, 0]]

        return estimated_velocity

    def getEstimatedPos(self):
        estimated_pos = [self._Xs[0, 0], self._Xs[1, 0], self._Xs[2, 0]]

        return estimated_pos











