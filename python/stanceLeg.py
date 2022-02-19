import math
import sys

import numpy as np
import convexMPCController
import force_distribution
import time

import gait

mass = 108 / 9.8
# inertial = [0.01683993, 0.056579028, 0.064713601, 8.3902e-05, 0.000597679, 2.5134e-05]
inertial = [0.01683993, 0.056579028, 0.064713601, 0, 0, 0]
# planning_time_step = 0.03
# planning_horizon = 10
# weights = [10, 10, 0.2, 0.0, 0.0, 100, 0.0, 0.0, 1., 1., 1., 1., 0]
# weights = [25, 25, 0.2, 0.0, 0.0, 100, 0.0, 0.0, 1., 1., 1., 1., 0]
weights = [1, 1, 0.2, 1000, 1000, 5]
alpha = 1e-5
friction_coeff = 0.4

kp_torque = [1000.0, 1000.0, 1000.0]*3
kd_torque = [63.0, 63.0, 63.0]*3
kp_force = [10.0, 10.0, 100.0]*3
kd_force = [6.3, 6.3, 20.0]*3


class StanceLeg:
    def __init__(self, robot, gait_generator, state_estimator, ground_estimator):
        self._robot = robot
        self._ground_estimator = ground_estimator
        self._state_estimator = state_estimator
        self._gait_generator = gait_generator
        # self._mpc_controller = convexMPCController.ContactForceMpcCaculate(mass, inertial,planning_horizon,
        #                                                 planning_time_step,
        #                                                 weights, weight_alpha, friction_coeff)
        self._vmc_controller =force_distribution.VMC_QP(mass, inertial, friction_coeff,
                          kp_torque, kd_torque, kp_force, kd_force, weights, alpha)

    def run(self, current_time, desired_x_speed, desired_y_speed, desired_twist_speed, desired_robot_height):
        leg_contact_state = [int(0)]*4
        for leg_id in range(4):
            if self._gait_generator._desired_leg_state[leg_id] == gait.STANCE:
                leg_contact_state[leg_id] = int(1)


        # print(leg_contact_state)
        # if leg_contact_state == [int(0)]*4:
        #     print(current_time)
        #     print(leg_contact_state, '\n\n')
        #     sys.exit(0)


        # linear_velocity_in_gravity_frame = self._robot.getRobotLinearVelocity()
        linear_velocity_in_gravity_frame = self._state_estimator.getEstimatedVelocity()
        angular_velocity_in_gravity_frame = self._robot.getRobotAngulurVelocity()
        euler_angle = self._robot.getRobotEuler()
        euler_angle = list(euler_angle)
        euler_angle[2] = 0.0
        foot_pos_in_gravity_frame = self._robot.getFootPosInGravityFrame()
        foot_pos_in_gravity_frame = np.asarray(foot_pos_in_gravity_frame).flatten()
        robot_height = self._robot.getRobotPos()[2]

        desired_linear_velocity = self._ground_estimator._ground_posture_mat * np.mat([[desired_x_speed,
                                                                                        desired_y_speed,
                                                                                        0]]).transpose()
        desired_angular_velocity = [0.0, 0.0, desired_twist_speed]
        desired_euler_angle = [self._ground_estimator._ground_roll,
                               self._ground_estimator._ground_pitch,
                               0.0]

        # contact_force_trajectory = self._mpc_controller.getContactForceTrajectory(linear_velocity_in_gravity_frame,
        #                                                          angular_velocity_in_gravity_frame,
        #                                                          euler_angle,
        #                                                          foot_pos_in_gravity_frame,
        #                                                          leg_contact_state,
        #                                                          desired_linear_velocity,
        #                                                          desired_angular_velocity,
        #                                                          desired_euler_angle,
        #                                                          desired_robot_height)
        robot_pos = self._robot.getRobotPos()

        # start = time.clock()
        contact_force_trajectory = self._vmc_controller.getContactForce(0,
                                                                        robot_pos,
                                                                        euler_angle,
                                                                        linear_velocity_in_gravity_frame,
                                                                        angular_velocity_in_gravity_frame,
                                                                        foot_pos_in_gravity_frame,
                                                                        leg_contact_state,
                                                                        [0, 0, desired_robot_height],
                                                                        desired_euler_angle,
                                                                        desired_linear_velocity,
                                                                        desired_angular_velocity)
        # end = time.clock()
        # print(end - start)
        # print(desired_euler_angle)
        # print(euler_angle)
        # print('\n\n')
        # print(leg_contact_state)
        # for i in range(4):
        #     print(contact_force_trajectory[3 * i:3 * i + 3], '\n')
        # print('\n\n')

        for leg_id in range(4):
            if leg_contact_state[leg_id] == 0:
                continue

            self._robot.setFootForceInGravityFrame(leg_id, contact_force_trajectory[3*leg_id: 3*leg_id + 3])







