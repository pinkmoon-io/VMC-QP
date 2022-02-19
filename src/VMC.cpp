#include "VMC.h"
#include "tools.h"
#include "Goldfarb_Optimizer/QuadProg++.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

VMC_QP::VMC_QP(double mass, std::vector<double> inertial,
               double coef_friction,
               std::vector<double> kp_torque,
               std::vector<double> kd_torque,
               std::vector<double> kp_force,
               std::vector<double> kd_force,
               std::vector<double> weights,
               double alpha)
    : mass_(mass),
      coef_friction_(coef_friction),
      kp_torque_(kp_torque),
      kd_torque_(kd_torque),
      kp_force_(kp_force),
      kd_force_(kd_force),
      alpha_(alpha),
      S(6, 6),
      robot_height_(0.3)
{
    inertial_ << inertial[0], -inertial[3], -inertial[4],
        -inertial[3], inertial[1], -inertial[5],
        -inertial[4], -inertial[5], inertial[2];

    S.setZero();
    for (int i = 0; i < 6; i++)
    {
        S(i, i) = weights[i];
    }
}

std::vector<double> VMC_QP::getContactForce(int control_mode,
                                            std::vector<double> robot_position,
                                            std::vector<double> robot_euler,
                                            std::vector<double> robot_linear_velocity,
                                            std::vector<double> robot_angular_velocity,
                                            std::vector<double> foot_pos,
                                            std::vector<int> leg_contact_state,
                                            std::vector<double> desired_robot_position,
                                            std::vector<double> desired_robot_euler,
                                            std::vector<double> desired_linear_velocity,
                                            std::vector<double> desired_angular_velocity)
{
    updateRobotHeight(foot_pos, leg_contact_state);

    Eigen::Matrix3d kp_force_mat;
    Eigen::Matrix3d kd_force_mat;
    Eigen::Matrix3d kp_torque_mat;
    Eigen::Matrix3d kd_torque_mat;
    kp_force_mat.setZero();
    kd_force_mat.setZero();
    kp_torque_mat.setZero();
    kd_torque_mat.setZero();
    for (int i = 0; i < 3; i++)
    {
        kp_force_mat(i, i) = kp_force_[i];
        kd_force_mat(i, i) = kd_force_[i];
        kp_torque_mat(i, i) = kp_torque_[i];
        kd_torque_mat(i, i) = kd_torque_[i];
    }

    Eigen::Vector3d gravity(0.0, 0.0, -9.8);

    Eigen::Vector3d robot_pos = convertVector2VectorXd(robot_position);
    robot_pos[2] = robot_height_;
    Eigen::Vector3d robot_linear_vel = convertVector2VectorXd(robot_linear_velocity);
    Eigen::Vector3d robot_angular_vel = convertVector2VectorXd(robot_angular_velocity);

    Eigen::Vector3d deisred_robot_pos = convertVector2VectorXd(desired_robot_position);
    Eigen::Vector3d desired_linear_vel = convertVector2VectorXd(desired_linear_velocity);
    Eigen::Vector3d desired_angular_vel = convertVector2VectorXd(desired_angular_velocity);

    Eigen::Vector3d desired_force;
    desired_force.setZero();

    Eigen::Vector3d desired_linear_acc;
    desired_linear_acc.setZero();
    if (control_mode == POSITION_MODE)
    {
        desired_linear_acc =
            kp_force_mat * (deisred_robot_pos - robot_pos) - kd_force_mat * robot_linear_vel;
    }
    else if (control_mode == VELOCITY_MODE)
    {
        kp_force_mat(0, 0) = kp_force_mat(1, 1) = 0.0;
        desired_linear_acc = kp_force_mat * (deisred_robot_pos - robot_pos) + kd_force_mat * (desired_linear_vel - robot_linear_vel);
    }

    desired_force = mass_ * (desired_linear_acc - gravity);

    Eigen::Vector3d desired_torque;
    desired_torque.setZero();

    Eigen::Vector3d desired_angular_acc;
    desired_angular_acc.setZero();

    if (control_mode == POSITION_MODE)
    {
        Eigen::Vector3d error_so3 = getSo3FromEulerAngle(desired_robot_euler, robot_euler);
        desired_angular_acc = kp_torque_mat * error_so3 - kd_torque_mat * robot_angular_vel;
    }
    else if (control_mode == VELOCITY_MODE)
    {
        desired_robot_euler[2] = robot_euler[2] = 0.0;
        desired_angular_vel[0] = desired_angular_vel[1] = 0.0;
        Eigen::Vector3d error_so3 = getSo3FromEulerAngle(desired_robot_euler, robot_euler);
        desired_angular_acc = kp_torque_mat * error_so3 + kd_torque_mat * (desired_angular_vel - robot_angular_vel);
    }

    Eigen::Matrix3d inertial_in_gravity_frame = getInertialInGravityFrame(inertial_, robot_euler);
    desired_torque = inertial_in_gravity_frame * desired_angular_acc +
                     skew(robot_angular_vel) * inertial_in_gravity_frame * robot_angular_vel;
    // desired_torque = inertial_in_gravity_frame * desired_angular_acc;

    Eigen::MatrixXd A(6, 12);

    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        A.block(0, 3 * leg_id, 3, 3).setIdentity(3, 3);
        Eigen::Vector3d single_foot_pos(foot_pos[3 * leg_id], foot_pos[3 * leg_id + 1], foot_pos[3 * leg_id + 2]);
        A.block(3, 3 * leg_id, 3, 3) = skew(single_foot_pos);
    }

    Eigen::VectorXd b(6);

    b.head(3) = desired_force;
    b.tail(3) = desired_torque;

    int num_of_leg_on_ground = 0;

    for (auto &e : leg_contact_state)
    {
        num_of_leg_on_ground += e;
    }

    Eigen::MatrixXd G;
    Eigen::VectorXd g0;
    Eigen::MatrixXd CE(3 * (4 - num_of_leg_on_ground), 12);
    Eigen::VectorXd ce0(3 * (4 - num_of_leg_on_ground));
    Eigen::MatrixXd CI(5 * num_of_leg_on_ground, 12);
    Eigen::VectorXd ci0(5 * num_of_leg_on_ground);
    Eigen::VectorXd x(12);

    CE.setZero();
    ce0.setZero();
    CI.setZero();
    ci0.setZero();
    x.setZero();

    int leg_on_ground_index = 0;
    int leg_on_air_index = 0;
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state[leg_id] == 0)
        {
            CE.block(3 * leg_on_air_index, 3 * leg_id, 3, 3).setIdentity(3, 3);
            leg_on_air_index++;
        }
        else
        {
            CI.block(5 * leg_on_ground_index, 3 * leg_id, 5, 3) << 1.0, 0.0, coef_friction_,
                -1.0, 0.0, coef_friction_,
                0.0, 1.0, coef_friction_,
                0.0, -1.0, coef_friction_,
                0.0, 0.0, 1.0;
            leg_on_ground_index++;
        }
    }

    G = 2 * (A.transpose() * S * A + alpha_ * Eigen::MatrixXd::Identity(12, 12));
    g0 = -2 * b.transpose() * S * A;

    double f = solve_quadprog(G, g0, CE.transpose(), ce0, CI.transpose(), ci0, x);

    std::vector<double> desired_foot_force;

    x = -x;
    for (int i = 0; i < 12; i++)
    {
        desired_foot_force.push_back(x(i));
    }

    return desired_foot_force;
}

void VMC_QP::updateRobotHeight(std::vector<double> foot_pos, std::vector<int> leg_contact_state)
{
    int num_of_leg_on_ground = 0;

    double sum = 0.0;
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state[leg_id] == ON_GROUND)
        {
            sum += foot_pos[3 * leg_id + 2];
            num_of_leg_on_ground++;
        }
    }

    if (num_of_leg_on_ground != 0)
    {
        robot_height_ = fabs(sum) / num_of_leg_on_ground;
    }
}

PYBIND11_MODULE(force_distribution, m)
{

    py::class_<VMC_QP>(m, "VMC_QP")
        .def(py::init<double, std::vector<double>, double,
                      std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>,
                      std::vector<double>, double>())
        .def("getContactForce", &VMC_QP::getContactForce);
}