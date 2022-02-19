#ifndef _VMC_H_
#define _VMC_H_

#include <Eigen>
#include <vector>

const int VELOCITY_MODE = 0;
const int POSITION_MODE = 1;

const int ON_AIR = 0;
const int ON_GROUND = 1;

class VMC_QP
{
private:
    double mass_;
    Eigen::Matrix3d inertial_;
    double coef_friction_;

    std::vector<double> kp_torque_;
    std::vector<double> kd_torque_;

    std::vector<double> kp_force_;
    std::vector<double> kd_force_;

    double alpha_;
    Eigen::MatrixXd S;

    double robot_height_;

    void updateRobotHeight(std::vector<double> foot_pos, std::vector<int> leg_contact_state);

public:
    VMC_QP(double mass, std::vector<double> inertial,
           double coef_friction,
           std::vector<double> kp_torque,
           std::vector<double> kd_torque,
           std::vector<double> kp_force,
           std::vector<double> kd_force,
           std::vector<double> weights,
           double alpha);

    std::vector<double> getContactForce(int control_mode,
                                        std::vector<double> robot_position,
                                        std::vector<double> robot_euler,
                                        std::vector<double> robot_linear_velocity,
                                        std::vector<double> robot_angular_velocity,
                                        std::vector<double> foot_pos,
                                        std::vector<int> leg_contact_state,
                                        std::vector<double> desired_robot_position,
                                        std::vector<double> desired_robot_euler,
                                        std::vector<double> desired_linear_velocity,
                                        std::vector<double> desired_angular_velocity);
};

#endif