#include "tools.h"
#include <iostream>

Eigen::Matrix3d skew(Eigen::Vector3d vec)
{
    Eigen::Matrix3d mat;

    mat << 0.0, -vec(2), vec(1),
        vec(2), 0.0, -vec(0),
        -vec(1), vec(0), 0.0;

    return mat;
}

Eigen::VectorXd convertVector2VectorXd(std::vector<double> vec)
{
    int len_of_vector = vec.size();

    Eigen::VectorXd vectorXd(len_of_vector);

    for (int i = 0; i < len_of_vector; i++)
    {
        vectorXd(i) = vec[i];
    }

    return vectorXd;
}

Eigen::Vector3d getSo3FromEulerAngle(std::vector<double> desired_euler_angle,
                                     std::vector<double> euler_angle)
{
    Eigen::Matrix3d desired_rot_mat = (Eigen::AngleAxisd(desired_euler_angle[2], Eigen::Vector3d(0, 0, 1)) *
                                       Eigen::AngleAxisd(desired_euler_angle[1], Eigen::Vector3d(0, 1, 0)) *
                                       Eigen::AngleAxisd(desired_euler_angle[0], Eigen::Vector3d(1, 0, 0)))
                                          .matrix();

    Eigen::Matrix3d rot_mat = (Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d(0, 0, 1)) *
                               Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d(0, 1, 0)) *
                               Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d(1, 0, 0)))
                                  .matrix();

    Eigen::Matrix3d error_rot_mat = rot_mat.transpose() * desired_rot_mat;

    Eigen::AngleAxisd error_angle_axis;
    error_angle_axis = error_angle_axis.fromRotationMatrix(error_rot_mat);

    Eigen::Vector3d error_so3 = error_angle_axis.angle() * error_angle_axis.axis();

    error_so3 = rot_mat * error_so3;

    return error_so3;
}

Eigen::Matrix3d getInertialInGravityFrame(Eigen::Matrix3d inertial, std::vector<double> euler_angle)
{
    Eigen::Matrix3d rot_mat = (Eigen::AngleAxisd(0.0, Eigen::Vector3d(0, 0, 1)) *
                               Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d(0, 1, 0)) *
                               Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d(1, 0, 0)))
                                  .matrix();

    return rot_mat * inertial * rot_mat.transpose();
}