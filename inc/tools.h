#ifndef _TOOLS_H_
#define _TOOLS_H_

#include <Eigen>

Eigen::Matrix3d skew(Eigen::Vector3d vec);

Eigen::VectorXd convertVector2VectorXd(std::vector<double> vec);
Eigen::Vector3d getSo3FromEulerAngle(std::vector<double> desired_euler_angle,
                                     std::vector<double> euler_angle);
Eigen::Matrix3d getInertialInGravityFrame(Eigen::Matrix3d inertial, std::vector<double> euler_angle);

#endif