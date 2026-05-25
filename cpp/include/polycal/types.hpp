#pragma once

#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace polycal {

using Mat3d = Eigen::Matrix3d;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Vec3d = Eigen::Vector3d;
using Vec6d = Eigen::Matrix<double, 6, 1>;

}  // namespace polycal
