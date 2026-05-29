#pragma once

#include "polycal/types.hpp"

namespace polycal {

class ExtrinsicEKF {
public:
    struct UpdateResult {
        Vec6d innovation;
        Mat6d innovation_cov;
    };

    ExtrinsicEKF(const Sophus::SE3d& T_lc_init,
                 const Mat6d& P_init,
                 const Mat6d& Q);

    // Propagate covariance forward by dt seconds.
    void predict(double dt);

    // Compute hand-eye measurement residual.
    // T_cam_odom: camera odometry this step (SE3)
    // T_lidar_odom: LiDAR odometry this step (SE3)
    Vec6d compute_residual(const Sophus::SE3d& T_cam_odom,
                           const Sophus::SE3d& T_lidar_odom);

    // Compute measurement Jacobian (6x6).
    Mat6d compute_jacobian(const Sophus::SE3d& T_cam_odom,
                           const Sophus::SE3d& T_lidar_odom);

    // Full EKF update step. Calls compute_residual and compute_jacobian
    // internally. R is the 6x6 measurement noise covariance.
    void update(const Sophus::SE3d& T_cam_odom,
                const Sophus::SE3d& T_lidar_odom,
                const Mat6d& R);

    UpdateResult update_with_cusum(const Sophus::SE3d& T_cam_odom,
                                   const Sophus::SE3d& T_lidar_odom,
                                   const Mat6d& R);

    // Accessors
    Sophus::SE3d T_lc() const { return T_lc_; }
    Mat6d P() const { return P_; }
    Vec6d x() const { return x_; }

private:
    Vec6d x_;
    Mat6d P_;
    Mat6d Q_;
    Sophus::SE3d T_lc_;
};

}  // namespace polycal
