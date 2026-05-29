#include "polycal/ekf.hpp"

namespace polycal {

ExtrinsicEKF::ExtrinsicEKF(const Sophus::SE3d& T_lc_init,
                           const Mat6d& P_init,
                           const Mat6d& Q)
    : x_(Vec6d::Zero()), P_(P_init), Q_(Q), T_lc_(T_lc_init) {}

void ExtrinsicEKF::predict(double dt) {
    P_ += Q_ * dt;
}

Vec6d ExtrinsicEKF::compute_residual(
    const Sophus::SE3d& T_cam_odom,
    const Sophus::SE3d& T_lidar_odom) {
    Sophus::SE3d predicted = T_lc_ * T_lidar_odom * T_lc_.inverse();
    Sophus::SE3d error = T_cam_odom.inverse() * predicted;
    return error.log();
}

Mat6d ExtrinsicEKF::compute_jacobian(
    const Sophus::SE3d& T_cam_odom,
    const Sophus::SE3d& T_lidar_odom) {
    // J_r(r)^{-1} approximated as Identity for small residuals.
    // Valid in the slow-drift operating regime (|r| << 1).
    // TODO Phase 2: implement closed-form J_r^{-1} per Sola et al.
    // arXiv:1812.01537 eq. 10.95 before certifying coverage results.
    return T_lc_.Adj() *
           (T_lidar_odom.inverse().Adj() - Mat6d::Identity());
}

void ExtrinsicEKF::update(const Sophus::SE3d& T_cam_odom,
                          const Sophus::SE3d& T_lidar_odom,
                          const Mat6d& R) {
    Mat6d H = compute_jacobian(T_cam_odom, T_lidar_odom);
    Vec6d r = compute_residual(T_cam_odom, T_lidar_odom);
    Mat6d S = H * P_ * H.transpose() + R;
    Mat6d K = P_ * H.transpose() * S.inverse();
    x_ = x_ - K * r;
    P_ = (Mat6d::Identity() - K * H) * P_;
    T_lc_ = T_lc_ * Sophus::SE3d::exp(x_);
    x_ = Vec6d::Zero();
}

ExtrinsicEKF::UpdateResult ExtrinsicEKF::update_with_cusum(
    const Sophus::SE3d& T_cam_odom,
    const Sophus::SE3d& T_lidar_odom,
    const Mat6d& R) {
    Mat6d H = compute_jacobian(T_cam_odom, T_lidar_odom);
    Vec6d r = compute_residual(T_cam_odom, T_lidar_odom);
    Mat6d S = H * P_ * H.transpose() + R;
    Mat6d K = P_ * H.transpose() * S.inverse();
    x_ = x_ - K * r;
    P_ = (Mat6d::Identity() - K * H) * P_;
    T_lc_ = T_lc_ * Sophus::SE3d::exp(x_);
    x_ = Vec6d::Zero();
    return {r, S};
}

}  // namespace polycal
