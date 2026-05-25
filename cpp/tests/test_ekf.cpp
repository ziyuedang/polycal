#include "polycal/ekf.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace {

polycal::Mat6d scaled_identity(double scale) {
    return scale * polycal::Mat6d::Identity();
}

Sophus::SE3d make_se3(const polycal::Vec6d& tangent) {
    return Sophus::SE3d::exp(tangent);
}

}  // namespace

TEST(ExtrinsicEKF, test_predict_covariance_grows) {
    const polycal::Mat6d P = scaled_identity(0.01);
    const polycal::Mat6d Q = scaled_identity(1e-6);
    polycal::ExtrinsicEKF ekf(Sophus::SE3d(), P, Q);

    ekf.predict(1.0);

    const polycal::Mat6d expected = scaled_identity(0.01 + 1e-6);
    EXPECT_TRUE(ekf.P().isApprox(expected, 1e-10));
}

TEST(ExtrinsicEKF, test_predict_state_unchanged) {
    const polycal::Mat6d P = scaled_identity(0.01);
    const polycal::Mat6d Q = scaled_identity(1e-6);
    polycal::ExtrinsicEKF ekf(Sophus::SE3d(), P, Q);

    ekf.predict(1.0);

    EXPECT_TRUE(ekf.x().isZero(1e-12));
}

TEST(ExtrinsicEKF, test_residual_zero_at_ground_truth) {
    polycal::Vec6d extrinsic_tangent;
    extrinsic_tangent << 0.1, -0.2, 0.3, 0.05, -0.03, 0.02;
    const Sophus::SE3d T_lc = make_se3(extrinsic_tangent);

    polycal::Vec6d odom_tangent;
    odom_tangent << 0.5, -0.1, 0.2, 0.2, -0.1, 0.15;
    const Sophus::SE3d T_lidar_odom = make_se3(odom_tangent);
    const Sophus::SE3d T_cam_odom = T_lc * T_lidar_odom * T_lc.inverse();

    polycal::ExtrinsicEKF ekf(T_lc, scaled_identity(0.01), scaled_identity(1e-6));

    EXPECT_TRUE(ekf.compute_residual(T_cam_odom, T_lidar_odom).isZero(1e-10));
}

TEST(ExtrinsicEKF, test_residual_nonzero_when_miscalibrated) {
    polycal::Vec6d true_tangent;
    true_tangent << 0.1, -0.2, 0.3, 0.05, -0.03, 0.02;
    const Sophus::SE3d T_lc_true = make_se3(true_tangent);

    polycal::Vec6d odom_tangent;
    odom_tangent << 0.5, -0.1, 0.2, 0.2, -0.1, 0.15;
    const Sophus::SE3d T_lidar_odom = make_se3(odom_tangent);
    const Sophus::SE3d T_cam_odom = T_lc_true * T_lidar_odom * T_lc_true.inverse();

    polycal::Vec6d perturbation;
    perturbation << 0.03, -0.02, 0.01, 0.03, 0.0, -0.02;
    const Sophus::SE3d T_lc_perturbed = T_lc_true * make_se3(perturbation);
    polycal::ExtrinsicEKF ekf(
        T_lc_perturbed, scaled_identity(0.01), scaled_identity(1e-6));

    EXPECT_GT(ekf.compute_residual(T_cam_odom, T_lidar_odom).norm(), 0.01);
}

TEST(ExtrinsicEKF, test_update_reduces_uncertainty) {
    polycal::Vec6d true_tangent;
    true_tangent << 0.02, -0.01, 0.03, 0.02, -0.01, 0.03;
    const Sophus::SE3d T_lc_true = make_se3(true_tangent);

    polycal::Vec6d odom_tangent;
    odom_tangent << 0.5, -0.1, 0.2, 0.2, -0.1, 0.15;
    const Sophus::SE3d T_lidar_odom = make_se3(odom_tangent);
    const Sophus::SE3d T_cam_odom = T_lc_true * T_lidar_odom * T_lc_true.inverse();

    polycal::ExtrinsicEKF ekf(Sophus::SE3d(), scaled_identity(0.01), scaled_identity(1e-6));
    ekf.predict(0.1);
    const double trace_before_update = ekf.P().trace();
    ekf.update(T_cam_odom, T_lidar_odom, scaled_identity(1e-6));

    EXPECT_LT(ekf.P().trace(), trace_before_update);
}

TEST(ExtrinsicEKF, test_update_converges_on_synthetic) {
    polycal::Vec6d true_tangent = polycal::Vec6d::Zero();
    true_tangent[5] = 0.1;
    const Sophus::SE3d T_lc_true = make_se3(true_tangent);

    polycal::ExtrinsicEKF ekf(Sophus::SE3d(), scaled_identity(1.0), scaled_identity(1e-8));
    const polycal::Mat6d R = scaled_identity(1e-4);

    for (int i = 0; i < 100; ++i) {
        polycal::Vec6d odom_tangent = polycal::Vec6d::Zero();
        odom_tangent[0] = 0.2 + 0.01 * std::sin(static_cast<double>(i));
        odom_tangent[1] = 0.1 + 0.01 * std::cos(static_cast<double>(i));
        odom_tangent[3] = 0.05 + 0.005 * std::sin(0.5 * static_cast<double>(i));
        odom_tangent[4] = -0.04 + 0.005 * std::cos(0.5 * static_cast<double>(i));
        const Sophus::SE3d T_lidar_odom = make_se3(odom_tangent);
        const Sophus::SE3d T_cam_odom = T_lc_true * T_lidar_odom * T_lc_true.inverse();
        ekf.update(T_cam_odom, T_lidar_odom, R);
    }

    const Sophus::SE3d error = T_lc_true.inverse() * ekf.T_lc();
    const polycal::Vec6d error_tangent = error.log();
    EXPECT_LT(error_tangent.head<3>().norm(), 0.01);
    EXPECT_LT(error_tangent.tail<3>().norm(), 0.01);
}
