#include "polycal/ekf.hpp"
#include "polycal/cusum.hpp"

#include <cmath>
#include <random>
#include <vector>

#include <gtest/gtest.h>

namespace {

polycal::Mat6d scaled_identity(double scale) {
    return scale * polycal::Mat6d::Identity();
}

Sophus::SE3d make_se3(const polycal::Vec6d& tangent) {
    return Sophus::SE3d::exp(tangent);
}

polycal::Vec6d random_innovation(std::mt19937& rng) {
    std::normal_distribution<double> normal(0.0, 1.0);
    polycal::Vec6d innovation;
    for (int i = 0; i < 6; ++i) {
        innovation[i] = normal(rng);
    }
    return innovation;
}

polycal::Mat6d analytic_Q() {
    polycal::Vec6d diagonal;
    diagonal << 1e-8, 1e-8, 1e-8, 1e-7, 1e-7, 1e-7;
    return diagonal.asDiagonal();
}

polycal::Mat6d analytic_R() {
    polycal::Vec6d diagonal;
    diagonal << 2e-4, 2e-4, 2e-4, 2e-6, 2e-6, 2e-6;
    return diagonal.asDiagonal();
}

Sophus::SE3d add_small_noise(const Sophus::SE3d& T,
                             std::mt19937& rng) {
    std::normal_distribution<double> translation_noise(0.0, 1e-4);
    std::normal_distribution<double> rotation_noise(0.0, 1e-5);
    polycal::Vec6d noise = polycal::Vec6d::Zero();
    for (int i = 0; i < 3; ++i) {
        noise[i] = translation_noise(rng);
        noise[i + 3] = rotation_noise(rng);
    }
    return T * Sophus::SE3d::exp(noise);
}

Sophus::SE3d synthetic_lidar_odom(int step) {
    const double index = static_cast<double>(step);
    polycal::Vec6d tangent = polycal::Vec6d::Zero();
    tangent[0] = 0.2 + 0.03 * std::sin(0.07 * index);
    tangent[1] = 0.1 * std::cos(0.04 * index);
    tangent[3] = 0.2 * std::sin(0.03 * index);
    tangent[4] = 0.15 * std::cos(0.02 * index);
    tangent[5] = std::sin(index * 0.05) * 0.3;
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

TEST(CUSUMDetector, test_cusum_no_drift_no_alarm) {
    std::mt19937 rng(42);
    polycal::CUSUMDetector detector;
    const polycal::Mat6d innovation_cov = polycal::Mat6d::Identity();
    int alarm_count = 0;

    for (int i = 0; i < 1000; ++i) {
        const auto result = detector.update(random_innovation(rng), innovation_cov);
        if (result.first) {
            ++alarm_count;
            detector.reset();
        }
    }

    EXPECT_LT(alarm_count, 3);
}

TEST(CUSUMDetector, test_cusum_drift_triggers_alarm) {
    std::mt19937 rng(42);
    polycal::CUSUMDetector detector;
    const polycal::Mat6d innovation_cov = polycal::Mat6d::Identity();

    for (int i = 0; i < 200; ++i) {
        detector.update(random_innovation(rng), innovation_cov);
    }

    polycal::Vec6d drift = polycal::Vec6d::Zero();
    drift[3] = 3.0;
    for (int step = 1; step <= 50; ++step) {
        const auto result = detector.update(
            random_innovation(rng) + drift, innovation_cov);
        if (result.first) {
            EXPECT_LE(step, 50);
            return;
        }
    }

    FAIL() << "CUSUM did not alarm within 50 drift steps";
}

TEST(CUSUMDetector, test_cusum_reset) {
    polycal::CUSUMDetector detector(polycal::CUSUMConfig{1.5, 1.0, 6});
    const polycal::Mat6d innovation_cov = polycal::Mat6d::Identity();
    polycal::Vec6d large_innovation = polycal::Vec6d::Zero();
    large_innovation[0] = 5.0;

    const auto alarm_result = detector.update(large_innovation, innovation_cov);
    EXPECT_TRUE(alarm_result.first);

    detector.reset();
    EXPECT_DOUBLE_EQ(detector.statistic(), 0.0);
    const auto no_drift_result = detector.update(polycal::Vec6d::Zero(), innovation_cov);
    EXPECT_FALSE(no_drift_result.first);
}

TEST(CUSUMDetector, test_cusum_statistic_floors_at_zero) {
    polycal::CUSUMDetector detector;
    const polycal::Mat6d innovation_cov = polycal::Mat6d::Identity();
    const polycal::Vec6d innovation = polycal::Vec6d::Constant(1e-6);

    for (int i = 0; i < 100; ++i) {
        const auto result = detector.update(innovation, innovation_cov);
        EXPECT_DOUBLE_EQ(result.second, 0.0);
    }
}

TEST(CUSUMDetector, test_compute_innovation_cov) {
    const polycal::Mat6d H = polycal::Mat6d::Identity();
    const polycal::Mat6d P = 2.0 * polycal::Mat6d::Identity();
    const polycal::Mat6d R = polycal::Mat6d::Identity();
    const polycal::Mat6d S = polycal::CUSUMDetector::compute_innovation_cov(H, P, R);
    EXPECT_TRUE(S.isApprox(3.0 * polycal::Mat6d::Identity(), 1e-10));
}

TEST(CUSUMDetector, test_calibrate_kappa_basic) {
    std::mt19937 rng(42);
    std::vector<polycal::Vec6d> innovations;
    std::vector<polycal::Mat6d> covs;
    innovations.reserve(2000);
    covs.reserve(2000);

    for (int i = 0; i < 2000; ++i) {
        innovations.push_back(random_innovation(rng));
        covs.push_back(polycal::Mat6d::Identity());
    }

    const polycal::CUSUMCalibration result =
        polycal::CUSUMDetector::calibrate_kappa(innovations, covs, 0.80);
    EXPECT_GT(result.kappa, 1.2);
    EXPECT_LT(result.kappa, 1.7);
    EXPECT_EQ(result.n, 2000);
    EXPECT_GT(result.mean, 0.0);
}

TEST(CUSUMDetector, test_ekf_cusum_integration) {
    polycal::Vec6d true_tangent = polycal::Vec6d::Zero();
    true_tangent[0] = 0.03;
    true_tangent[1] = -0.02;
    true_tangent[5] = 0.1;
    const Sophus::SE3d T_lc_true = make_se3(true_tangent);

    polycal::Vec6d drift_tangent = polycal::Vec6d::Zero();
    drift_tangent[5] = 0.1;
    const Sophus::SE3d T_lc_drifted =
        T_lc_true * Sophus::SE3d::exp(drift_tangent);

    polycal::ExtrinsicEKF ekf(
        T_lc_true, 1e-4 * polycal::Mat6d::Identity(), analytic_Q());
    polycal::CUSUMDetector detector(polycal::CUSUMConfig{1.5, 5.0, 6});
    const polycal::Mat6d R_meas = analytic_R();
    std::mt19937 rng(7);

    int false_alarms = 0;
    for (int i = 0; i < 200; ++i) {
        const Sophus::SE3d T_lidar_odom = synthetic_lidar_odom(i);
        const Sophus::SE3d T_cam_odom =
            T_lc_true * T_lidar_odom * T_lc_true.inverse();
        ekf.predict(0.1);
        const polycal::ExtrinsicEKF::UpdateResult update_result =
            ekf.update_with_cusum(
                add_small_noise(T_cam_odom, rng),
                add_small_noise(T_lidar_odom, rng),
                R_meas);
        const auto cusum_result = detector.update(
            update_result.innovation, update_result.innovation_cov);
        false_alarms += cusum_result.first ? 1 : 0;
    }

    EXPECT_EQ(false_alarms, 0);

    for (int step = 1; step <= 200; ++step) {
        const Sophus::SE3d T_lidar_odom = synthetic_lidar_odom(200 + step);
        const Sophus::SE3d T_cam_odom =
            T_lc_drifted * T_lidar_odom * T_lc_drifted.inverse();
        ekf.predict(0.1);
        const polycal::ExtrinsicEKF::UpdateResult update_result =
            ekf.update_with_cusum(
                add_small_noise(T_cam_odom, rng),
                add_small_noise(T_lidar_odom, rng),
                R_meas);
        const auto cusum_result = detector.update(
            update_result.innovation, update_result.innovation_cov);
        if (cusum_result.first) {
            EXPECT_LE(step, 30);
            return;
        }
    }

    FAIL() << "CUSUM did not alarm within 30 drift steps";
}
