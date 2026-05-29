#pragma once

#include "polycal/types.hpp"

#include <utility>
#include <vector>

namespace polycal {

struct CUSUMConfig {
    // Reference value for false-alarm control.
    // Principled default: 1 + 0.85 * sqrt(2/dof) = 1.5 for dof=6.
    // kappa=1.0 is INVALID: chi^2(dof)/dof has mean 1.0, making
    // g_k a random walk with unpredictable false-alarm rate.
    // Reference: Page (1954) Biometrika 41(1/2):100-115.
    double kappa = 1.5;

    // Detection threshold. Alarm when g_k > threshold.
    // ARL_0 empirically ~29000 steps at kappa=1.5, threshold=5.0.
    double threshold = 5.0;

    // Degrees of freedom of the innovation vector.
    int dof = 6;
};

struct CUSUMCalibration {
    double kappa;
    double mean;
    double std_dev;
    double percentile;
    int n;
};

class CUSUMDetector {
public:
    explicit CUSUMDetector(const CUSUMConfig& config = CUSUMConfig{});

    // Update with one innovation vector.
    // innovation: shape (6,) EKF residual nu
    // innovation_cov: shape (6,6) S = H P H^T + R
    // Returns {alarm, current_statistic g_k}
    std::pair<bool, double> update(const Vec6d& innovation,
                                   const Mat6d& innovation_cov);

    // Reset CUSUM statistic to zero.
    void reset();

    // Current CUSUM statistic.
    double statistic() const { return g_; }

    // Step at which first alarm triggered. -1 if no alarm yet.
    int steps_to_alarm() const { return alarm_step_; }

    // S = H P H^T + R
    static Mat6d compute_innovation_cov(const Mat6d& H,
                                        const Mat6d& P,
                                        const Mat6d& R);

    // Empirical kappa calibration from no-drift sequence.
    // Precondition: innovations.size() == covs.size() >= 1.
    static CUSUMCalibration calibrate_kappa(
        const std::vector<Vec6d>& innovations,
        const std::vector<Mat6d>& covs,
        double target_percentile = 0.80);

private:
    CUSUMConfig config_;
    double g_ = 0.0;
    int step_ = 0;
    int alarm_step_ = -1;
};

}  // namespace polycal
