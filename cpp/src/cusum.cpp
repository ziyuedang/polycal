#include "polycal/cusum.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace polycal {

CUSUMDetector::CUSUMDetector(const CUSUMConfig& config)
    : config_(config) {}

std::pair<bool, double> CUSUMDetector::update(
    const Vec6d& innovation,
    const Mat6d& innovation_cov) {
    // Mahalanobis distance squared: nu^T S^{-1} nu
    double mahal_sq = innovation.transpose() *
                      innovation_cov.inverse() * innovation;

    // Normalized: chi^2(dof)/dof ~ 1.0 under no drift
    double normalized = mahal_sq / config_.dof;

    // CUSUM: accumulate excess above kappa, floor at 0
    g_ = std::max(0.0, g_ + normalized - config_.kappa);
    ++step_;

    bool alarm = g_ > config_.threshold;
    if (alarm && alarm_step_ < 0) {
        alarm_step_ = step_;
    }
    return {alarm, g_};
}

void CUSUMDetector::reset() {
    g_ = 0.0;
    alarm_step_ = -1;
    // Do not reset step_ -- preserves total step count.
}

Mat6d CUSUMDetector::compute_innovation_cov(
    const Mat6d& H, const Mat6d& P, const Mat6d& R) {
    return H * P * H.transpose() + R;
}

CUSUMCalibration CUSUMDetector::calibrate_kappa(
    const std::vector<Vec6d>& innovations,
    const std::vector<Mat6d>& covs,
    double target_percentile) {
    assert(!innovations.empty());
    assert(innovations.size() == covs.size());

    int dof = static_cast<int>(innovations[0].size());
    std::vector<double> normalized;
    normalized.reserve(innovations.size());

    for (size_t i = 0; i < innovations.size(); ++i) {
        double mahal_sq = innovations[i].transpose() *
                          covs[i].inverse() * innovations[i];
        normalized.push_back(mahal_sq / dof);
    }

    // Compute mean and std
    double mean = 0.0;
    for (double v : normalized) {
        mean += v;
    }
    mean /= normalized.size();

    double variance = 0.0;
    for (double v : normalized) {
        variance += (v - mean) * (v - mean);
    }
    variance /= normalized.size();

    // Percentile via sorting
    std::vector<double> sorted = normalized;
    std::sort(sorted.begin(), sorted.end());
    size_t idx = static_cast<size_t>(
        target_percentile * (sorted.size() - 1));
    double kappa = sorted[idx];

    return CUSUMCalibration{
        kappa,
        mean,
        std::sqrt(variance),
        target_percentile,
        static_cast<int>(normalized.size())
    };
}

}  // namespace polycal
