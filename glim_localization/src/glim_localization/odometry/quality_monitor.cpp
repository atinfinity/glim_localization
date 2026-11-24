#include <glim_localization/odometry/quality_monitor.hpp>

#include <stdexcept>

namespace glim_localization {

const char* to_string(LocalizationState state) {
  switch (state) {
    case LocalizationState::INITIALIZING:
      return "INITIALIZING";
    case LocalizationState::TRACKING:
      return "TRACKING";
    case LocalizationState::DEGRADED:
      return "DEGRADED";
    case LocalizationState::LOST:
      return "LOST";
  }
  return "UNKNOWN";
}

QualityMonitor::QualityMonitor(const QualityMonitorParams& params) : params_(params) {
  if (params_.tracking_frames <= 0 || params_.degraded_frames <= 0 || params_.lost_frames <= 0) {
    throw std::invalid_argument("QualityMonitor frame counts must be positive");
  }
}

LocalizationState QualityMonitor::update(double inlier_ratio) {
  if (state_ == LocalizationState::LOST) {
    return state_;
  }

  // Written so that NaN is a bad frame.
  if (inlier_ratio >= params_.degraded_inlier_ratio) {
    good_count_++;
    bad_count_ = 0;
  } else {
    bad_count_++;
    good_count_ = 0;
  }

  if (bad_count_ >= params_.lost_frames) {
    state_ = LocalizationState::LOST;
  } else if (state_ == LocalizationState::TRACKING) {
    if (bad_count_ >= params_.degraded_frames) {
      state_ = LocalizationState::DEGRADED;
    }
  } else if (good_count_ >= params_.tracking_frames) {
    state_ = LocalizationState::TRACKING;
  }
  return state_;
}

void QualityMonitor::reset() {
  state_ = LocalizationState::INITIALIZING;
  good_count_ = 0;
  bad_count_ = 0;
}

}  // namespace glim_localization
