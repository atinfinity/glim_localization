#pragma once

namespace glim_localization {

enum class LocalizationState { INITIALIZING, TRACKING, DEGRADED, LOST };

const char* to_string(LocalizationState state);

struct QualityMonitorParams {
  double degraded_inlier_ratio = 0.3;  ///< A frame is good if its inlier ratio is at least this value
  int tracking_frames = 5;             ///< Consecutive good frames to enter TRACKING
  int degraded_frames = 5;             ///< Consecutive bad frames to leave TRACKING
  int lost_frames = 50;                ///< Consecutive bad frames to enter LOST
};

/// State machine over per-frame inlier ratios. LOST is left only through reset().
/// A NaN ratio counts as a bad frame. Not thread-safe.
class QualityMonitor {
public:
  /// @throw std::invalid_argument if a frame count is not positive
  explicit QualityMonitor(const QualityMonitorParams& params = QualityMonitorParams());

  LocalizationState update(double inlier_ratio);
  void reset();

  LocalizationState state() const { return state_; }

private:
  QualityMonitorParams params_;
  LocalizationState state_ = LocalizationState::INITIALIZING;
  int good_count_ = 0;
  int bad_count_ = 0;
};

}  // namespace glim_localization
