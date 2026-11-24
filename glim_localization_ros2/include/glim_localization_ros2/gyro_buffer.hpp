// Copyright (c) 2026 atinfinity
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <iterator>
#include <utility>

#include "Eigen/Core"

namespace glim_localization_ros2
{

/// The last second of raw gyro samples, to give each frame the angular velocity of its stamp
/// (FrameEstimate::w_imu). Not thread-safe; the caller locks. Header only and free of ROS, so that
/// it can be tested alone.
class GyroBuffer {
public:
  /// A frame whose nearest gyro sample is farther than this [s] gets no angular velocity. About 2
  /// periods of a 100 Hz IMU, the slowest rate in common use; a faster IMU is held to the same
  /// bound. It also rejects a frame that the odometry handles more than the window after the IMU
  /// data it has seen (the window is counted back from the newest sample).
  static constexpr double MAX_GAP = 0.02;
  static constexpr double WINDOW = 1.0;  ///< [s], counted back from the newest sample

  void add(double stamp, const Eigen::Vector3d & gyro)
  {
    if (!std::isfinite(stamp)) {
      return;
    }
    if (!samples_.empty() && stamp < samples_.back().first) {
      // time went backwards (a bag played again): old samples are not neighbours any more
      samples_.clear();
    }
    samples_.emplace_back(stamp, gyro);
    while (samples_.front().first < stamp - WINDOW) {
      samples_.pop_front();
    }
  }

  /// The sample nearest to stamp, if it is within MAX_GAP. Returns false for an empty buffer or a
  /// non-finite stamp.
  bool nearest(double stamp, Eigen::Vector3d & gyro) const
  {
    if (samples_.empty() || !std::isfinite(stamp)) {
      return false;
    }
    const auto it = std::lower_bound(samples_.begin(), samples_.end(), stamp,
        [](const auto & s, double t) {return s.first < t;});
    auto best = it;
    if (it == samples_.end() ||
      (it != samples_.begin() &&
      std::abs(std::prev(it)->first - stamp) <= std::abs(it->first - stamp)))
    {
      best = std::prev(it);
    }
    if (std::abs(best->first - stamp) > MAX_GAP) {
      return false;
    }
    gyro = best->second;
    return true;
  }

  size_t size() const {return samples_.size();}

private:
  std::deque<std::pair<double, Eigen::Vector3d>> samples_;
};

}  // namespace glim_localization_ros2
