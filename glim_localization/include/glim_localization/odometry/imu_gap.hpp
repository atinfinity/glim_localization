// Derived from koide3/glim src/glim/odometry/async_odometry_estimation.cpp at commit 1a197c999e4865f5cab7c9cd25474128413ebbca (MIT License, Copyright (c) 2024 koide3)
#pragma once

#include <algorithm>
#include <cstddef>
#include <optional>

namespace glim_localization {

/// Why a frame cannot be integrated with the IMU data around it
enum class ImuCoverage {
  OK,               ///< The IMU data covers the frame
  INCOMPLETE,       ///< No sample reaches the end of the scan yet (or the IMU data ended); nothing can be judged
  GAP,              ///< Two consecutive samples are further apart than max_gap
  TOO_FEW_SAMPLES   ///< Fewer than 2 samples in (last_stamp, frame_stamp]
};

struct ImuCoverageResult {
  ImuCoverage kind = ImuCoverage::OK;
  double gap_begin = 0.0;    ///< GAP: stamp of the sample before the gap
  double gap_end = 0.0;      ///< GAP: stamp of the sample after the gap
  double gap_seconds = 0.0;  ///< Length of the missing IMU data: gap_end - gap_begin, or frame_stamp - last_stamp for TOO_FEW_SAMPLES

  bool ok() const { return kind == ImuCoverage::OK; }
  bool is_gap() const { return kind == ImuCoverage::GAP || kind == ImuCoverage::TOO_FEW_SAMPLES; }
};

/// The single definition of an IMU gap, used by ReplayRunner and by the localization odometry.
///
/// The upstream IMU integration holds one sample across a gap, which corrupts the estimate, so a frame is
/// not integrable if the window [last_stamp, scan_end] has two consecutive samples more than max_gap apart,
/// or if (last_stamp, frame_stamp] holds fewer than 2 samples. The reported gap is the longest one in the window. The window starts at the last sample at or
/// before last_stamp, so a gap that straddles last_stamp counts as a whole.
///
/// @param n            number of samples; stamp_at(k) must be ascending in k
/// @param last_stamp   stamp of the last estimated frame. Without one (before the first pose, or the first
///                     frame after a gap) only the window around the frame itself is checked.
/// @param tolerance    per-point times are float32, so scan_end can exceed the last sample by rounding
/// @param predecessor  stamp of the sample received just before sample 0 and already dropped from the history (a trim),
///                     if any. Without one, sample 0 is the first sample there is (after a fault, or the IMU started late).
template <typename StampAt>
ImuCoverageResult check_imu_coverage(
  const size_t n,
  StampAt stamp_at,
  const std::optional<double> last_stamp,
  const double frame_stamp,
  const double scan_end,
  const double max_gap,
  const double tolerance = 1e-6,
  const std::optional<double> predecessor = std::nullopt) {
  constexpr size_t npos = static_cast<size_t>(-1);
  // First index with stamp >= t, or n
  const auto lower = [&](double t) {
    size_t lo = 0, hi = n;
    while (lo < hi) {
      const size_t mid = lo + (hi - lo) / 2;
      if (stamp_at(mid) < t) {
        lo = mid + 1;
      } else {
        hi = mid;
      }
    }
    return lo;
  };
  // Last index with stamp <= t, or npos
  const auto last_at_or_before = [&](double t) {
    size_t lo = 0, hi = n;
    while (lo < hi) {
      const size_t mid = lo + (hi - lo) / 2;
      if (stamp_at(mid) <= t) {
        lo = mid + 1;
      } else {
        hi = mid;
      }
    }
    return lo == 0 ? npos : lo - 1;
  };

  ImuCoverageResult result;
  const size_t last = lower(scan_end - tolerance);
  if (last >= n) {
    result.kind = ImuCoverage::INCOMPLETE;
    return result;
  }

  size_t first = last_at_or_before(last_stamp.value_or(frame_stamp));
  if (first == npos) {
    if (!last_stamp) {
      // The IMU data starts after the frame: a leading hole
      result.kind = ImuCoverage::GAP;
      result.gap_begin = frame_stamp;
      result.gap_end = stamp_at(0);
      result.gap_seconds = result.gap_end - result.gap_begin;
      return result;
    }
    // No sample at or before last_stamp. If the history was only trimmed, the real hole is the distance between the
    // adjacent samples; if there was never an earlier sample (the queue was erased, or the first pose is older than the
    // IMU data), it starts at last_stamp. Upstream holds sample 0 across it either way.
    const double hole_begin = predecessor ? *predecessor : *last_stamp;
    const double leading = stamp_at(0) - hole_begin;
    if (leading > max_gap) {
      result.kind = ImuCoverage::GAP;
      result.gap_begin = hole_begin;
      result.gap_end = stamp_at(0);
      result.gap_seconds = leading;  // A longer gap further in the window still wins below
    }
    first = 0;
  }
  // The longest gap in the window decides: a short gap in front of a long one must not hide it.
  for (size_t k = first; k < last; k++) {
    const double gap = stamp_at(k + 1) - stamp_at(k);
    if (gap > max_gap && gap > result.gap_seconds) {
      result.kind = ImuCoverage::GAP;
      result.gap_begin = stamp_at(k);
      result.gap_end = stamp_at(k + 1);
      result.gap_seconds = gap;
    }
  }
  if (result.kind == ImuCoverage::GAP) {
    return result;
  }

  if (last_stamp) {
    // Samples in (last_stamp, frame_stamp]
    const auto index = [&](double t) { return last_at_or_before(t) == npos ? -1L : static_cast<long>(last_at_or_before(t)); };
    if (index(frame_stamp) - index(*last_stamp) < 2) {
      result.kind = ImuCoverage::TOO_FEW_SAMPLES;
      result.gap_begin = *last_stamp;
      result.gap_end = frame_stamp;
      result.gap_seconds = frame_stamp - *last_stamp;
      return result;
    }
  }
  return result;
}

/// The longest interval that upstream integrates across when it integrates (last_stamp, frame_stamp] from a queue of samples.
/// Mirrors integrate_imu: samples at or before last_stamp are skipped, each later sample is held to the next one,
/// the last one is held to frame_stamp, and samples after frame_stamp are not used.
/// @return the largest interval [s]; the whole span if no sample is in range
template <typename StampAt>
double longest_imu_hold(const size_t n, StampAt stamp_at, const double last_stamp, const double frame_stamp) {
  double prev = last_stamp;
  double worst = 0.0;
  for (size_t k = 0; k < n; k++) {
    const double stamp = stamp_at(k);
    if (stamp > frame_stamp) {
      break;
    }
    if (stamp <= prev) {
      continue;
    }
    worst = std::max(worst, stamp - prev);
    prev = stamp;
  }
  return std::max(worst, frame_stamp - prev);
}

}  // namespace glim_localization
