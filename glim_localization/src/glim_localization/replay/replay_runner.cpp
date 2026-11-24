// Derived from koide3/glim src/glim/odometry/async_odometry_estimation.cpp at commit 1a197c999e4865f5cab7c9cd25474128413ebbca (MIT License, Copyright (c) 2024 koide3)
#include <glim_localization/replay/replay_runner.hpp>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <vector>

#include <nlohmann/json.hpp>

namespace glim_localization {
namespace {

using Clock = std::chrono::steady_clock;

// Per-point times are float32, so scan_end_time can exceed the last sample by rounding
constexpr double kStampTolerance = 1e-6;

double elapsed_ms(Clock::time_point begin, Clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

}  // namespace

bool estimate_is_sane(const glim::EstimationFrame& frame) {
  return frame.T_world_imu.matrix().allFinite() && frame.v_world_imu.allFinite() && frame.imu_bias.allFinite() &&
         frame.v_world_imu.norm() <= kMaxPlausibleSpeed;
}

const char* to_string(ReplayResult result) {
  switch (result) {
    case ReplayResult::ESTIMATED:
      return "estimated";
    case ReplayResult::DROPPED:
      return "dropped";
    case ReplayResult::DIVERGED:
      return "diverged";
    case ReplayResult::SKIPPED_NO_IMU:
      return "skipped_no_imu";
  }
  return "unknown";
}

ReplayRunner::ReplayRunner(
  const ReplayDataset& dataset,
  OdometryEstimationLocalization& odometry,
  glim::CloudPreprocessor& preprocessor,
  const ReplayOptions& options)
: dataset_(dataset),
  odometry_(odometry),
  preprocessor_(preprocessor),
  options_(options) {}

ReplaySummary ReplayRunner::run(const std::function<void(const ReplayFrameRecord&)>& on_frame) {
  const auto run_begin = Clock::now();
  ReplaySummary summary;

  if (options_.initial_reset) {
    const auto& reset = *options_.initial_reset;
    if (!odometry_.reset(reset.T_world_imu, reset.v_world_imu, reset.min_stamp)) {
      throw std::invalid_argument("the initial pose is rejected (non-finite or not a rigid transform)");
    }
  }

  const auto& imu = dataset_.imu();
  const auto sample_stamp = [&](size_t k) { return imu[k].stamp_ns * 1e-9; };
  // Stamp of the last frame that produced a pose, or that ended a gap episode. The upstream IMU integration runs from there to the next frame.
  std::optional<double> last_pose_stamp;
  // A gap was skipped (OnImuGap::RESET) and the IMU has not been clean around a frame since
  bool gap_episode = false;
  // The initial pose has not been applied to a frame yet, and a frame since its min_stamp was skipped for the IMU
  bool init_pending = options_.initial_reset && options_.initial_reset->min_stamp > 0.0;
  bool init_waited_for_imu = false;
  int last_reset_count = 0;

  enum class Coverage { OK, SKIP };
  // Leading holes and the end of the IMU data skip the frame. After a pose was produced, a hole in
  // [last_pose_stamp, scan_end] would be integrated with one held sample and corrupt the estimate: abort,
  // or with OnImuGap::RESET skip the frames until the IMU is clean again. The odometry then resets itself
  // (it applies the same check). When the IMU is clean again the window restarts at that frame.
  const auto check_coverage = [&](double stamp, double scan_end) {
    const auto coverage_of = [&](std::optional<double> last) {
      return check_imu_coverage(imu.size(), sample_stamp, last, stamp, scan_end, options_.max_imu_gap, kStampTolerance);
    };
    const auto abort_run = [&](const std::string& what) {
      char text[96];
      std::snprintf(text, sizeof(text), "frame %.9f s: ", stamp);
      throw InvalidDatasetError(std::string(text) + what + "; replay a part of the data that has IMU with --start/--end, or fix the dataset");
    };

    const auto full = coverage_of(last_pose_stamp);
    if (full.kind == ImuCoverage::INCOMPLETE) {
      return Coverage::SKIP;  // The IMU data ended
    }
    if (gap_episode && coverage_of(std::nullopt).ok()) {
      // Clean around this frame, but the whole interval since the last pose counts: an outage with no frame in it
      // would otherwise be invisible. Past the limit it is handled below like any other gap.
      if (!full.is_gap() || full.gap_seconds <= options_.imu_gap_reset_max) {
        gap_episode = false;
        last_pose_stamp = stamp;  // The next window starts at this frame
        return Coverage::OK;
      }
    }
    if (full.ok()) {
      return Coverage::OK;
    }
    if (!last_pose_stamp) {
      return Coverage::SKIP;  // Leading hole: nothing is corrupted
    }
    if (options_.on_imu_gap == ReplayOptions::OnImuGap::RESET && full.gap_seconds <= options_.imu_gap_reset_max) {
      gap_episode = true;
      return Coverage::SKIP;
    }
    char text[256];
    if (full.kind == ImuCoverage::GAP) {
      std::snprintf(
        text, sizeof(text), "IMU gap of %.3f s between %.9f s and %.9f s (limit %.3f s)%s", full.gap_seconds, full.gap_begin, full.gap_end, options_.max_imu_gap,
        " after the last estimated frame or recovery");
    } else {
      std::snprintf(text, sizeof(text), "fewer than 2 IMU samples since the last estimated frame at %.9f s (%.3f s ago)", last_pose_stamp.value_or(0.0), full.gap_seconds);
    }
    abort_run(text);
    return Coverage::SKIP;
  };
  size_t imu_fed = 0;  // Number of samples handed to the odometry

  int64_t processed = 0;
  for (size_t index = 0; index < dataset_.frames().size(); index++) {
    const auto& info = dataset_.frames()[index];
    const double stamp = info.stamp_ns * 1e-9;
    if ((options_.start && stamp < *options_.start) || (options_.end && stamp > *options_.end)) {
      continue;
    }
    if (options_.max_frames >= 0 && processed >= options_.max_frames) {
      break;
    }
    processed++;

    ReplayFrameRecord record;
    record.stamp_ns = info.stamp_ns;
    bool diverged = false;

    auto t0 = Clock::now();
    const auto raw = dataset_.load_cloud(index);
    auto t1 = Clock::now();
    record.num_points_raw = raw->size();
    record.t_load_ms = elapsed_ms(t0, t1);

    const auto preprocessed = preprocessor_.preprocess(raw);
    t0 = Clock::now();
    record.num_points_preprocessed = preprocessed->size();
    record.t_preprocess_ms = elapsed_ms(t1, t0);

    // Frames before --init-stamp never reach the IMU integration (the odometry drops them first), so the IMU
    // coverage does not matter for them: they are `dropped`, whatever the IMU data looks like there.
    const bool before_init_stamp = options_.initial_reset && preprocessed->stamp < options_.initial_reset->min_stamp;
    if (!before_init_stamp && check_coverage(preprocessed->stamp, preprocessed->scan_end_time) == Coverage::SKIP) {
      record.result = ReplayResult::SKIPPED_NO_IMU;
      init_waited_for_imu = init_waited_for_imu || init_pending;
    } else {
      // An empty cloud never reaches the odometry, so it does not apply the pose and must not satisfy the delay check
      if (init_pending && !before_init_stamp && preprocessed->size() > 0) {
        // The odometry applies the initial pose to this frame
        const double delay = preprocessed->stamp - options_.initial_reset->min_stamp;
        if (delay > options_.max_init_pose_delay + 1e-9) {
          char text[256];
          std::snprintf(
            text, sizeof(text), "the initial pose is for %.9f s, but the first frame the IMU covers is at %.9f s, %.3f s later (limit %.3f s)",
            options_.initial_reset->min_stamp, preprocessed->stamp, delay, options_.max_init_pose_delay);
          throw std::invalid_argument(text);
        }
        if (init_waited_for_imu) {
          std::fprintf(stderr, "warning: the initial pose is applied %.3f s late because the IMU data does not cover the frames before\n", delay);
        }
        init_pending = false;
      }
      // Every sample up to scan_end_time, then the one after it.
      const auto after_end = std::upper_bound(
        imu.begin(), imu.end(), preprocessed->scan_end_time,
        [](double end_time, const ReplayImu& sample) { return end_time < sample.stamp_ns * 1e-9; });
      const size_t target = std::min(static_cast<size_t>(after_end - imu.begin()) + 1, imu.size());
      for (; imu_fed < target; imu_fed++) {
        odometry_.insert_imu(imu[imu_fed].stamp_ns * 1e-9, imu[imu_fed].acc, imu[imu_fed].gyro);
        record.num_imu_fed++;
      }

      if (preprocessed->size() > 0) {
        std::vector<glim::EstimationFrame::ConstPtr> marginalized;
        const auto t_insert = Clock::now();
        const auto estimate = odometry_.insert_frame(preprocessed, marginalized);
        record.t_odometry_ms = elapsed_ms(t_insert, Clock::now());
        if (estimate && !estimate_is_sane(*estimate)) {
          record.result = ReplayResult::DIVERGED;
          diverged = true;
        } else if (estimate) {
          record.result = ReplayResult::ESTIMATED;
          last_pose_stamp = preprocessed->stamp;
          record.T_world_imu = estimate->T_world_imu;
        }
      }
    }

    const auto status = odometry_.status();
    record.state = status.state;
    record.reset_count = status.reset_count;
    if (status.reset_count > last_reset_count && status.last_reset_reason == ResetReason::IMU_GAP) {
      summary.imu_gap_resets += status.reset_count - last_reset_count;
    }
    last_reset_count = status.reset_count;
    if (status.init_search != InitSearchResult::NONE) {
      summary.init_search = ReplaySummary::InitSearch{
        status.init_search,
        status.init_search_reason,
        status.init_search_candidates,
        status.init_search_candidates_total,
        status.init_search_score,
        status.init_search_yaw_offset,
        status.init_search_runner_up_score,
        status.init_search_runner_up_yaw_offset,
        status.init_search_time};
    }
    if (record.result == ReplayResult::ESTIMATED) {
      record.frame_id = status.frame_id;
      record.inlier_ratio = status.inlier_ratio;
      record.residual = status.residual;
      record.num_map_factors = status.num_map_factors;
      record.num_tiles = status.num_tiles;
      summary.estimated_by_state[to_string(record.state)]++;
    }
    summary.frames_by_result[to_string(record.result)]++;
    summary.num_frames++;

    if (on_frame) {
      on_frame(record);
    }
    if (diverged) {
      throw std::runtime_error("the estimate diverged at frame " + format_stamp_ns(record.stamp_ns) + " s (non-finite values or speed above 1000 m/s)");
    }
  }

  summary.wall_seconds = std::chrono::duration<double>(Clock::now() - run_begin).count();
  return summary;
}

ReplayOutputWriter::ReplayOutputWriter(const std::string& out_dir) : out_dir_(out_dir) {
  std::error_code ec;
  std::filesystem::create_directories(out_dir, ec);
  if (ec) {
    throw std::runtime_error("cannot create " + out_dir + ": " + ec.message());
  }
  std::filesystem::remove(out_dir + "/summary.json", ec);
  if (ec) {
    throw std::runtime_error("cannot remove the old summary.json in " + out_dir + ": " + ec.message());
  }
  traj_.open(out_dir + "/traj_localization.txt", std::ios::trunc);
  frames_.open(out_dir + "/frames.csv", std::ios::trunc);
  if (!traj_ || !frames_) {
    throw std::runtime_error("cannot create output files in " + out_dir);
  }
  frames_ << "stamp_ns,result,frame_id,state,reset_count,num_points_raw,num_points_preprocessed,num_imu_fed,t_load_ms,t_preprocess_ms,t_odometry_ms,"
             "inlier_ratio,residual,num_map_factors,num_tiles\n";
  frames_.flush();
}

void ReplayOutputWriter::write(const ReplayFrameRecord& r) {
  char line[512];
  std::snprintf(
    line, sizeof(line), "%" PRId64 ",%s,%d,%s,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.6g,%.6g,%d,%d\n", r.stamp_ns, to_string(r.result), r.frame_id,
    to_string(r.state), r.reset_count, r.num_points_raw, r.num_points_preprocessed, r.num_imu_fed, r.t_load_ms, r.t_preprocess_ms, r.t_odometry_ms,
    r.inlier_ratio, r.residual, r.num_map_factors, r.num_tiles);
  frames_ << line;
  frames_.flush();

  if (r.T_world_imu) {
    const Eigen::Quaterniond q(r.T_world_imu->linear());
    const Eigen::Vector3d& t = r.T_world_imu->translation();
    std::snprintf(
      line, sizeof(line), "%s %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n", format_stamp_ns(r.stamp_ns).c_str(), t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
    traj_ << line;
    traj_.flush();
    num_poses_++;
  }
  if (!traj_ || !frames_) {
    throw std::runtime_error("write failure in " + out_dir_);
  }
}

void ReplayOutputWriter::write_summary(const ReplaySummary& summary, double map_load_seconds) {
  nlohmann::json json;
  json["num_frames"] = summary.num_frames;
  json["frames_by_result"] = summary.frames_by_result;
  json["estimated_by_state"] = summary.estimated_by_state;
  json["num_poses"] = num_poses_;
  json["map_load_seconds"] = map_load_seconds;
  json["wall_seconds"] = summary.wall_seconds;
  json["imu_gap_resets"] = summary.imu_gap_resets;
  if (summary.tracking_ratio) {
    json["tracking_ratio"] = *summary.tracking_ratio;
  }
  if (summary.min_tracking_ratio) {
    json["min_tracking_ratio"] = *summary.min_tracking_ratio;
  }
  if (summary.init_search) {
    const auto& search = *summary.init_search;
    json["init_search"] = {
      {"result", to_string(search.result)},
      {"reason", to_string(search.reason)},
      {"candidates", search.candidates},
      {"candidates_total", search.candidates_total},
      {"truncated", search.candidates < search.candidates_total},
      {"score", search.score},
      {"yaw_offset_deg", search.yaw_offset * 180.0 / M_PI},
      {"runner_up_score", search.runner_up_score},
      {"runner_up_yaw_offset_deg", search.runner_up_yaw_offset * 180.0 / M_PI},
      {"seconds", search.seconds}};
  }

  std::ofstream ofs(out_dir_ + "/summary.json", std::ios::trunc);
  if (!(ofs << json.dump(2) << "\n")) {
    throw std::runtime_error("cannot write " + out_dir_ + "/summary.json");
  }
}

}  // namespace glim_localization
