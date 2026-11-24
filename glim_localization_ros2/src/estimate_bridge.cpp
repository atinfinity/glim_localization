#include <glim_localization_ros2/estimate_bridge.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <spdlog/spdlog.h>

#include <glim/odometry/callbacks.hpp>

namespace glim_localization_ros2
{

Eigen::Isometry3d parse_pose7(const std::vector<double> & v, const std::string & name)
{
  if (v.size() != 7) {
    throw std::invalid_argument(name + " must have 7 values [x, y, z, qx, qy, qz, qw]");
  }
  const Eigen::Quaterniond q(v[6], v[3], v[4], v[5]);
  if (!std::isfinite(q.norm()) || q.norm() < 1e-6 || !std::all_of(v.begin(), v.begin() + 3,
    [](double x) {return std::isfinite(x);}))
  {
    throw std::invalid_argument(name + " is not a valid pose");
  }
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = q.normalized().toRotationMatrix();
  T.translation() = Eigen::Vector3d(v[0], v[1], v[2]);
  return T;
}

LocalizationPublisherParams read_publisher_params(const glim::Config & config)
{
  LocalizationPublisherParams params;
  params.map_frame = config.param<std::string>("glim_ros", "map_frame_id", params.map_frame);
  params.odom_frame = config.param<std::string>("glim_ros", "odom_frame_id", params.odom_frame);
  const std::string section = "localization_ros";
  params.base_frame = config.param<std::string>(section, "base_frame_id", params.base_frame);
  params.tf_mode = parse_tf_mode(config.param<std::string>(section, "tf_mode",
      to_string(params.tf_mode)));
  params.diagnostics_period = config.param<double>(section, "diagnostics_period",
      params.diagnostics_period);
  params.T_base_imu = parse_pose7(config.param<std::vector<double>>(section, "T_base_imu",
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}), "T_base_imu");
  return params;
}

void EstimateBridge::connect(const std::shared_ptr<EstimateBridge> & bridge)
{
  // The IMU, frame and status callbacks all run on the thread of AsyncOdometryEstimation (insert_imu, insert_frame and
  // the reset applied there); the mutex also covers attach, detach and the pose conversion, which run on other threads.
  glim::OdometryEstimationCallbacks::on_insert_imu.add(
    [bridge](const double stamp, const Eigen::Vector3d &, const Eigen::Vector3d & angular_vel) {
      bridge->add_imu(stamp, angular_vel);
                                                                                                                                    });
  glim::OdometryEstimationCallbacks::on_update_new_frame.add(
    [bridge](const glim::EstimationFrame::ConstPtr & frame) {
      if (frame) {
        bridge->add_frame(*frame);
      }
  });
  glim_localization::LocalizationCallbacks::on_status.add(
    [bridge](const glim_localization::LocalizationStatus & status) {bridge->add_status(status);});
}

void EstimateBridge::attach(
  std::unique_ptr<LocalizationPublisher> publisher,
  const LocalizationPublisherParams & params)
{
  std::lock_guard<std::mutex> lock(mutex_);
  publisher_ = std::move(publisher);
  map_frame_ = params.map_frame;
  T_base_imu_ = params.T_base_imu;
  gyro_ = GyroBuffer();
  frames_without_gyro_ = 0;
}

void EstimateBridge::detach()
{
  std::unique_ptr<LocalizationPublisher> publisher;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publisher = std::move(publisher_);
  }
  // Destroyed outside the lock: no callback can reach it any more
}

bool EstimateBridge::attached() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return publisher_ != nullptr;
}

void EstimateBridge::add_imu(double stamp, const Eigen::Vector3d & angular_vel)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (publisher_ && std::isfinite(stamp)) {
    gyro_.add(stamp, angular_vel);
  }
}

void EstimateBridge::add_frame(const glim::EstimationFrame & frame)
{
  FrameEstimate estimate;
  estimate.stamp = frame.stamp;
  estimate.T_map_imu = frame.T_world_imu;
  estimate.v_map_imu = frame.v_world_imu;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!publisher_) {
    return;
  }
  Eigen::Vector3d gyro;
  if (!gyro_.nearest(frame.stamp, gyro)) {
    // Throttled: the first rejected frame, then every 100th
    if (frames_without_gyro_++ % 100 == 0) {
      spdlog::warn(
        "[glim_localization] a frame has no IMU sample within {} s of its stamp: it is not published ({} so far)",
          GyroBuffer::MAX_GAP,
        frames_without_gyro_);
    }
    return;
  }
  estimate.w_imu = gyro - frame.imu_bias.tail<3>();  // bias-corrected, in the IMU frame
  publisher_->on_frame(estimate);
}

void EstimateBridge::add_status(const glim_localization::LocalizationStatus & status)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!publisher_) {
    return;
  }
  publisher_->on_status(status);
}

size_t EstimateBridge::frames_without_gyro() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return frames_without_gyro_;
}

EstimateBridge::DropCounts EstimateBridge::drop_counts() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  DropCounts counts;
  if (publisher_) {
    counts.frames_without_gyro = frames_without_gyro_;
    counts.unpaired_frames = publisher_->unpaired_frames();
    counts.stale_frames = publisher_->stale_frames();
    counts.tf_lookup_failures = publisher_->tf_lookup_failures();
  }
  return counts;
}

std::string EstimateBridge::to_T_map_imu(
  const PoseMsg & msg, Eigen::Isometry3d & T_map_imu,
  double & min_stamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!publisher_) {
    return "the localization is not active";
  }
  if (!msg.header.frame_id.empty() && msg.header.frame_id != map_frame_) {
    return "the pose is in frame '" + msg.header.frame_id + "': expected '" + map_frame_ + "'";
  }
  const auto & p = msg.pose.pose;
  const Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  const Eigen::Vector3d t(p.position.x, p.position.y, p.position.z);
  if (!t.allFinite() || !q.coeffs().allFinite() || q.norm() < 1e-6) {
    return "the pose has a non-finite position or a zero quaternion";
  }
  Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();
  T_map_base.linear() = q.normalized().toRotationMatrix();
  T_map_base.translation() = t;
  T_map_imu = T_map_base * T_base_imu_;
  min_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;  // 0 for an unstamped pose: applies at the next frame
  return "";
}

}  // namespace glim_localization_ros2
