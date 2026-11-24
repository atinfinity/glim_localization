#include <glim_localization/odometry/map_initial_state_estimation.hpp>

namespace glim_localization {

MapInitialStateEstimation::MapInitialStateEstimation(const Eigen::Isometry3d& T_lidar_imu, const Eigen::Matrix<double, 6, 1>& imu_bias)
: T_lidar_imu_(T_lidar_imu), imu_bias_(imu_bias) {}

void MapInitialStateEstimation::set_state(const Eigen::Isometry3d& T_world_imu, const Eigen::Vector3d& v_world_imu) {
  T_world_imu_ = T_world_imu;
  v_world_imu_ = v_world_imu;
  ready_ = true;
}

void MapInitialStateEstimation::insert_frame(const glim::PreprocessedFrame::ConstPtr& raw_frame) {
  stamp_ = raw_frame->stamp;
}

glim::EstimationFrame::ConstPtr MapInitialStateEstimation::initial_pose() {
  if (!ready_) {
    return nullptr;
  }

  glim::EstimationFrame::Ptr estimated(new glim::EstimationFrame);
  estimated->id = -1;
  estimated->stamp = stamp_;
  estimated->T_lidar_imu = T_lidar_imu_;
  estimated->T_world_imu = T_world_imu_;
  estimated->T_world_lidar = T_world_imu_ * T_lidar_imu_.inverse();
  estimated->v_world_imu = v_world_imu_;
  estimated->imu_bias = imu_bias_;
  estimated->frame_id = glim::FrameID::IMU;
  return estimated;
}

}  // namespace glim_localization
