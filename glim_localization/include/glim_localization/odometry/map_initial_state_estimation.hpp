#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/odometry/initial_state_estimation.hpp>

namespace glim_localization {

/// Initial state given in the map frame. initial_pose() is nullptr until a state is set.
class MapInitialStateEstimation : public glim::InitialStateEstimation {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MapInitialStateEstimation(const Eigen::Isometry3d& T_lidar_imu, const Eigen::Matrix<double, 6, 1>& imu_bias);

  void set_state(const Eigen::Isometry3d& T_world_imu, const Eigen::Vector3d& v_world_imu);

  void insert_frame(const glim::PreprocessedFrame::ConstPtr& raw_frame) override;
  glim::EstimationFrame::ConstPtr initial_pose() override;

private:
  Eigen::Isometry3d T_lidar_imu_;
  Eigen::Matrix<double, 6, 1> imu_bias_;
  bool ready_ = false;
  double stamp_ = 0.0;
  Eigen::Isometry3d T_world_imu_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d v_world_imu_ = Eigen::Vector3d::Zero();
};

}  // namespace glim_localization
