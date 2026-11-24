#include <dlfcn.h>

#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <glim/odometry/odometry_estimation_base.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

namespace {

bool check_module(const char* so_name) {
  void* handle = dlopen(so_name, RTLD_NOW);
  if (handle == nullptr) {
    std::cerr << "dlopen failed: " << dlerror() << std::endl;
    return false;
  }
  if (dlsym(handle, "create_odometry_estimation_module") == nullptr) {
    std::cerr << "dlsym failed: " << dlerror() << std::endl;
    return false;
  }
  return true;
}

}  // namespace

int main() {
  const std::vector<Eigen::Vector4d> points = {{0.0, 0.0, 0.0, 1.0}, {1.0, 0.0, 0.0, 1.0}, {0.0, 1.0, 0.0, 1.0}};
  const auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
  if (cloud->size() != points.size()) {
    std::cerr << "unexpected point cloud size: " << cloud->size() << std::endl;
    return 1;
  }

  // Keep a real reference to libglim so that the link is not dropped by --as-needed.
  volatile auto load_module = &glim::OdometryEstimationBase::load_module;
  if (load_module == nullptr) {
    std::cerr << "glim::OdometryEstimationBase::load_module is unavailable" << std::endl;
    return 1;
  }

  if (!check_module("libodometry_estimation_cpu.so")) {
    return 1;
  }
#ifdef CHECK_DEPS_WITH_CUDA
  // Loading needs no GPU: libcudart opens the driver only on the first CUDA call.
  if (!check_module("libodometry_estimation_gpu.so")) {
    return 1;
  }
  std::cout << "gtsam_points, glim, and the CPU and GPU odometry modules are usable" << std::endl;
#else
  std::cout << "gtsam_points, glim, and the CPU odometry module are usable" << std::endl;
#endif
  return 0;
}
