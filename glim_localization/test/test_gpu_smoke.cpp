// GPU smoke test, built only with BUILD_WITH_CUDA=ON. It checks that the CUDA build of gtsam_points
// works on the visible GPU: a point cloud is uploaded and a GaussianVoxelMapGPU is built from it.
// Without a GPU the tests skip (ctest reports them as skipped), unless GLIM_LOC_REQUIRE_GPU=1.
#include <cstdlib>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <cuda_runtime_api.h>
#include <gtest/gtest.h>
#include <gtsam_points/cuda/cuda_device_prop.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>

namespace {

// GLIM_LOC_REQUIRE_GPU=1 (set by scripts/ci.sh when nvidia-smi lists a GPU) turns a skip into a failure.
bool require_gpu() {
  const char* value = std::getenv("GLIM_LOC_REQUIRE_GPU");
  return value != nullptr && std::string(value) == "1";
}

// Number of visible CUDA devices, 0 when the driver or the device is missing.
int visible_devices(std::string* reason) {
  int count = 0;
  const cudaError_t error = cudaGetDeviceCount(&count);
  if (error != cudaSuccess) {
    *reason = std::string("cudaGetDeviceCount: ") + cudaGetErrorString(error);
    return 0;
  }
  if (count == 0) {
    *reason = "no CUDA device";
  }
  return count;
}

#define SKIP_WITHOUT_GPU()                                                    \
  do {                                                                        \
    std::string reason;                                                       \
    if (visible_devices(&reason) == 0) {                                      \
      if (require_gpu()) {                                                    \
        FAIL() << "GLIM_LOC_REQUIRE_GPU=1 but no GPU is usable: " << reason;  \
      }                                                                       \
      GTEST_SKIP() << "no GPU: " << reason;                                   \
    }                                                                         \
  } while (false)

// Points on a 10 x 10 x 1 m slab with small isotropic covariances.
std::shared_ptr<gtsam_points::PointCloudCPU> make_cloud() {
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> xy(0.0, 10.0);
  std::uniform_real_distribution<double> z(0.0, 1.0);
  std::vector<Eigen::Vector4d> points(20000);
  for (auto& p : points) {
    p = Eigen::Vector4d(xy(rng), xy(rng), z(rng), 1.0);
  }
  Eigen::Matrix4d cov = Eigen::Matrix4d::Zero();
  cov.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 1e-3;
  const std::vector<Eigen::Matrix4d> covs(points.size(), cov);
  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
  cloud->add_covs(covs);
  return cloud;
}

}  // namespace

TEST(GpuSmoke, ListsDevices) {
  SKIP_WITHOUT_GPU();
  const auto names = gtsam_points::cuda_device_names();
  ASSERT_FALSE(names.empty());
  for (const auto& name : names) {
    std::cout << "CUDA device: " << name << std::endl;
  }
}

TEST(GpuSmoke, BuildsGaussianVoxelMapGPU) {
  SKIP_WITHOUT_GPU();
  const auto cloud = make_cloud();
  const auto cloud_gpu = gtsam_points::PointCloudGPU::clone(*cloud);
  ASSERT_TRUE(cloud_gpu);
  ASSERT_NE(cloud_gpu->points_gpu, nullptr);
  ASSERT_NE(cloud_gpu->covs_gpu, nullptr);

  const double resolution = 1.0;
  gtsam_points::GaussianVoxelMapGPU voxelmap_gpu(resolution);
  voxelmap_gpu.insert(*cloud_gpu);
  ASSERT_TRUE(voxelmap_gpu.loaded_on_gpu());

  // Every point lands in one voxel, so the per-voxel counts add up to the number of points.
  const auto num_points = gtsam_points::download_voxel_num_points(voxelmap_gpu);
  const auto means = gtsam_points::download_voxel_means(voxelmap_gpu);
  ASSERT_EQ(num_points.size(), means.size());
  const long total = std::accumulate(num_points.begin(), num_points.end(), 0L);
  EXPECT_EQ(total, static_cast<long>(cloud->size()));

  // Points in [0, 10) x [0, 10) x [0, 1) with 1 m voxels occupy at most 10 x 10 x 1 voxels; the GPU map
  // must find the same occupied voxels as the CPU one.
  gtsam_points::GaussianVoxelMapCPU voxelmap_cpu(resolution);
  voxelmap_cpu.insert(*cloud);
  long occupied = 0;
  for (size_t i = 0; i < num_points.size(); i++) {
    if (num_points[i] == 0) {
      continue;
    }
    occupied++;
    const Eigen::Vector3f& mean = means[i];
    EXPECT_GE(mean.x(), -1e-3f);
    EXPECT_LE(mean.x(), 10.0f + 1e-3f);
    EXPECT_GE(mean.y(), -1e-3f);
    EXPECT_LE(mean.y(), 10.0f + 1e-3f);
    EXPECT_GE(mean.z(), -1e-3f);
    EXPECT_LE(mean.z(), 1.0f + 1e-3f);
  }
  EXPECT_EQ(occupied, static_cast<long>(voxelmap_cpu.num_voxels()));
}
