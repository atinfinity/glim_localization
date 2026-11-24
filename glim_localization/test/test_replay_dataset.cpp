#include <cstring>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <unistd.h>

#include <glim_localization/replay/replay_dataset.hpp>

namespace fs = std::filesystem;
using namespace glim_localization;

namespace {

std::vector<ReplayPoint> make_points(uint32_t count) {
  std::vector<ReplayPoint> points;
  for (uint32_t i = 0; i < count; i++) {
    points.push_back({1.0f + i, -2.0f * i, 0.5f, i == 0 ? 0.0f : 0.001f * i, 10.0f + i});
  }
  return points;
}

std::vector<ReplayImu> make_imu(int count) {
  std::vector<ReplayImu> imu;
  for (int i = 0; i < count; i++) {
    ReplayImu sample;
    sample.stamp_ns = 1700000000123456789LL + i * 10000000LL;
    sample.acc = Eigen::Vector3d(0.1 * i + 1.0 / 3.0, -9.81, 1e-12);
    sample.gyro = Eigen::Vector3d(M_PI, -1e-9 * i, 0.25);
    imu.push_back(sample);
  }
  return imu;
}

constexpr int64_t kStamp = 1700000000123456789LL;

}  // namespace

class ReplayDatasetTest : public testing::Test {
protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("glim_loc_replay_dataset_" + std::to_string(::getpid()) + "_" +
                                         testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(dir_);
  }
  void TearDown() override { fs::remove_all(dir_); }

  void write_valid(uint32_t flags = kReplayFlagPointTime | kReplayFlagIntensity) {
    fs::remove_all(dir_);
    ReplayDatasetWriter writer(dir_.string());
    writer.write_cloud(kStamp, make_points(5), flags);
    writer.write_cloud(kStamp + 100000000, make_points(3), flags);
    writer.write_imu(make_imu(4));
    writer.finish(flags & kReplayFlagPointTime, R"({"topic":"/points"})");
  }

  fs::path first_bin() const { return dir_ / "points" / "1700000000123456789.bin"; }

  void edit_bytes(const fs::path& path, const std::function<void(std::string&)>& edit) const {
    std::string data;
    {
      std::ifstream ifs(path, std::ios::binary);
      data.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
    }
    edit(data);
    std::ofstream(path, std::ios::binary | std::ios::trunc).write(data.data(), data.size());
  }

  void expect_open_fails(const std::string& message_part) const {
    try {
      ReplayDataset::open(dir_.string());
      FAIL() << "open succeeded";
    } catch (const InvalidDatasetError& e) {
      EXPECT_NE(std::string(e.what()).find(message_part), std::string::npos) << e.what();
    }
  }

  fs::path dir_;
};

TEST_F(ReplayDatasetTest, RoundTripPreservesData) {
  write_valid();
  const auto dataset = ReplayDataset::open(dir_.string());

  EXPECT_TRUE(dataset.meta().per_point_time);
  EXPECT_EQ(dataset.meta().num_frames, 2u);
  EXPECT_EQ(dataset.meta().num_imu, 4u);
  EXPECT_EQ(nlohmann::json::parse(dataset.meta().conversion_json).at("topic"), "/points");
  EXPECT_FALSE(dataset.has_ground_truth());

  const auto expected_imu = make_imu(4);
  ASSERT_EQ(dataset.imu().size(), expected_imu.size());
  for (size_t i = 0; i < expected_imu.size(); i++) {
    EXPECT_EQ(dataset.imu()[i].stamp_ns, expected_imu[i].stamp_ns);
    EXPECT_EQ(dataset.imu()[i].acc, expected_imu[i].acc);  // 17 digits are exact
    EXPECT_EQ(dataset.imu()[i].gyro, expected_imu[i].gyro);
  }

  ASSERT_EQ(dataset.frames().size(), 2u);
  EXPECT_EQ(dataset.frames()[0].stamp_ns, kStamp);
  EXPECT_EQ(dataset.frames()[1].stamp_ns, kStamp + 100000000);
  EXPECT_EQ(dataset.frames()[1].num_points, 3u);

  const auto cloud = dataset.load_cloud(0);
  const auto expected_points = make_points(5);
  ASSERT_EQ(cloud->size(), 5);
  EXPECT_DOUBLE_EQ(cloud->stamp, kStamp * 1e-9);
  for (size_t i = 0; i < expected_points.size(); i++) {
    EXPECT_FLOAT_EQ(cloud->points[i].x(), expected_points[i].x);
    EXPECT_FLOAT_EQ(cloud->points[i].y(), expected_points[i].y);
    EXPECT_FLOAT_EQ(cloud->points[i].z(), expected_points[i].z);
    EXPECT_DOUBLE_EQ(cloud->points[i].w(), 1.0);
    EXPECT_FLOAT_EQ(cloud->times[i], expected_points[i].t);
    EXPECT_FLOAT_EQ(cloud->intensities[i], expected_points[i].intensity);
  }
}

TEST_F(ReplayDatasetTest, MissingFlagsReadAsZero) {
  write_valid(0);
  const auto dataset = ReplayDataset::open(dir_.string());
  EXPECT_FALSE(dataset.meta().per_point_time);
  const auto cloud = dataset.load_cloud(0);
  for (int i = 0; i < cloud->size(); i++) {
    EXPECT_EQ(cloud->times[i], 0.0);
    EXPECT_EQ(cloud->intensities[i], 0.0);
  }
}

TEST_F(ReplayDatasetTest, FormatsStamps) {
  EXPECT_EQ(format_stamp_ns(1700000000123456789LL), "1700000000.123456789");
  EXPECT_EQ(format_stamp_ns(2000000000LL), "2.000000000");
  EXPECT_EQ(format_stamp_ns(5), "0.000000005");
  EXPECT_EQ(format_stamp_ns(0), "0.000000000");
}

TEST_F(ReplayDatasetTest, RejectsMissingRequiredFiles) {
  write_valid();
  fs::remove(dir_ / "meta.json");
  expect_open_fails("meta.json");
  write_valid();
  fs::remove(dir_ / "imu.csv");
  expect_open_fails("imu.csv");
  EXPECT_THROW(ReplayDataset::open((dir_ / "nothing").string()), InvalidDatasetError);
}

TEST_F(ReplayDatasetTest, RejectsBadMeta) {
  const auto rewrite = [&](const std::function<void(nlohmann::json&)>& edit) {
    write_valid();
    nlohmann::json meta = nlohmann::json::parse(std::ifstream(dir_ / "meta.json"));
    edit(meta);
    std::ofstream(dir_ / "meta.json", std::ios::trunc) << meta.dump();
  };
  rewrite([](nlohmann::json& m) { m["format"] = "other"; });
  expect_open_fails("format");
  rewrite([](nlohmann::json& m) { m["version"] = 2; });
  expect_open_fails("version");
  rewrite([](nlohmann::json& m) { m["version"] = 1.9; });
  expect_open_fails("version");
  rewrite([](nlohmann::json& m) { m["version"] = "1"; });
  expect_open_fails("meta.json");
  rewrite([](nlohmann::json& m) { m["stamp_definition"] = "last_point"; });
  expect_open_fails("stamp_definition");
  rewrite([](nlohmann::json& m) { m["num_frames"] = 3; });
  expect_open_fails("num_frames");
  rewrite([](nlohmann::json& m) { m["num_imu"] = 9; });
  expect_open_fails("num_imu");
  rewrite([](nlohmann::json& m) { m.erase("per_point_time"); });
  expect_open_fails("meta.json");
  write_valid();
  std::ofstream(dir_ / "meta.json", std::ios::trunc) << "{not json";
  expect_open_fails("invalid JSON");
}

TEST_F(ReplayDatasetTest, RejectsBadCloudHeaders) {
  const auto corrupt = [&](size_t offset, uint32_t value, const std::string& message) {
    write_valid();
    edit_bytes(first_bin(), [&](std::string& d) { std::memcpy(&d[offset], &value, 4); });
    expect_open_fails(message);
  };
  corrupt(0, 0x58585858u, "bad magic");
  corrupt(4, 2, "version");
  corrupt(16, 0, "num_points is 0");
  corrupt(16, 6, "file size");
  corrupt(20, 4, "flags");
  corrupt(24, 16, "point_step");
  corrupt(28, 1, "reserved");

  write_valid();
  edit_bytes(first_bin(), [](std::string& d) { d.resize(d.size() - 4); });
  expect_open_fails("file size");

  write_valid();
  edit_bytes(first_bin(), [](std::string& d) { d.resize(10); });
  expect_open_fails("truncated header");

  write_valid();
  edit_bytes(first_bin(), [](std::string& d) {
    const int64_t other = kStamp + 1;
    std::memcpy(&d[8], &other, 8);
  });
  expect_open_fails("does not match the file name");

  write_valid();
  fs::rename(first_bin(), dir_ / "points" / "1.bin");
  expect_open_fails("19 digits");

  write_valid();
  edit_bytes(first_bin(), [](std::string& d) {
    const int64_t negative = -8;
    std::memcpy(&d[8], &negative, 8);
  });
  fs::rename(first_bin(), dir_ / "points" / "-000000000000000008.bin");
  expect_open_fails("negative stamps");

  write_valid();
  std::ofstream(dir_ / "points" / "cloud.ply") << "ply";
  expect_open_fails("PLY");
}

TEST_F(ReplayDatasetTest, RejectsBadImu) {
  const auto rewrite_imu = [&](const std::string& body, size_t count = 0) {
    write_valid();
    std::ofstream(dir_ / "imu.csv", std::ios::trunc) << "stamp_ns,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\n" << body;
    nlohmann::json meta = nlohmann::json::parse(std::ifstream(dir_ / "meta.json"));
    meta["num_imu"] = count;
    std::ofstream(dir_ / "meta.json", std::ios::trunc) << meta.dump();
  };
  rewrite_imu("10,0,0,0,0,0,0\n10,0,0,0,0,0,0\n", 2);
  expect_open_fails("imu.csv:3: stamp_ns is not strictly increasing");
  rewrite_imu("10,0,0,0,0,0,0\n5,0,0,0,0,0,0\n", 2);
  expect_open_fails("imu.csv:3");
  rewrite_imu("10,0,0,0,0,0\n", 1);
  expect_open_fails("imu.csv:2: expected 7 columns");
  rewrite_imu("10,0,0,0,0,0,0,0\n", 1);
  expect_open_fails("expected 7 columns");
  rewrite_imu("10,0,nan,0,0,0,0\n", 1);
  expect_open_fails("imu.csv:2: invalid or non-finite");
  rewrite_imu("10,0,inf,0,0,0,0\n", 1);
  expect_open_fails("non-finite");
  rewrite_imu("10,0,abc,0,0,0,0\n", 1);
  expect_open_fails("imu.csv:2");
  rewrite_imu("10, 0,0,0,0,0,0\n", 1);
  expect_open_fails("imu.csv:2");
  rewrite_imu("10,0x1p3,0,0,0,0,0\n", 1);
  expect_open_fails("imu.csv:2");
  rewrite_imu("10,0,0,0,0,0,0 \n", 1);
  expect_open_fails("imu.csv:2");
  rewrite_imu("1.5,0,0,0,0,0,0\n", 1);
  expect_open_fails("invalid stamp_ns");
  rewrite_imu("10,0,0,0,0,0,0\n\n20,0,0,0,0,0,0\n", 2);
  expect_open_fails("imu.csv:3");

  write_valid();
  std::ofstream(dir_ / "imu.csv", std::ios::trunc) << "time,ax\n";
  expect_open_fails("header");
}

TEST_F(ReplayDatasetTest, RejectsBadPointBodies) {
  const auto set_float = [&](size_t point, size_t field, float value) {
    write_valid();
    edit_bytes(first_bin(), [&](std::string& d) { std::memcpy(&d[kReplayCloudHeaderSize + point * kReplayPointStep + field * 4], &value, 4); });
    const auto dataset = ReplayDataset::open(dir_.string());  // The header is fine
    try {
      dataset.load_cloud(0);
      ADD_FAILURE() << "load_cloud succeeded";
    } catch (const InvalidDatasetError& e) {
      EXPECT_NE(std::string(e.what()).find(".bin"), std::string::npos) << e.what();
    }
  };
  const float nan = std::numeric_limits<float>::quiet_NaN();
  set_float(2, 0, nan);
  set_float(2, 2, std::numeric_limits<float>::infinity());
  set_float(2, 3, nan);
  set_float(2, 4, nan);
  set_float(2, 3, 1.0f);
  set_float(2, 3, -0.1f);
  set_float(0, 3, 0.01f);  // The minimum time is no longer 0
}
