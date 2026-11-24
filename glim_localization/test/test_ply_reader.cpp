#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

#include <glim_localization/map/pcd_map.hpp>
#include <glim_localization/map/ply_reader.hpp>

#include "synthetic/synthetic_map.hpp"

namespace fs = std::filesystem;
using namespace glim_localization;
using namespace glim_localization::test;

class PlyReaderTest : public testing::Test {
protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("glim_loc_ply_reader_" + std::to_string(::getpid()));
    fs::remove_all(dir_);
    fs::create_directories(dir_);
    params_.num_submaps = 2;
    params_.point_spacing = 0.5;
    params_.world_offset = Eigen::Vector3d(12345.678901, -9876.54321, 321.123456);
    points_ = make_synthetic_world_points(params_);
  }
  void TearDown() override { fs::remove_all(dir_); }

  fs::path write_bytes(const std::string& name, const std::string& bytes) const {
    const fs::path path = dir_ / name;
    std::ofstream(path, std::ios::binary) << bytes;
    return path;
  }

  /// A failed read must return false, keep the caller's data, and log a message containing `message`.
  static void expect_read_fails(const fs::path& path, const std::string& message) {
    std::ostringstream log;
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(log);
    spdlog::default_logger()->sinks().push_back(sink);
    PcdPoints points(1, Eigen::Vector4d(7.0, 7.0, 7.0, 1.0));
    EXPECT_FALSE(read_ply(path.string(), points)) << path;
    spdlog::default_logger()->sinks().pop_back();
    ASSERT_EQ(points.size(), 1u);
    EXPECT_EQ(points[0].x(), 7.0);
    EXPECT_NE(log.str().find(message), std::string::npos) << "expected \"" << message << "\" in: " << log.str();
  }

  template <typename T>
  static std::string bytes_of(const T value) {
    return std::string(reinterpret_cast<const char*>(&value), sizeof(value));
  }

  fs::path dir_;
  SyntheticMapParams params_;
  SyntheticPoints points_;
};

TEST_F(PlyReaderTest, ReadsAsciiAndBinaryWithExtraProperties) {
  for (const bool lists : {false, true}) {
    for (const bool double_xyz : {false, true}) {
      for (const auto encoding : {PlyEncoding::Ascii, PlyEncoding::BinaryLittleEndian}) {
        SCOPED_TRACE(testing::Message() << "lists " << lists << " double " << double_xyz << " ascii " << (encoding == PlyEncoding::Ascii));
        const fs::path path = dir_ / "cloud.ply";
        write_synthetic_ply(path.string(), points_, encoding, PlyOptions{double_xyz, lists});
        PcdPoints read;
        ASSERT_TRUE(read_ply(path.string(), read));
        ASSERT_EQ(read.size(), points_.size());
        for (size_t i = 0; i < read.size(); i++) {
          for (int axis = 0; axis < 3; axis++) {
            // Ascii is printed with 17 digits; binary float rounds to float.
            const double expected = (encoding == PlyEncoding::BinaryLittleEndian && !double_xyz) ? static_cast<float>(points_[i][axis]) : points_[i][axis];
            ASSERT_EQ(read[i][axis], expected) << i;
          }
          ASSERT_EQ(read[i].w(), 1.0);
        }
      }
    }
  }
}

TEST_F(PlyReaderTest, MatchesPcd) {
  for (const auto& [ply_encoding, pcd_encoding] : {std::make_pair(PlyEncoding::Ascii, PcdEncoding::Ascii),
                                                    std::make_pair(PlyEncoding::BinaryLittleEndian, PcdEncoding::Binary)}) {
    write_synthetic_ply((dir_ / "cloud.ply").string(), points_, ply_encoding, PlyOptions{true, false});
    write_synthetic_pcd((dir_ / "cloud.pcd").string(), points_, pcd_encoding, true);
    PcdPoints from_ply, from_pcd;
    ASSERT_TRUE(read_ply((dir_ / "cloud.ply").string(), from_ply));
    ASSERT_TRUE(read_pcd((dir_ / "cloud.pcd").string(), from_pcd));
    ASSERT_EQ(from_ply.size(), from_pcd.size());
    for (size_t i = 0; i < from_ply.size(); i++) {
      ASSERT_EQ(from_ply[i], from_pcd[i]) << i;
    }
  }
}

TEST_F(PlyReaderTest, ReadsCrlfHeaderSizedTypesAndDropsNonFinitePoints) {
  const std::string header =
    "ply\r\nformat ascii 1.0\r\ncomment x y z\r\nelement vertex 4\r\nproperty float64 z\r\nproperty float32 y\r\nproperty double x\r\n"
    "property int16 label\r\nend_header\r\n";
  const fs::path path = write_bytes("crlf.ply", header + "3 2 1 -5\r\n\r\n6 5 4 0\r\nnan 0 0 1\r\n9 8 inf 2\r\n");
  PcdPoints points;
  ASSERT_TRUE(read_ply(path.string(), points));
  ASSERT_EQ(points.size(), 2u);
  EXPECT_EQ(points[0], Eigen::Vector4d(1.0, 2.0, 3.0, 1.0));
  EXPECT_EQ(points[1], Eigen::Vector4d(4.0, 5.0, 6.0, 1.0));
}

TEST_F(PlyReaderTest, ReadsBinaryWithACrlfHeader) {
  // The data starts right after "end_header\r\n"; the \r must not be taken as the first data byte.
  const std::string header =
    "ply\r\nformat binary_little_endian 1.0\r\nelement vertex 2\r\nproperty uchar label\r\nproperty float x\r\nproperty float y\r\n"
    "property double z\r\nend_header\r\n";
  const std::string data = bytes_of(uint8_t(13)) + bytes_of(1.5f) + bytes_of(-2.0f) + bytes_of(3.25) +  //
                           bytes_of(uint8_t(10)) + bytes_of(4.0f) + bytes_of(5.0f) + bytes_of(-6.5);
  const fs::path path = write_bytes("crlf_binary.ply", header + data);
  PcdPoints points;
  ASSERT_TRUE(read_ply(path.string(), points));
  ASSERT_EQ(points.size(), 2u);
  EXPECT_EQ(points[0], Eigen::Vector4d(1.5, -2.0, 3.25, 1.0));
  EXPECT_EQ(points[1], Eigen::Vector4d(4.0, 5.0, -6.5, 1.0));
}

TEST_F(PlyReaderTest, ReadsAnEmptyVertexElement) {
  const fs::path path = write_bytes("empty.ply", "ply\nformat binary_little_endian 1.0\nelement vertex 0\nproperty float x\nproperty float y\nproperty float z\nend_header\n");
  PcdPoints points(1);
  ASSERT_TRUE(read_ply(path.string(), points));
  EXPECT_TRUE(points.empty());
}

TEST_F(PlyReaderTest, RejectsUnsupportedOrMalformedHeaders) {
  const std::string xyz = "property float x\nproperty float y\nproperty float z\n";
  const std::vector<std::pair<std::string, std::string>> cases = {
    {"PLY\nformat ascii 1.0\n", "not a PLY file"},
    {"ply\nformat binary_big_endian 1.0\nelement vertex 1\n" + xyz + "end_header\n", "big-endian PLY is not supported"},
    {"ply\nformat binary_middle_endian 1.0\nelement vertex 1\n" + xyz + "end_header\n", "unknown PLY format"},
    {"ply\nformat ascii 2.0\n", "unsupported PLY version 2.0"},
    {"ply\nelement vertex 1\n" + xyz + "end_header\n", "no format line"},
    {"ply\nformat ascii 1.0\nformat ascii 1.0\n", "invalid format line"},
    {"ply\nformat ascii 1.0\nelement vertex 1\n" + xyz, "no end_header"},
    {"ply\nformat ascii 1.0\nproperty float x\n", "property before any element"},
    {"ply\nformat ascii 1.0\nelement vertex -1\n" + xyz + "end_header\n", "invalid element line"},
    {"ply\nformat ascii 1.0\nelement vertex 1\n" + xyz + "element vertex 1\n", "element vertex appears more than once"},
    {"ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\nend_header\n0 0\n", "missing vertex property x, y or z"},
    {"ply\nformat ascii 1.0\nelement point 1\n" + xyz + "end_header\n0 0 0\n", "no vertex element"},
    {"ply\nformat ascii 1.0\nelement vertex 1\nproperty int x\nproperty float y\nproperty float z\nend_header\n0 0 0\n", "must be a float or double"},
    {"ply\nformat ascii 1.0\nelement vertex 1\nproperty list uchar float x\nproperty float y\nproperty float z\nend_header\n", "must be a float or double"},
    {"ply\nformat ascii 1.0\nelement vertex 1\n" + xyz + "property float x\nend_header\n", "property x appears more than once"},
    {"ply\nformat ascii 1.0\nelement vertex 1\nproperty half x\n", "invalid property line"},
    {"ply\nformat ascii 1.0\nelement vertex 1\nproperty list float int idx\n", "invalid property line"},
    {"ply\nformat ascii 1.0\nelement vertex 1\nproperty float\n", "invalid property line"},
    {"ply\nformat ascii 1.0\nelement blank 1\nelement vertex 1\n" + xyz + "end_header\n", "element blank in"},
    {"ply\nformat ascii 1.0\nvertex 1\n", "unknown PLY header line"},
    // Headers are capped at 64 KiB: a long comment, and a file with no newline at all.
    {"ply\nformat ascii 1.0\ncomment " + std::string(70000, 'c') + "\nelement vertex 1\n" + xyz + "end_header\n0 0 0\n", "longer than 65536 bytes"},
    {std::string(70000, 'p'), "longer than 65536 bytes"},
  };
  for (size_t i = 0; i < cases.size(); i++) {
    SCOPED_TRACE(cases[i].first);
    expect_read_fails(write_bytes("bad" + std::to_string(i) + ".ply", cases[i].first), cases[i].second);
  }
  expect_read_fails(dir_ / "missing.ply", "failed to open");
}

TEST_F(PlyReaderTest, RejectsTruncatedOrBrokenData) {
  const std::string xyz = "property float x\nproperty float y\nproperty float z\n";
  const std::string binary = "ply\nformat binary_little_endian 1.0\n";
  const std::string ascii = "ply\nformat ascii 1.0\n";
  std::string three_floats = bytes_of(1.0f) + bytes_of(2.0f) + bytes_of(3.0f);
  const std::vector<std::pair<std::string, std::string>> cases = {
    // Fixed-size binary: the count is checked against the file size before allocating.
    {binary + "element vertex 2\n" + xyz + "end_header\n" + three_floats + "abc", "truncated binary data"},
    {binary + "element vertex 1000000000000\n" + xyz + "end_header\n" + three_floats, "truncated binary data"},
    // Variable-size binary: the second vertex says 200 list items, but the file ends.
    {binary + "element vertex 2\n" + xyz + "property list uchar int l\nend_header\n" + three_floats + bytes_of(uint8_t(0)) + three_floats +
       bytes_of(uint8_t(200)) + bytes_of(int32_t(1)),
     "truncated or invalid binary data at vertex 1 of 2"},
    // A negative list length
    {binary + "element vertex 2\n" + xyz + "property list char int l\nend_header\n" + three_floats + bytes_of(int8_t(-1)) + three_floats +
       bytes_of(int8_t(0)),
     "truncated or invalid binary data at vertex 0 of 2"},
    // An element before the vertices ends early.
    {binary + "element head 3\nproperty int a\nelement vertex 1\n" + xyz + "end_header\n" + bytes_of(int32_t(1)), "truncated or invalid binary data in element head"},
    {ascii + "element vertex 3\n" + xyz + "end_header\n1 2 3\n4 5 6\n", "truncated ascii data"},
    {ascii + "element vertex 1000000\n" + xyz + "end_header\n1 2 3\n", "truncated ascii data"},
    {ascii + "element head 3\nproperty int a\nelement vertex 1\n" + xyz + "end_header\n1\n2\n", "truncated ascii data in element head"},
    {ascii + "element vertex 1\n" + xyz + "end_header\n1 2 3 4\n", "has 4 values, expected 3"},
    {ascii + "element vertex 1\n" + xyz + "end_header\n1          2\n", "too few values"},  // Padded past the size check
    {ascii + "element vertex 1\n" + xyz + "end_header\n1 2 abc\n", "invalid value abc"},
    // Values other than x, y, z are checked too.
    {ascii + "element vertex 1\n" + xyz + "property uchar red\nend_header\n1 2 3 red\n", "invalid value red"},
    {ascii + "element vertex 1\n" + xyz + "property list uchar int l\nend_header\n1 2 3 4 0\n", "invalid list length 4"},
    {ascii + "element vertex 1\n" + xyz + "property list uchar int l\nend_header\n1 2 3 -1\n", "invalid list length -1"},
  };
  for (size_t i = 0; i < cases.size(); i++) {
    SCOPED_TRACE(i);
    expect_read_fails(write_bytes("broken" + std::to_string(i) + ".ply", cases[i].first), cases[i].second);
  }
}
