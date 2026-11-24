#include <glim_localization/replay/replay_dataset.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <sstream>

#include <nlohmann/json.hpp>

static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "the replay format is little-endian and read with memcpy");

namespace glim_localization {
namespace {

namespace fs = std::filesystem;

constexpr char kMagic[4] = {'G', 'L', 'R', 'P'};
constexpr const char* kImuHeader = "stamp_ns,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z";
constexpr size_t kStampDigits = 19;

[[noreturn]] void fail(const fs::path& path, const std::string& message) {
  throw InvalidDatasetError(path.string() + ": " + message);
}

[[noreturn]] void fail_line(const fs::path& path, size_t line, const std::string& message) {
  throw InvalidDatasetError(path.string() + ":" + std::to_string(line) + ": " + message);
}

std::string stamp_file_name(int64_t stamp_ns) {
  char name[40];
  std::snprintf(name, sizeof(name), "%019" PRId64 ".bin", stamp_ns);
  return name;
}

template <typename T>
T read_value(const char* data, size_t offset) {
  T value;
  std::memcpy(&value, data + offset, sizeof(T));
  return value;
}

template <typename T>
void write_value(std::string& out, T value) {
  out.append(reinterpret_cast<const char*>(&value), sizeof(T));
}

bool parse_int64(const std::string& text, int64_t& value) {
  if (text.empty() || text.size() > 20) {
    return false;
  }
  size_t i = text[0] == '-' ? 1 : 0;
  if (i == text.size()) {
    return false;
  }
  for (size_t j = i; j < text.size(); j++) {
    if (text[j] < '0' || text[j] > '9') {
      return false;
    }
  }
  errno = 0;
  char* end = nullptr;
  const long long parsed = std::strtoll(text.c_str(), &end, 10);
  if (errno != 0 || end != text.c_str() + text.size()) {
    return false;
  }
  value = parsed;
  return true;
}

bool parse_finite_double(const std::string& text, double& value) {
  if (text.empty()) {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  // Plain decimal notation only: no whitespace, hex floats, inf or nan
  if (text.find_first_not_of("0123456789+-.eE") != std::string::npos) {
    return false;
  }
  const double parsed = std::strtod(text.c_str(), &end);
  if (end != text.c_str() + text.size() || !std::isfinite(parsed)) {
    return false;
  }
  value = parsed;
  return true;
}

std::vector<std::string> split_csv(const std::string& line) {
  std::vector<std::string> fields;
  size_t begin = 0;
  while (true) {
    const size_t comma = line.find(',', begin);
    if (comma == std::string::npos) {
      fields.push_back(line.substr(begin));
      return fields;
    }
    fields.push_back(line.substr(begin, comma - begin));
    begin = comma + 1;
  }
}

ReplayMeta read_meta(const fs::path& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    fail(path, "cannot open (meta.json is required)");
  }
  nlohmann::json json;
  try {
    json = nlohmann::json::parse(ifs);
  } catch (const std::exception& e) {
    fail(path, std::string("invalid JSON: ") + e.what());
  }

  try {
    if (json.at("format").get<std::string>() != "glim_localization_replay") {
      fail(path, "format is not glim_localization_replay");
    }
    if (!json.at("version").is_number_integer() || json.at("version").get<int>() != 1) {
      fail(path, "unsupported version " + json.at("version").dump());
    }
    if (json.at("stamp_definition").get<std::string>() != "first_point") {
      fail(path, "stamp_definition must be first_point");
    }
    ReplayMeta meta;
    meta.per_point_time = json.at("per_point_time").get<bool>();
    const int64_t num_frames = json.at("num_frames").get<int64_t>();
    const int64_t num_imu = json.at("num_imu").get<int64_t>();
    if (num_frames < 0 || num_imu < 0) {
      fail(path, "negative count");
    }
    meta.num_frames = num_frames;
    meta.num_imu = num_imu;
    if (json.contains("conversion")) {
      if (!json["conversion"].is_object()) {
        fail(path, "conversion must be an object");
      }
      meta.conversion_json = json["conversion"].dump();
    }
    return meta;
  } catch (const nlohmann::json::exception& e) {
    fail(path, std::string("invalid field: ") + e.what());
  }
}

std::vector<ReplayImu> read_imu(const fs::path& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    fail(path, "cannot open (imu.csv is required)");
  }
  std::string line;
  if (!std::getline(ifs, line)) {
    fail_line(path, 1, "empty file");
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  if (line != kImuHeader) {
    fail_line(path, 1, std::string("header must be '") + kImuHeader + "'");
  }

  std::vector<ReplayImu> imu;
  size_t line_number = 1;
  while (std::getline(ifs, line)) {
    line_number++;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    const auto fields = split_csv(line);
    if (fields.size() != 7) {
      fail_line(path, line_number, "expected 7 columns, got " + std::to_string(fields.size()));
    }
    ReplayImu sample;
    if (!parse_int64(fields[0], sample.stamp_ns)) {
      fail_line(path, line_number, "invalid stamp_ns '" + fields[0] + "'");
    }
    for (int i = 0; i < 6; i++) {
      double value;
      if (!parse_finite_double(fields[i + 1], value)) {
        fail_line(path, line_number, "invalid or non-finite value '" + fields[i + 1] + "'");
      }
      (i < 3 ? sample.acc[i] : sample.gyro[i - 3]) = value;
    }
    if (!imu.empty() && sample.stamp_ns <= imu.back().stamp_ns) {
      fail_line(path, line_number, "stamp_ns is not strictly increasing");
    }
    imu.push_back(sample);
  }
  return imu;
}

ReplayFrameInfo read_frame_header(const fs::path& path) {
  const std::string name = path.filename().string();
  int64_t name_stamp = 0;
  if (name.size() != kStampDigits + 4 || name.compare(kStampDigits, 4, ".bin") != 0 || !parse_int64(name.substr(0, kStampDigits), name_stamp)) {
    fail(path, "file name must be <stamp_ns as 19 digits>.bin");
  }

  std::ifstream ifs(path, std::ios::binary);
  std::array<char, kReplayCloudHeaderSize> header;
  if (!ifs.read(header.data(), header.size())) {
    fail(path, "truncated header");
  }
  if (std::memcmp(header.data(), kMagic, 4) != 0) {
    fail(path, "bad magic");
  }
  const auto version = read_value<uint32_t>(header.data(), 4);
  if (version != kReplayCloudVersion) {
    fail(path, "unsupported version " + std::to_string(version));
  }

  ReplayFrameInfo info;
  info.path = path.string();
  info.stamp_ns = read_value<int64_t>(header.data(), 8);
  info.num_points = read_value<uint32_t>(header.data(), 16);
  info.flags = read_value<uint32_t>(header.data(), 20);
  const auto point_step = read_value<uint32_t>(header.data(), 24);
  const auto reserved = read_value<uint32_t>(header.data(), 28);

  if (name_stamp < 0 || info.stamp_ns < 0) {
    fail(path, "negative stamps are not supported");
  }
  if (info.stamp_ns != name_stamp) {
    fail(path, "stamp_ns " + std::to_string(info.stamp_ns) + " does not match the file name");
  }
  if (info.num_points == 0) {
    fail(path, "num_points is 0");
  }
  if (point_step != kReplayPointStep) {
    fail(path, "point_step must be " + std::to_string(kReplayPointStep) + ", got " + std::to_string(point_step));
  }
  if (info.flags & ~(kReplayFlagPointTime | kReplayFlagIntensity)) {
    fail(path, "unknown flags " + std::to_string(info.flags));
  }
  if (reserved != 0) {
    fail(path, "reserved field is not 0");
  }
  const uint64_t expected = kReplayCloudHeaderSize + static_cast<uint64_t>(info.num_points) * kReplayPointStep;
  const uint64_t actual = fs::file_size(path);
  if (actual != expected) {
    fail(path, "file size " + std::to_string(actual) + " does not match the header (expected " + std::to_string(expected) + ")");
  }
  return info;
}

}  // namespace

std::string format_stamp_ns(int64_t stamp_ns) {
  const int64_t seconds = stamp_ns / 1000000000;
  const int64_t nanoseconds = stamp_ns % 1000000000;
  char text[48];
  if (stamp_ns < 0 && (seconds == 0 || nanoseconds != 0)) {
    std::snprintf(text, sizeof(text), "-%" PRId64 ".%09" PRId64, -seconds, -nanoseconds);
  } else {
    std::snprintf(text, sizeof(text), "%" PRId64 ".%09" PRId64, seconds, nanoseconds);
  }
  return text;
}

ReplayDatasetWriter::ReplayDatasetWriter(const std::string& dir) : dir_(dir) {
  std::error_code ec;
  fs::create_directories(fs::path(dir) / "points", ec);
  if (ec) {
    throw std::runtime_error("cannot create " + dir + ": " + ec.message());
  }
}

void ReplayDatasetWriter::write_cloud(int64_t stamp_ns, const std::vector<ReplayPoint>& points, uint32_t flags) {
  if (points.empty()) {
    throw std::invalid_argument("a replay cloud needs at least one point");
  }
  if (stamp_ns < 0) {
    throw std::invalid_argument("negative stamp");
  }

  std::string data;
  data.reserve(kReplayCloudHeaderSize + points.size() * kReplayPointStep);
  data.append(kMagic, 4);
  write_value<uint32_t>(data, kReplayCloudVersion);
  write_value<int64_t>(data, stamp_ns);
  write_value<uint32_t>(data, static_cast<uint32_t>(points.size()));
  write_value<uint32_t>(data, flags);
  write_value<uint32_t>(data, kReplayPointStep);
  write_value<uint32_t>(data, 0);
  for (const auto& p : points) {
    write_value<float>(data, p.x);
    write_value<float>(data, p.y);
    write_value<float>(data, p.z);
    write_value<float>(data, (flags & kReplayFlagPointTime) ? p.t : 0.0f);
    write_value<float>(data, (flags & kReplayFlagIntensity) ? p.intensity : 0.0f);
  }

  const fs::path path = fs::path(dir_) / "points" / stamp_file_name(stamp_ns);
  std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
  if (!ofs.write(data.data(), data.size())) {
    throw std::runtime_error("cannot write " + path.string());
  }
  num_frames_++;
}

void ReplayDatasetWriter::write_imu(const std::vector<ReplayImu>& imu) {
  const fs::path path = fs::path(dir_) / "imu.csv";
  std::ofstream ofs(path, std::ios::trunc);
  ofs << kImuHeader << "\n";
  char line[256];
  for (size_t i = 0; i < imu.size(); i++) {
    if (i > 0 && imu[i].stamp_ns <= imu[i - 1].stamp_ns) {
      throw std::invalid_argument("IMU stamps must be strictly increasing");
    }
    std::snprintf(
      line, sizeof(line), "%" PRId64 ",%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n",
      imu[i].stamp_ns, imu[i].acc.x(), imu[i].acc.y(), imu[i].acc.z(), imu[i].gyro.x(), imu[i].gyro.y(), imu[i].gyro.z());
    ofs << line;
  }
  if (!ofs) {
    throw std::runtime_error("cannot write " + path.string());
  }
  num_imu_ = imu.size();
}

void ReplayDatasetWriter::finish(bool per_point_time, const std::string& conversion_json) {
  nlohmann::json meta;
  meta["format"] = "glim_localization_replay";
  meta["version"] = 1;
  meta["stamp_definition"] = "first_point";
  meta["per_point_time"] = per_point_time;
  meta["num_frames"] = num_frames_;
  meta["num_imu"] = num_imu_;
  meta["conversion"] = nlohmann::json::parse(conversion_json);

  const fs::path path = fs::path(dir_) / "meta.json";
  std::ofstream ofs(path, std::ios::trunc);
  if (!(ofs << meta.dump(2) << "\n")) {
    throw std::runtime_error("cannot write " + path.string());
  }
}

ReplayDataset ReplayDataset::open(const std::string& dir) {
  const fs::path root(dir);
  if (!fs::is_directory(root)) {
    fail(root, "not a directory");
  }

  ReplayDataset dataset;
  dataset.dir_ = dir;
  dataset.meta_ = read_meta(root / "meta.json");
  dataset.imu_ = read_imu(root / "imu.csv");

  const fs::path points_dir = root / "points";
  if (!fs::is_directory(points_dir)) {
    fail(points_dir, "not a directory");
  }
  std::vector<fs::path> files;
  for (const auto& entry : fs::directory_iterator(points_dir)) {
    files.push_back(entry.path());
  }
  std::sort(files.begin(), files.end());
  for (const auto& file : files) {
    if (file.extension() == ".ply") {
      fail(file, "PLY input is not supported; convert it to the .bin format");
    }
    dataset.frames_.push_back(read_frame_header(file));
  }

  if (dataset.frames_.size() != dataset.meta_.num_frames) {
    fail(root / "meta.json", "num_frames is " + std::to_string(dataset.meta_.num_frames) + " but points/ holds " + std::to_string(dataset.frames_.size()));
  }
  if (dataset.imu_.size() != dataset.meta_.num_imu) {
    fail(root / "meta.json", "num_imu is " + std::to_string(dataset.meta_.num_imu) + " but imu.csv holds " + std::to_string(dataset.imu_.size()));
  }
  return dataset;
}

bool ReplayDataset::has_ground_truth() const {
  return fs::is_regular_file(ground_truth_path());
}

std::string ReplayDataset::ground_truth_path() const {
  return (fs::path(dir_) / "traj_gt.txt").string();
}

glim::RawPoints::Ptr ReplayDataset::load_cloud(size_t index) const {
  const ReplayFrameInfo& info = frames_.at(index);
  std::ifstream ifs(info.path, std::ios::binary);
  std::string data((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  const uint64_t expected = kReplayCloudHeaderSize + static_cast<uint64_t>(info.num_points) * kReplayPointStep;
  if (data.size() != expected) {
    fail(info.path, "file changed after it was validated");
  }

  auto cloud = std::make_shared<glim::RawPoints>();
  cloud->stamp = info.stamp_ns * 1e-9;
  cloud->points.reserve(info.num_points);
  cloud->times.reserve(info.num_points);
  cloud->intensities.reserve(info.num_points);

  float min_time = std::numeric_limits<float>::infinity();
  for (uint32_t i = 0; i < info.num_points; i++) {
    const size_t offset = kReplayCloudHeaderSize + static_cast<size_t>(i) * kReplayPointStep;
    const float x = read_value<float>(data.data(), offset);
    const float y = read_value<float>(data.data(), offset + 4);
    const float z = read_value<float>(data.data(), offset + 8);
    const float t = read_value<float>(data.data(), offset + 12);
    const float intensity = read_value<float>(data.data(), offset + 16);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(t) || !std::isfinite(intensity)) {
      fail(info.path, "non-finite value in point " + std::to_string(i));
    }
    if (t < 0.0f || t >= 1.0f) {
      fail(info.path, "point time " + std::to_string(t) + " in point " + std::to_string(i) + " is outside [0, 1)");
    }
    min_time = std::min(min_time, t);
    cloud->points.emplace_back(x, y, z, 1.0);
    cloud->times.push_back(t);
    cloud->intensities.push_back(intensity);
  }
  if (min_time != 0.0f) {
    fail(info.path, "the minimum point time must be 0, got " + std::to_string(min_time));
  }
  return cloud;
}

}  // namespace glim_localization
