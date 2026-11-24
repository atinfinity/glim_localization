#include <glim_localization/map/ply_reader.hpp>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <optional>
#include <set>
#include <sstream>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

namespace glim_localization {
namespace {

// Binary PLY reading copies little-endian bytes as they are.
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "binary PLY reading assumes a little-endian host");

/// Vertices decoded per read in the fixed-size binary path
constexpr size_t kBinaryChunkVertices = 1 << 16;

struct PlyType {
  size_t size = 0;
  bool is_float = false;
  bool is_signed = false;
};

struct PlyProperty {
  std::string name;
  bool is_list = false;
  PlyType type;        ///< The value type (of each item for a list)
  PlyType count_type;  ///< The list length type (lists only)
};

struct PlyElement {
  std::string name;
  size_t count = 0;
  std::vector<PlyProperty> properties;
};

struct PlyHeader {
  std::string format;
  std::vector<PlyElement> elements;
  size_t vertex = 0;          ///< Index of the vertex element
  int xyz[3] = {-1, -1, -1};  ///< Property index of x, y, z in the vertex element
};

std::vector<std::string> split(const std::string& line) {
  std::istringstream iss(line);
  std::vector<std::string> tokens;
  std::string token;
  while (iss >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

std::optional<long long> parse_int(const std::string& text) {
  try {
    size_t pos = 0;
    const long long value = std::stoll(text, &pos);
    if (pos != text.size()) {
      return std::nullopt;
    }
    return value;
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

std::optional<PlyType> parse_type(const std::string& name) {
  // Both the original names and the sized aliases (as written by most tools) are accepted.
  if (name == "char" || name == "int8") return PlyType{1, false, true};
  if (name == "uchar" || name == "uint8") return PlyType{1, false, false};
  if (name == "short" || name == "int16") return PlyType{2, false, true};
  if (name == "ushort" || name == "uint16") return PlyType{2, false, false};
  if (name == "int" || name == "int32") return PlyType{4, false, true};
  if (name == "uint" || name == "uint32") return PlyType{4, false, false};
  if (name == "float" || name == "float32") return PlyType{4, true, true};
  if (name == "double" || name == "float64") return PlyType{8, true, true};
  return std::nullopt;
}

/// Reads up to and including the end_header line, leaving `ifs` at the first data byte.
bool read_header(std::ifstream& ifs, const std::string& path, PlyHeader& header) {
  const auto next_line = [&](std::string& line) {
    if (!std::getline(ifs, line)) {
      return false;
    }
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    return true;
  };

  std::string line;
  if (!next_line(line) || line != "ply") {
    spdlog::error("{} is not a PLY file (the first line must be \"ply\")", path);
    return false;
  }

  bool end_header = false;
  while (next_line(line)) {
    const auto tokens = split(line);
    if (tokens.empty()) {
      continue;
    }
    const std::string& key = tokens[0];
    if (key == "comment" || key == "obj_info") {
      continue;
    }
    if (key == "end_header") {
      end_header = true;
      break;
    }
    if (key == "format") {
      if (!header.format.empty() || tokens.size() != 3) {
        spdlog::error("invalid format line in {}", path);
        return false;
      }
      if (tokens[2] != "1.0") {
        spdlog::error("unsupported PLY version {} in {} (only 1.0)", tokens[2], path);
        return false;
      }
      header.format = tokens[1];
    } else if (key == "element") {
      const auto count = tokens.size() == 3 ? parse_int(tokens[2]) : std::nullopt;
      if (!count || *count < 0) {
        spdlog::error("invalid element line \"{}\" in {}", line, path);
        return false;
      }
      for (const auto& element : header.elements) {
        if (element.name == tokens[1]) {
          spdlog::error("element {} appears more than once in {}", tokens[1], path);
          return false;
        }
      }
      PlyElement element;
      element.name = tokens[1];
      element.count = static_cast<size_t>(*count);
      header.elements.push_back(std::move(element));
    } else if (key == "property") {
      if (header.elements.empty()) {
        spdlog::error("property before any element in {}", path);
        return false;
      }
      PlyProperty property;
      std::optional<PlyType> type, count_type;
      if (tokens.size() == 5 && tokens[1] == "list") {
        property.is_list = true;
        count_type = parse_type(tokens[2]);
        type = parse_type(tokens[3]);
        property.name = tokens[4];
        if (count_type && count_type->is_float) {
          count_type.reset();
        }
      } else if (tokens.size() == 3) {
        type = parse_type(tokens[1]);
        property.name = tokens[2];
        count_type = PlyType();
      }
      if (!type || !count_type) {
        spdlog::error("invalid property line \"{}\" in {}", line, path);
        return false;
      }
      property.type = *type;
      property.count_type = *count_type;
      auto& properties = header.elements.back().properties;
      for (const auto& other : properties) {
        if (other.name == property.name) {
          spdlog::error("property {} appears more than once in element {} in {}", property.name, header.elements.back().name, path);
          return false;
        }
      }
      properties.push_back(property);
    } else {
      spdlog::error("unknown PLY header line \"{}\" in {}", line, path);
      return false;
    }
  }

  if (!end_header) {
    spdlog::error("no end_header in {}", path);
    return false;
  }
  if (header.format.empty()) {
    spdlog::error("no format line in {}", path);
    return false;
  }
  if (header.format == "binary_big_endian") {
    spdlog::error("big-endian PLY is not supported: {} (convert it to binary_little_endian or ascii)", path);
    return false;
  }
  if (header.format != "ascii" && header.format != "binary_little_endian") {
    spdlog::error("unknown PLY format {} in {}", header.format, path);
    return false;
  }

  bool has_vertex = false;
  for (size_t i = 0; i < header.elements.size(); i++) {
    // Records without properties have no size, so their count could not be checked against the file.
    if (header.elements[i].properties.empty() && header.elements[i].count > 0) {
      spdlog::error("element {} in {} has no properties", header.elements[i].name, path);
      return false;
    }
    if (header.elements[i].name == "vertex") {
      header.vertex = i;
      has_vertex = true;
    }
  }
  if (!has_vertex) {
    spdlog::error("no vertex element in {}", path);
    return false;
  }
  const auto& properties = header.elements[header.vertex].properties;
  const char* axes[3] = {"x", "y", "z"};
  for (size_t i = 0; i < properties.size(); i++) {
    for (int axis = 0; axis < 3; axis++) {
      if (properties[i].name == axes[axis]) {
        if (properties[i].is_list || !properties[i].type.is_float) {
          spdlog::error("vertex property {} in {} must be a float or double scalar", axes[axis], path);
          return false;
        }
        header.xyz[axis] = static_cast<int>(i);
      }
    }
  }
  if (header.xyz[0] < 0 || header.xyz[1] < 0 || header.xyz[2] < 0) {
    spdlog::error("missing vertex property x, y or z in {}", path);
    return false;
  }
  return true;
}

/// Bytes from the current position to the end of the stream, which is left at the current position.
std::optional<size_t> remaining_bytes(std::ifstream& ifs) {
  const std::streampos pos = ifs.tellg();
  if (pos < 0 || !ifs.seekg(0, std::ios::end)) {
    return std::nullopt;
  }
  const std::streampos end = ifs.tellg();
  if (end < pos || !ifs.seekg(pos)) {
    return std::nullopt;
  }
  return static_cast<size_t>(end - pos);
}

double read_float(const char* bytes, size_t size) {
  if (size == 4) {
    float value;
    std::memcpy(&value, bytes, sizeof(value));
    return value;
  }
  double value;
  std::memcpy(&value, bytes, sizeof(value));
  return value;
}

/// A list length; nullopt if negative.
std::optional<uint64_t> read_count(const char* bytes, const PlyType& type) {
  int64_t value = 0;
  switch (type.size) {
    case 1: value = type.is_signed ? static_cast<int64_t>(*reinterpret_cast<const int8_t*>(bytes)) : static_cast<int64_t>(*reinterpret_cast<const uint8_t*>(bytes)); break;
    case 2: {
      int16_t s;
      uint16_t u;
      std::memcpy(&s, bytes, 2);
      std::memcpy(&u, bytes, 2);
      value = type.is_signed ? s : u;
      break;
    }
    default: {
      int32_t s;
      uint32_t u;
      std::memcpy(&s, bytes, 4);
      std::memcpy(&u, bytes, 4);
      value = type.is_signed ? s : static_cast<int64_t>(u);
      break;
    }
  }
  if (value < 0) {
    return std::nullopt;
  }
  return static_cast<uint64_t>(value);
}

/// Smallest size of one record: lists may be empty.
size_t min_record_bytes(const PlyElement& element) {
  size_t bytes = 0;
  for (const auto& property : element.properties) {
    bytes += property.is_list ? property.count_type.size : property.type.size;
  }
  return bytes;
}

bool has_lists(const PlyElement& element) {
  for (const auto& property : element.properties) {
    if (property.is_list) {
      return true;
    }
  }
  return false;
}

/// Read one binary record property by property. `xyz` receives the vertex coordinates when `axes` is given.
bool read_binary_record(std::ifstream& ifs, const PlyElement& element, const int* axes, Eigen::Vector4d* xyz) {
  char bytes[8];
  for (size_t i = 0; i < element.properties.size(); i++) {
    const PlyProperty& property = element.properties[i];
    if (!property.is_list) {
      if (!ifs.read(bytes, static_cast<std::streamsize>(property.type.size))) {
        return false;
      }
      for (int axis = 0; axes && axis < 3; axis++) {
        if (axes[axis] == static_cast<int>(i)) {
          (*xyz)[axis] = read_float(bytes, property.type.size);
        }
      }
      continue;
    }
    if (!ifs.read(bytes, static_cast<std::streamsize>(property.count_type.size))) {
      return false;
    }
    const auto count = read_count(bytes, property.count_type);
    if (!count) {
      return false;
    }
    // count < 2^32 and size <= 8, so the product fits.
    const auto skip = static_cast<std::streamsize>(*count * property.type.size);
    if (skip > 0 && (!ifs.ignore(skip) || ifs.gcount() != skip)) {
      return false;
    }
  }
  return true;
}

bool read_binary(std::ifstream& ifs, const std::string& path, const PlyHeader& header, PcdPoints& points) {
  for (size_t e = 0; e < header.vertex; e++) {
    const PlyElement& element = header.elements[e];
    for (size_t i = 0; i < element.count; i++) {
      if (!read_binary_record(ifs, element, nullptr, nullptr)) {
        spdlog::error("truncated or invalid binary data in element {} ({} of {}) in {}", element.name, i, element.count, path);
        return false;
      }
    }
  }

  const PlyElement& vertex = header.elements[header.vertex];
  const size_t min_bytes = min_record_bytes(vertex);  // >= 12 (x, y and z)
  // Compare with the file before allocating, so a huge count cannot exhaust memory.
  const auto remaining = remaining_bytes(ifs);
  if (!remaining) {
    spdlog::error("failed to get the size of {}", path);
    return false;
  }
  if (vertex.count > *remaining / min_bytes) {
    spdlog::error("truncated binary data in {}: {} vertices of at least {} bytes need more than the {} bytes left", path, vertex.count,
                  min_bytes, *remaining);
    return false;
  }
  points.reserve(vertex.count);

  if (has_lists(vertex)) {
    for (size_t i = 0; i < vertex.count; i++) {
      Eigen::Vector4d p(0.0, 0.0, 0.0, 1.0);
      if (!read_binary_record(ifs, vertex, header.xyz, &p)) {
        spdlog::error("truncated or invalid binary data at vertex {} of {} in {}", i, vertex.count, path);
        return false;
      }
      points.push_back(p);
    }
    return true;
  }

  // Fixed-size records: decode in chunks.
  size_t offsets[3];
  for (int axis = 0; axis < 3; axis++) {
    offsets[axis] = 0;
    for (int i = 0; i < header.xyz[axis]; i++) {
      offsets[axis] += vertex.properties[i].type.size;
    }
  }
  std::vector<char> buffer;
  for (size_t begin = 0; begin < vertex.count; begin += kBinaryChunkVertices) {
    const size_t n = std::min(kBinaryChunkVertices, vertex.count - begin);
    buffer.resize(n * min_bytes);
    ifs.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    if (static_cast<size_t>(ifs.gcount()) != buffer.size()) {
      spdlog::error("truncated binary data in {}", path);
      return false;
    }
    for (size_t i = 0; i < n; i++) {
      const char* record = buffer.data() + i * min_bytes;
      Eigen::Vector4d p(0.0, 0.0, 0.0, 1.0);
      for (int axis = 0; axis < 3; axis++) {
        p[axis] = read_float(record + offsets[axis], vertex.properties[header.xyz[axis]].type.size);
      }
      points.push_back(p);
    }
  }
  return true;
}

bool read_ascii(std::ifstream& ifs, const std::string& path, const PlyHeader& header, PcdPoints& points) {
  std::string line;
  const auto next_record = [&]() {
    while (std::getline(ifs, line)) {
      if (line.find_first_not_of(" \t\r") != std::string::npos) {
        return true;
      }
    }
    return false;
  };

  for (size_t e = 0; e < header.vertex; e++) {
    const PlyElement& element = header.elements[e];
    for (size_t i = 0; i < element.count; i++) {
      if (!next_record()) {
        spdlog::error("truncated ascii data in element {} ({} of {}) in {}", element.name, i, element.count, path);
        return false;
      }
    }
  }

  const PlyElement& vertex = header.elements[header.vertex];
  // Each value takes at least one character and one separator (the last line may lack its newline).
  // Reject a huge count before reserving.
  const auto remaining = remaining_bytes(ifs);
  if (!remaining) {
    spdlog::error("failed to get the size of {}", path);
    return false;
  }
  if (vertex.count > (*remaining + 1) / (2 * vertex.properties.size())) {
    spdlog::error("truncated ascii data in {}: {} vertices of {} properties need more than the {} bytes left", path, vertex.count,
                  vertex.properties.size(), *remaining);
    return false;
  }
  points.reserve(vertex.count);

  for (size_t i = 0; i < vertex.count; i++) {
    if (!next_record()) {
      spdlog::error("truncated ascii data in {}: {} of {} vertices", path, i, vertex.count);
      return false;
    }
    const auto tokens = split(line);
    Eigen::Vector4d p(0.0, 0.0, 0.0, 1.0);
    size_t t = 0;
    for (size_t j = 0; j < vertex.properties.size(); j++) {
      if (t >= tokens.size()) {
        spdlog::error("vertex {} in {} has too few values", i, path);
        return false;
      }
      if (vertex.properties[j].is_list) {
        const auto count = parse_int(tokens[t]);
        if (!count || *count < 0 || static_cast<unsigned long long>(*count) > tokens.size() - t - 1) {
          spdlog::error("invalid list length {} at vertex {} in {}", tokens[t], i, path);
          return false;
        }
        t += 1 + static_cast<size_t>(*count);
        continue;
      }
      for (int axis = 0; axis < 3; axis++) {
        if (header.xyz[axis] == static_cast<int>(j)) {
          const std::string& text = tokens[t];
          char* end = nullptr;
          p[axis] = std::strtod(text.c_str(), &end);
          if (end != text.c_str() + text.size()) {
            spdlog::error("invalid value {} at vertex {} in {}", text, i, path);
            return false;
          }
        }
      }
      t++;
    }
    if (t != tokens.size()) {
      spdlog::error("vertex {} in {} has {} values, expected {}", i, path, tokens.size(), t);
      return false;
    }
    points.push_back(p);
  }
  return true;
}

}  // namespace

bool read_ply(const std::string& path, PcdPoints& points) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    spdlog::error("failed to open {}", path);
    return false;
  }

  PlyHeader header;
  if (!read_header(ifs, path, header)) {
    return false;
  }

  PcdPoints all;
  try {
    if (!(header.format == "ascii" ? read_ascii(ifs, path, header, all) : read_binary(ifs, path, header, all))) {
      return false;
    }
  } catch (const std::exception& e) {
    spdlog::error("failed to read {}: {}", path, e.what());
    return false;
  }

  PcdPoints result;
  result.reserve(all.size());
  for (const auto& p : all) {
    if (p.allFinite()) {
      result.push_back(p);
    }
  }
  if (result.size() != all.size()) {
    spdlog::info("dropped {} points with non-finite coordinates from {}", all.size() - result.size(), path);
  }
  points = std::move(result);
  return true;
}

}  // namespace glim_localization
