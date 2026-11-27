// Writes a synthetic map dump and a matching replay dataset. Not installed.
#include <cstdlib>
#include <iostream>

#include "synthetic/synthetic_map.hpp"
#include "synthetic/synthetic_replay.hpp"

int main(int argc, char** argv) {
  if (argc < 2 || argc > 4) {
    std::cerr << "usage: synthetic_replay_dataset <out_dir> [duration_s=12] [seed]\n"
              << "  writes <out_dir>/map (a GLIM dump), <out_dir>/map.pcd (the same points as one cloud),\n"
              << "  and <out_dir>/data (a replay dataset with traj_gt.txt)\n"
              << "  seed: the noise of the map points and of the LiDAR ranges (default: the fixed seeds of the tests)" << std::endl;
    return 2;
  }
  const std::string out = argv[1];
  const double duration = argc >= 3 ? std::atof(argv[2]) : 12.0;
  if (!(duration > 0.0)) {
    std::cerr << "invalid duration" << std::endl;
    return 2;
  }

  try {
    auto map_params = glim_localization::test::replay_map_params();
    glim_localization::test::SyntheticReplayOptions options;
    if (argc == 4) {
      const unsigned long seed = std::strtoul(argv[3], nullptr, 10);
      map_params.seed = static_cast<unsigned int>(seed);
      options.lidar.seed = static_cast<unsigned int>(seed + 1);
    }
    glim_localization::test::write_synthetic_dump(out + "/map", map_params);
    glim_localization::test::write_synthetic_pcd(
      out + "/map.pcd",
      glim_localization::test::make_synthetic_world_points(map_params),
      glim_localization::test::PcdEncoding::Binary);
    const int frames = glim_localization::test::write_synthetic_replay(out + "/data", duration, map_params, options);
    std::cout << "wrote " << frames << " frames to " << out << "/data and the map to " << out << "/map" << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
