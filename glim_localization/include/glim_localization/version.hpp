#pragma once

#include <string>

namespace glim_localization {

/// @brief Version of this library.
std::string version();

/// @brief Returns true when GLIM's odometry module loader is linked.
bool is_glim_linked();

}  // namespace glim_localization
