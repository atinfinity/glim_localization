#pragma once

#include <string>

namespace glim_localization {

/// @brief A process-wide claim that one localization runs in this process.
///
/// GLIM's callback slots (OdometryEstimationCallbacks, LocalizationCallbacks) are static and are not
/// thread-safe to modify while an odometry thread dispatches them, and every listener receives the
/// callbacks of every odometry instance in the process. So a front end that registers in them (the R1
/// extension, the R2 node) holds this claim for as long as it is registered. The flag lives in the
/// shared core library, so the R1 extension module and the R2 component see the same one.
///
/// The core itself does not take the claim: OdometryEstimationLocalization can have several live
/// instances (the core tests do); it is the slot registration that must be unique.
///
/// Movable, not copyable. The destructor releases the claim (a moved-from claim releases nothing).
/// Thread-safe.
class ProcessClaim {
public:
  /// @brief Take the claim for owner (a name for the error message of a later acquire).
  /// @throw std::runtime_error "one localization per process: already held by <owner>" while
  ///        another claim is live
  static ProcessClaim acquire(const std::string& owner);

  /// @brief The owner of the live claim, or an empty string when none is held.
  static std::string holder();

  ProcessClaim(ProcessClaim&& other) noexcept;
  ProcessClaim& operator=(ProcessClaim&& other) noexcept;
  ProcessClaim(const ProcessClaim&) = delete;
  ProcessClaim& operator=(const ProcessClaim&) = delete;
  ~ProcessClaim();

  /// @brief Release the claim now. Idempotent.
  void release();

  /// @brief True until released or moved from.
  bool held() const { return held_; }

private:
  ProcessClaim() = default;

  bool held_ = false;
};

}  // namespace glim_localization
