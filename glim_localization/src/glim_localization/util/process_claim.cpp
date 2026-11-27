#include <glim_localization/util/process_claim.hpp>

#include <mutex>
#include <stdexcept>
#include <utility>

namespace glim_localization {

namespace {

struct ClaimState {
  std::mutex mutex;
  bool held = false;
  std::string owner;
};

ClaimState& state() {
  // Never destroyed: a claim can be released during static destruction (by a bridge that stays in
  // GLIM's static slots until the process exits), after a function-local static would be gone.
  static ClaimState* const s = new ClaimState;
  return *s;
}

}  // namespace

ProcessClaim ProcessClaim::acquire(const std::string& owner) {
  ClaimState& s = state();
  std::lock_guard<std::mutex> lock(s.mutex);
  if (s.held) {
    throw std::runtime_error("one localization per process: already held by " + s.owner);
  }
  s.held = true;
  s.owner = owner;
  ProcessClaim claim;
  claim.held_ = true;
  return claim;
}

std::string ProcessClaim::holder() {
  ClaimState& s = state();
  std::lock_guard<std::mutex> lock(s.mutex);
  return s.held ? s.owner : std::string();
}

ProcessClaim::ProcessClaim(ProcessClaim&& other) noexcept : held_(std::exchange(other.held_, false)) {}

ProcessClaim& ProcessClaim::operator=(ProcessClaim&& other) noexcept {
  if (this != &other) {
    release();
    held_ = std::exchange(other.held_, false);
  }
  return *this;
}

ProcessClaim::~ProcessClaim() {
  release();
}

void ProcessClaim::release() {
  if (!held_) {
    return;
  }
  held_ = false;
  ClaimState& s = state();
  std::lock_guard<std::mutex> lock(s.mutex);
  s.held = false;
  s.owner.clear();
}

}  // namespace glim_localization
