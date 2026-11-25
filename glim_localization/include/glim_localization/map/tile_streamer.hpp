#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <glim_localization/map/map_tile.hpp>
#include <glim_localization/map/tile_residency.hpp>

namespace glim_localization {

/// Counters of a TileStreamer since construction (clear() keeps them)
struct TileStreamerStats {
  size_t uploads = 0;           ///< Tiles made resident, in the foreground or by prefetch
  size_t prefetch_uploads = 0;  ///< Of these, by prefetch in the background
  size_t misses = 0;            ///< acquire() found a tile neither resident nor in flight and uploaded it itself
  size_t prefetch_waits = 0;    ///< acquire() waited for a prefetch that was already running
  size_t evictions = 0;         ///< Resident tiles dropped to stay within the budget
  size_t over_budget = 0;       ///< acquire() calls whose own tiles did not fit in the budget
  size_t peak_bytes = 0;        ///< Most bytes held at once (resident and in flight)
};

/// Keeps device copies of map tiles within a TileBudget, uploads tiles ahead of use on a background thread, and drops
/// the least recently used ones that are not needed. The device side is injected (`upload`, `estimate`), so the
/// policy and the threading are testable without a GPU; GpuTileCache instantiates it with CUDA uploads.
///
/// - acquire(): the tiles one frame needs now. Resident tiles are used as they are; a tile whose prefetch is queued
///   but not started is taken off the queue and uploaded by the caller; a running prefetch is waited for. These tiles
///   are kept until the next acquire(), even if they alone exceed the budget (counted in stats().over_budget).
/// - prefetch(): tiles likely needed soon, most urgent first. Each one not yet held is queued for the background
///   thread if it fits in the budget after evicting least recently used tiles that are neither kept by acquire() nor
///   earlier in the list; the first one that does not fit ends the call. It never blocks on an upload.
///
/// An evicted payload is only dropped from the streamer: whoever still holds it (factors in the smoother) keeps it
/// alive. A failed background upload is rethrown by the next acquire(), prefetch() or poll() on the caller thread.
/// Tiles are keyed by id; a different tile object with a known id (another map) replaces the old one.
/// All methods must be called from one thread; only `upload` also runs on the background thread.
template <typename Payload>
class TileStreamer {
public:
  using PayloadPtr = std::shared_ptr<const Payload>;
  struct Upload {
    PayloadPtr payload;
    size_t bytes = 0;
  };
  enum class Context { Foreground, Background };
  /// Upload one tile and return its device copy and size. Called on the caller thread (Foreground) and on the
  /// background thread (Background), possibly at the same time. Throws on failure.
  using UploadFn = std::function<Upload(const MapTile::ConstPtr&, Context)>;
  /// Device bytes a tile will take, before it is uploaded. Caller thread only.
  using EstimateFn = std::function<size_t(const MapTile&)>;

  /// @throw std::invalid_argument for an invalid budget or a missing function
  TileStreamer(const TileBudget& budget, UploadFn upload, EstimateFn estimate)
  : residency_(budget),
    upload_(std::move(upload)),
    estimate_(std::move(estimate)) {
    if (!upload_ || !estimate_) {
      throw std::invalid_argument("the tile streamer needs an upload and an estimate function");
    }
  }

  ~TileStreamer() {
    cancel_and_wait();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  TileStreamer(const TileStreamer&) = delete;
  TileStreamer& operator=(const TileStreamer&) = delete;

  /// The device copies of `tiles`, in the same order. Blocks only for tiles that are not resident.
  std::vector<PayloadPtr> acquire(const std::vector<MapTile::ConstPtr>& tiles) {
    poll();
    std::unordered_set<int> keep;
    for (const auto& tile : tiles) {
      keep.insert(tile->id);
    }

    std::vector<PayloadPtr> result;
    result.reserve(tiles.size());
    for (const auto& tile : tiles) {
      result.push_back(acquire_one(tile, keep));
    }

    evict(residency_.plan_trim(keep));
    if (residency_.over_budget()) {
      stats_.over_budget++;
    }
    pinned_ = std::move(keep);
    return result;
  }

  /// Queue background uploads for `candidates` (most urgent first) within the budget.
  /// @return The number of uploads queued
  size_t prefetch(const std::vector<MapTile::ConstPtr>& candidates) {
    poll();
    std::unordered_set<int> keep = pinned_;
    size_t queued = 0;
    for (const auto& tile : candidates) {
      keep.insert(tile->id);
      const auto found = slots_.find(tile->id);
      if (found != slots_.end()) {
        // Resident or in flight (a tile of another map with the same id is left to acquire())
        continue;
      }
      const size_t estimate = estimate_(*tile);
      const auto victims = residency_.plan_eviction(estimate, keep);
      if (!victims) {
        break;
      }
      evict(*victims);

      auto job = std::make_shared<Job>();
      job->tile = tile;
      Slot slot;
      slot.tile = tile;
      slot.pending = job->promise.get_future().share();
      slots_.emplace(tile->id, std::move(slot));
      residency_.add_pending(tile->id, estimate);
      enqueue(job);
      queued++;
    }
    note_peak();
    return queued;
  }

  /// Make finished background uploads resident (most recently used), then trim to the budget.
  /// @throw whatever a failed background upload threw; the tile is forgotten first
  void poll() {
    for (auto it = slots_.begin(); it != slots_.end(); ++it) {
      auto& slot = it->second;
      if (!slot.pending.valid() || slot.pending.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        continue;
      }
      finish_pending(it->first, slot);
    }
    evict(residency_.plan_trim(pinned_));
  }

  /// Wait until no background upload is queued or running, then poll(). For tests and benchmarks.
  void wait_idle() {
    for (auto& [id, slot] : slots_) {
      if (slot.pending.valid()) {
        slot.pending.wait();
      }
    }
    poll();
  }

  /// Drop every tile. Queued uploads are cancelled and a running one is waited for (its result is discarded).
  void clear() {
    cancel_and_wait();
    slots_.clear();
    residency_.clear();
    pinned_.clear();
  }

  const TileResidency& residency() const { return residency_; }
  const TileStreamerStats& stats() const { return stats_; }
  /// A tile is resident (not only in flight)
  bool contains(int id) const { return residency_.is_resident(id); }
  bool in_flight(int id) const { return residency_.is_pending(id); }
  /// The resident payload of a tile, or null
  PayloadPtr find(int id) const {
    const auto found = slots_.find(id);
    return found == slots_.end() ? nullptr : found->second.payload;
  }

private:
  struct Job {
    MapTile::ConstPtr tile;
    std::promise<Upload> promise;
  };
  struct Slot {
    MapTile::ConstPtr tile;
    PayloadPtr payload;                  ///< Set once resident
    std::shared_future<Upload> pending;  ///< Valid while a background upload is queued or running
  };

  PayloadPtr acquire_one(const MapTile::ConstPtr& tile, const std::unordered_set<int>& keep) {
    auto found = slots_.find(tile->id);
    if (found != slots_.end() && found->second.tile != tile) {
      // Same id from another map
      drop_slot(found);
      found = slots_.end();
    }
    if (found != slots_.end() && found->second.payload) {
      residency_.touch(tile->id);
      return found->second.payload;
    }
    if (found != slots_.end()) {
      if (!cancel_queued(tile->id)) {
        stats_.prefetch_waits++;
        found->second.pending.wait();
        finish_pending(tile->id, found->second);
        return found->second.payload;
      }
      // Not started yet: upload it here instead of waiting behind the rest of the queue
      residency_.erase(tile->id);
      slots_.erase(found);
    }

    stats_.misses++;
    if (const auto victims = residency_.plan_eviction(estimate_(*tile), keep)) {
      evict(*victims);
    }
    Upload upload = upload_(tile, Context::Foreground);
    residency_.commit(tile->id, upload.bytes);
    Slot slot;
    slot.tile = tile;
    slot.payload = upload.payload;
    slots_[tile->id] = std::move(slot);
    stats_.uploads++;
    note_peak();
    return upload.payload;
  }

  /// A background upload is ready: make it resident, or forget it and rethrow its failure.
  void finish_pending(int id, Slot& slot) {
    Upload upload;
    try {
      upload = slot.pending.get();
    } catch (...) {
      residency_.erase(id);
      slots_.erase(id);
      throw;
    }
    slot.pending = std::shared_future<Upload>();
    slot.payload = upload.payload;
    residency_.commit(id, upload.bytes);
    stats_.uploads++;
    stats_.prefetch_uploads++;
    note_peak();
  }

  void drop_slot(typename std::unordered_map<int, Slot>::iterator it) {
    if (it->second.pending.valid() && !cancel_queued(it->first)) {
      it->second.pending.wait();  // Running: its result is discarded
    }
    residency_.erase(it->first);
    slots_.erase(it);
  }

  void evict(const std::vector<int>& victims) {
    for (const int id : victims) {
      residency_.erase(id);
      slots_.erase(id);
      stats_.evictions++;
    }
  }

  void note_peak() { stats_.peak_bytes = std::max(stats_.peak_bytes, residency_.bytes()); }

  void enqueue(const std::shared_ptr<Job>& job) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.push_back(job);
      if (!worker_.joinable()) {
        worker_ = std::thread([this] { run_worker(); });
      }
    }
    cv_.notify_one();
  }

  /// Take a queued job off the queue. @return false if it is not queued (running or done)
  bool cancel_queued(int id) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = std::find_if(queue_.begin(), queue_.end(), [id](const auto& job) { return job->tile->id == id; });
    if (found == queue_.end()) {
      return false;
    }
    queue_.erase(found);
    return true;
  }

  /// Cancel every queued job, wait for the running one, and forget all in-flight tiles.
  void cancel_and_wait() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.clear();  // Their promises are dropped unfulfilled; nobody waits for them below
    }
    for (auto it = slots_.begin(); it != slots_.end();) {
      if (!it->second.pending.valid()) {
        ++it;
        continue;
      }
      // The job is either running (it finishes) or was just cancelled (its promise is broken): wait() returns for both.
      it->second.pending.wait();
      residency_.erase(it->first);
      it = slots_.erase(it);
    }
  }

  void run_worker() {
    while (true) {
      std::shared_ptr<Job> job;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
        if (queue_.empty()) {
          return;
        }
        job = queue_.front();
        queue_.pop_front();
      }
      try {
        job->promise.set_value(upload_(job->tile, Context::Background));
      } catch (...) {
        job->promise.set_exception(std::current_exception());
      }
    }
  }

  TileResidency residency_;
  UploadFn upload_;
  EstimateFn estimate_;
  std::unordered_map<int, Slot> slots_;
  std::unordered_set<int> pinned_;  ///< Tiles of the last acquire()
  TileStreamerStats stats_;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<std::shared_ptr<Job>> queue_;
  bool stop_ = false;
  std::thread worker_;
};

}  // namespace glim_localization
