#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include <glim_localization/map/tile_streamer.hpp>

using namespace glim_localization;

namespace {

struct FakePayload {
  int id = -1;
  bool background = false;
};

using Streamer = TileStreamer<FakePayload>;

MapTile::ConstPtr make_tile(int id) {
  auto tile = std::make_shared<MapTile>();
  tile->id = id;
  tile->T_world_origin.translation() = Eigen::Vector3d(10.0 * id, 0.0, 0.0);
  return tile;
}

std::vector<int> ids(const std::vector<Streamer::PayloadPtr>& payloads) {
  std::vector<int> result;
  for (const auto& payload : payloads) {
    result.push_back(payload ? payload->id : -1);
  }
  return result;
}

/// A fake device: each tile takes `bytes[id]` (default 10). Background uploads block until released, so a test
/// controls when they finish; it can also make an upload fail.
class FakeDevice {
public:
  Streamer::Upload upload(const MapTile::ConstPtr& tile, Streamer::Context context) {
    const bool background = context == Streamer::Context::Background;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (background) {
        running_.push_back(tile->id);
        cv_.notify_all();
        cv_.wait(lock, [&] { return released_ > 0 || release_all_; });
        if (released_ > 0) {
          released_--;
        }
      }
      (background ? background_uploads_ : foreground_uploads_).push_back(tile->id);
      if (fail_.count(tile->id) || (background && fail_background_.count(tile->id))) {
        throw std::runtime_error("upload failed");
      }
    }
    auto payload = std::make_shared<FakePayload>();
    payload->id = tile->id;
    payload->background = background;
    return Streamer::Upload{payload, bytes_of(tile->id)};
  }

  size_t bytes_of(int id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = bytes_.find(id);
    return found == bytes_.end() ? 10 : found->second;
  }

  void set_bytes(int id, size_t bytes) {
    std::lock_guard<std::mutex> lock(mutex_);
    bytes_[id] = bytes;
  }
  void fail(int id) {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_[id] = true;
  }
  /// Only background uploads of the tile fail (an out-of-memory prefetch that fits later)
  void fail_background(int id) {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_background_[id] = true;
  }
  /// Let one blocked background upload finish
  void release_one() {
    std::lock_guard<std::mutex> lock(mutex_);
    released_++;
    cv_.notify_all();
  }
  void release_all() {
    std::lock_guard<std::mutex> lock(mutex_);
    release_all_ = true;
    cv_.notify_all();
  }
  /// Wait until `n` background uploads have started
  void wait_started(size_t n) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] { return running_.size() >= n; });
  }
  std::vector<int> foreground() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return foreground_uploads_;
  }
  std::vector<int> background() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return background_uploads_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::unordered_map<int, size_t> bytes_;
  std::unordered_map<int, bool> fail_;
  std::unordered_map<int, bool> fail_background_;
  std::vector<int> running_;
  std::vector<int> foreground_uploads_;
  std::vector<int> background_uploads_;
  int released_ = 0;
  bool release_all_ = false;
};

TileBudget budget(size_t max_bytes, int max_tiles) {
  TileBudget b;
  b.max_bytes = max_bytes;
  b.max_tiles = max_tiles;
  return b;
}

std::unique_ptr<Streamer> make_streamer(FakeDevice& device, const TileBudget& b, Streamer::FailureFn on_failure = nullptr) {
  return std::make_unique<Streamer>(
    b,
    [&device](const MapTile::ConstPtr& tile, Streamer::Context context) { return device.upload(tile, context); },
    [&device](const MapTile& tile) { return device.bytes_of(tile.id); },
    std::move(on_failure));
}

/// Call `step` until the streamer has seen `n` failed prefetches (the background upload finishes on its own time)
template <typename Step>
void until_prefetch_failures(const Streamer& streamer, size_t n, Step step) {
  for (int i = 0; i < 5000 && streamer.stats().prefetch_failures < n; i++) {
    step();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

}  // namespace

TEST(TileStreamer, RejectsMissingFunctions) {
  EXPECT_THROW(Streamer(budget(100, 4), nullptr, [](const MapTile&) { return size_t(1); }), std::invalid_argument);
  EXPECT_THROW(Streamer(budget(100, 4), [](const MapTile::ConstPtr&, Streamer::Context) { return Streamer::Upload(); }, nullptr), std::invalid_argument);
  EXPECT_THROW(Streamer(budget(0, 4), [](const MapTile::ConstPtr&, Streamer::Context) { return Streamer::Upload(); }, [](const MapTile&) { return size_t(1); }), std::invalid_argument);
}

TEST(TileStreamer, AcquireUploadsMissesAndReusesResidentTiles) {
  FakeDevice device;
  auto streamer = make_streamer(device, budget(1000, 8));
  const auto t0 = make_tile(0);
  const auto t1 = make_tile(1);

  EXPECT_EQ(ids(streamer->acquire({t0, t1})), (std::vector<int>{0, 1}));
  EXPECT_EQ(device.foreground(), (std::vector<int>{0, 1}));
  EXPECT_EQ(streamer->stats().misses, 2u);
  EXPECT_EQ(streamer->stats().uploads, 2u);
  EXPECT_EQ(streamer->residency().bytes(), 20u);

  const auto first = streamer->find(0);
  EXPECT_EQ(ids(streamer->acquire({t1, t0})), (std::vector<int>{1, 0}));
  EXPECT_EQ(streamer->find(0), first);
  EXPECT_EQ(device.foreground().size(), 2u);
  EXPECT_EQ(streamer->stats().misses, 2u);
}

TEST(TileStreamer, EvictsLeastRecentlyUsedWithinTheByteBudget) {
  FakeDevice device;
  auto streamer = make_streamer(device, budget(30, 8));
  std::vector<MapTile::ConstPtr> tiles;
  for (int i = 0; i < 5; i++) {
    tiles.push_back(make_tile(i));
  }

  streamer->acquire({tiles[0]});
  streamer->acquire({tiles[1]});
  streamer->acquire({tiles[2]});
  streamer->acquire({tiles[0]});  // LRU order: 1, 2, 0
  streamer->acquire({tiles[3]});  // Evicts 1
  EXPECT_FALSE(streamer->contains(1));
  EXPECT_TRUE(streamer->contains(0));
  EXPECT_TRUE(streamer->contains(2));
  EXPECT_TRUE(streamer->contains(3));
  EXPECT_EQ(streamer->stats().evictions, 1u);
  EXPECT_LE(streamer->residency().bytes(), 30u);
  EXPECT_LE(streamer->stats().peak_bytes, 30u);

  // A held payload stays valid after its tile is evicted.
  const auto held = streamer->find(2);
  streamer->acquire({tiles[4], tiles[0]});  // Evicts 2
  EXPECT_FALSE(streamer->contains(2));
  EXPECT_EQ(held->id, 2);
}

TEST(TileStreamer, KeepsTheTilesOfOneFrameEvenOverBudget) {
  FakeDevice device;
  auto streamer = make_streamer(device, budget(25, 8));
  const auto t0 = make_tile(0);
  const auto t1 = make_tile(1);
  const auto t2 = make_tile(2);

  EXPECT_EQ(ids(streamer->acquire({t0, t1, t2})), (std::vector<int>{0, 1, 2}));
  EXPECT_EQ(streamer->residency().size(), 3u);
  EXPECT_EQ(streamer->stats().over_budget, 1u);
  EXPECT_EQ(streamer->stats().evictions, 0u);

  // Back within the budget once the frame needs fewer tiles
  streamer->acquire({t2});
  EXPECT_LE(streamer->residency().bytes(), 25u);
  EXPECT_TRUE(streamer->contains(2));
  EXPECT_EQ(streamer->stats().over_budget, 1u);
}

TEST(TileStreamer, PrefetchUploadsInTheBackground) {
  FakeDevice device;
  auto streamer = make_streamer(device, budget(1000, 8));
  const auto t0 = make_tile(0);
  const auto t1 = make_tile(1);
  const auto t2 = make_tile(2);

  streamer->acquire({t0});
  EXPECT_EQ(streamer->prefetch({t0, t1, t2}), 2u);  // t0 is already resident
  EXPECT_TRUE(streamer->in_flight(1));
  EXPECT_TRUE(streamer->in_flight(2));
  EXPECT_EQ(streamer->residency().bytes(), 30u);  // In-flight tiles count with their estimate
  EXPECT_EQ(streamer->prefetch({t1, t2}), 0u);     // Already in flight

  device.release_all();
  streamer->wait_idle();
  EXPECT_TRUE(streamer->contains(1));
  EXPECT_TRUE(streamer->contains(2));
  EXPECT_EQ(device.background(), (std::vector<int>{1, 2}));
  EXPECT_EQ(streamer->stats().prefetch_uploads, 2u);

  // The prefetched tiles are hits: no upload on the caller thread
  EXPECT_EQ(ids(streamer->acquire({t1, t2})), (std::vector<int>{1, 2}));
  EXPECT_TRUE(streamer->find(1)->background);
  EXPECT_EQ(device.foreground(), (std::vector<int>{0}));
  EXPECT_EQ(streamer->stats().misses, 1u);
}

TEST(TileStreamer, PrefetchDoesNotBlockOnRunningUploads) {
  FakeDevice device;
  auto streamer = make_streamer(device, budget(1000, 8));
  const auto t1 = make_tile(1);
  streamer->prefetch({t1});
  device.wait_started(1);

  // The upload is blocked; acquire of another tile and prefetch calls return without it.
  streamer->acquire({make_tile(0)});
  streamer->prefetch({t1});
  EXPECT_TRUE(streamer->in_flight(1));

  device.release_one();
  streamer->wait_idle();
  EXPECT_TRUE(streamer->contains(1));
}

TEST(TileStreamer, AcquireWaitsForARunningPrefetch) {
  FakeDevice device;
  auto streamer = make_streamer(device, budget(1000, 8));
  const auto t1 = make_tile(1);
  streamer->prefetch({t1});
  device.wait_started(1);

  std::thread releaser([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    device.release_one();
  });
  EXPECT_EQ(ids(streamer->acquire({t1})), (std::vector<int>{1}));
  releaser.join();
  EXPECT_EQ(streamer->stats().prefetch_waits, 1u);
  EXPECT_EQ(streamer->stats().misses, 0u);
  EXPECT_TRUE(device.foreground().empty());
}

TEST(TileStreamer, AcquireTakesAQueuedPrefetchOverItself) {
  FakeDevice device;
  auto streamer = make_streamer(device, budget(1000, 8));
  const auto t1 = make_tile(1);
  const auto t2 = make_tile(2);
  streamer->prefetch({t1, t2});
  device.wait_started(1);  // t1 runs, t2 is queued

  // t2 is uploaded on the caller thread instead of waiting behind t1.
  EXPECT_EQ(ids(streamer->acquire({t2})), (std::vector<int>{2}));
  EXPECT_EQ(device.foreground(), (std::vector<int>{2}));
  EXPECT_EQ(streamer->stats().misses, 1u);
  EXPECT_TRUE(streamer->in_flight(1));

  device.release_all();
  streamer->wait_idle();
  EXPECT_EQ(device.background(), (std::vector<int>{1}));
}

TEST(TileStreamer, PrefetchStaysWithinTheBudget) {
  FakeDevice device;
  auto streamer = make_streamer(device, budget(40, 8));
  device.release_all();
  std::vector<MapTile::ConstPtr> tiles;
  for (int i = 0; i < 6; i++) {
    tiles.push_back(make_tile(i));
  }

  streamer->acquire({tiles[0], tiles[1]});
  // Room for two more; the frame's tiles are never evicted for a prefetch, and the call stops at the first misfit.
  EXPECT_EQ(streamer->prefetch({tiles[2], tiles[3], tiles[4], tiles[5]}), 2u);
  streamer->wait_idle();
  EXPECT_TRUE(streamer->contains(0));
  EXPECT_TRUE(streamer->contains(1));
  EXPECT_TRUE(streamer->contains(2));
  EXPECT_TRUE(streamer->contains(3));
  EXPECT_EQ(streamer->stats().peak_bytes, 40u);

  // The next frame moved on: tiles outside it and the prefetch list are evicted, least recently used first.
  streamer->acquire({tiles[2], tiles[3]});
  EXPECT_EQ(streamer->prefetch({tiles[4], tiles[5]}), 2u);
  streamer->wait_idle();
  EXPECT_FALSE(streamer->contains(0));
  EXPECT_FALSE(streamer->contains(1));
  EXPECT_TRUE(streamer->contains(4));
  EXPECT_TRUE(streamer->contains(5));
  EXPECT_EQ(streamer->stats().evictions, 2u);
  EXPECT_LE(streamer->stats().peak_bytes, 40u);

  // A tile larger than the budget is never prefetched.
  device.set_bytes(9, 100);
  EXPECT_EQ(streamer->prefetch({make_tile(9)}), 0u);
}

TEST(TileStreamer, DropsAFailedPrefetchSeenByPrefetch) {
  FakeDevice device;
  device.fail_background(1);
  device.release_all();
  std::vector<int> reported;
  auto streamer = make_streamer(device, budget(1000, 8), [&](int id, std::exception_ptr error) {
    reported.push_back(id);
    EXPECT_THROW(std::rethrow_exception(error), std::runtime_error);
  });
  const auto t1 = make_tile(1);
  const auto t2 = make_tile(2);
  streamer->prefetch({t1});

  // A speculative upload that failed is dropped and reported, not thrown, by the next prefetch().
  until_prefetch_failures(*streamer, 1, [&] { EXPECT_NO_THROW(streamer->prefetch({t2})); });
  EXPECT_EQ(streamer->stats().prefetch_failures, 1u);
  EXPECT_EQ(reported, (std::vector<int>{1}));
  EXPECT_FALSE(streamer->in_flight(1));
  EXPECT_FALSE(streamer->contains(1));

  // A frame that needs the tile uploads it in the foreground.
  streamer->wait_idle();
  EXPECT_EQ(ids(streamer->acquire({t1, t2})), (std::vector<int>{1, 2}));
  EXPECT_EQ(device.foreground(), (std::vector<int>{1}));
  EXPECT_EQ(streamer->residency().bytes(), 20u);
}

TEST(TileStreamer, DropsAFailedPrefetchSeenByAcquire) {
  FakeDevice device;
  device.fail_background(1);
  device.fail_background(2);
  std::vector<int> reported;
  auto streamer = make_streamer(device, budget(1000, 8), [&](int id, std::exception_ptr) { reported.push_back(id); });
  const auto t0 = make_tile(0);
  const auto t1 = make_tile(1);
  const auto t2 = make_tile(2);

  // Through acquire() of another tile: t2 is dropped, and acquire() does not throw.
  streamer->prefetch({t2});
  device.release_one();
  until_prefetch_failures(*streamer, 1, [&] { EXPECT_NO_THROW(streamer->acquire({t0})); });
  EXPECT_EQ(reported, (std::vector<int>{2}));

  // Through acquire() of the same tile while its prefetch runs: the frame needs it, so it is uploaded in the
  // foreground after the prefetch fails.
  streamer->prefetch({t1});
  device.wait_started(2);
  std::thread releaser([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    device.release_one();
  });
  std::vector<Streamer::PayloadPtr> acquired;
  EXPECT_NO_THROW(acquired = streamer->acquire({t1}));
  releaser.join();
  ASSERT_EQ(ids(acquired), (std::vector<int>{1}));
  EXPECT_FALSE(acquired.front()->background);
  EXPECT_EQ(streamer->stats().prefetch_waits, 1u);
  EXPECT_EQ(streamer->stats().prefetch_failures, 2u);
  EXPECT_EQ(reported, (std::vector<int>{2, 1}));
  EXPECT_EQ(device.foreground(), (std::vector<int>{0, 1}));
}

TEST(TileStreamer, FailsClosedWhenTheFrameTileFailsInTheForeground) {
  FakeDevice device;
  device.fail(1);
  device.release_all();
  auto streamer = make_streamer(device, budget(1000, 8));
  const auto t1 = make_tile(1);
  streamer->prefetch({t1});
  until_prefetch_failures(*streamer, 1, [&] { streamer->poll(); });
  EXPECT_EQ(streamer->stats().prefetch_failures, 1u);

  EXPECT_THROW(streamer->acquire({t1}), std::runtime_error);
  EXPECT_FALSE(streamer->contains(1));
  EXPECT_EQ(streamer->residency().bytes(), 0u);
}

TEST(TileStreamer, CancelsQueuedPrefetchesToMakeRoomForTheFrame) {
  FakeDevice device;
  auto streamer = make_streamer(device, budget(30, 8));
  const auto t0 = make_tile(0);
  const auto t1 = make_tile(1);
  const auto t2 = make_tile(2);
  const auto t3 = make_tile(3);
  EXPECT_EQ(streamer->prefetch({t1, t2, t3}), 3u);  // The whole budget, in flight
  device.wait_started(1);                            // t1 runs, t2 and t3 are queued

  // In-flight tiles are never evicted, so the frame's tile only fits after the queued prefetches it does not need
  // (t3) are cancelled; t2, which the frame needs, is taken over; the running t1 is left alone.
  EXPECT_EQ(ids(streamer->acquire({t0, t2})), (std::vector<int>{0, 2}));
  EXPECT_EQ(streamer->stats().prefetch_cancels, 1u);
  EXPECT_EQ(streamer->stats().over_budget, 0u);
  EXPECT_EQ(streamer->stats().evictions, 0u);
  EXPECT_TRUE(streamer->in_flight(1));
  EXPECT_FALSE(streamer->in_flight(3));
  EXPECT_FALSE(streamer->contains(3));
  EXPECT_LE(streamer->residency().bytes(), 30u);

  device.release_all();
  streamer->wait_idle();
  EXPECT_EQ(device.background(), (std::vector<int>{1}));
  EXPECT_EQ(device.foreground(), (std::vector<int>{0, 2}));
  EXPECT_EQ(streamer->stats().misses, 2u);
}

TEST(TileStreamer, ReplacesATileOfAnotherMapWhileItsPrefetchRuns) {
  FakeDevice device;
  auto streamer = make_streamer(device, budget(1000, 8));
  const auto old_tile = make_tile(1);
  const auto new_tile = make_tile(1);
  streamer->prefetch({old_tile});
  device.wait_started(1);

  // The running upload of the old tile is waited for and discarded; the new tile is uploaded in the foreground.
  std::thread releaser([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    device.release_one();
  });
  const auto acquired = streamer->acquire({new_tile});
  releaser.join();
  ASSERT_EQ(ids(acquired), (std::vector<int>{1}));
  EXPECT_FALSE(acquired.front()->background);
  EXPECT_EQ(device.background(), (std::vector<int>{1}));
  EXPECT_EQ(device.foreground(), (std::vector<int>{1}));
  EXPECT_FALSE(streamer->in_flight(1));
  EXPECT_EQ(streamer->residency().size(), 1u);
  EXPECT_EQ(streamer->residency().bytes(), 10u);
  EXPECT_EQ(streamer->stats().prefetch_uploads, 0u);
  EXPECT_EQ(streamer->stats().prefetch_failures, 0u);

  // The old upload's result is gone: later calls see only the new tile.
  streamer->wait_idle();
  EXPECT_EQ(streamer->acquire({new_tile}).front(), acquired.front());
  EXPECT_EQ(device.foreground().size(), 1u);
}

TEST(TileStreamer, ReplacesATileOfAnotherMapWithTheSameId) {
  FakeDevice device;
  auto streamer = make_streamer(device, budget(1000, 8));
  const auto old_tile = make_tile(1);
  const auto first = streamer->acquire({old_tile}).front();
  const auto new_tile = make_tile(1);
  const auto second = streamer->acquire({new_tile}).front();
  EXPECT_NE(first, second);
  EXPECT_EQ(device.foreground(), (std::vector<int>{1, 1}));
  EXPECT_EQ(streamer->residency().size(), 1u);
}

TEST(TileStreamer, ClearCancelsQueuedUploads) {
  FakeDevice device;
  auto streamer = make_streamer(device, budget(1000, 8));
  streamer->prefetch({make_tile(1), make_tile(2), make_tile(3)});
  device.wait_started(1);

  std::thread releaser([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    device.release_one();
  });
  streamer->clear();
  releaser.join();
  EXPECT_EQ(streamer->residency().size(), 0u);
  EXPECT_EQ(streamer->residency().bytes(), 0u);
  // Only the running upload ran.
  EXPECT_EQ(device.background(), (std::vector<int>{1}));
}

TEST(TileStreamer, DestructionWithUploadsInFlightIsClean) {
  FakeDevice device;
  {
    auto streamer = make_streamer(device, budget(1000, 8));
    streamer->prefetch({make_tile(1), make_tile(2)});
    device.wait_started(1);
    device.release_all();
  }
  EXPECT_LE(device.background().size(), 2u);
}
