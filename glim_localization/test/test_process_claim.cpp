#include <atomic>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <glim_localization/util/process_claim.hpp>

using glim_localization::ProcessClaim;

TEST(ProcessClaim, SecondAcquireThrowsAndNamesTheHolder) {
  ProcessClaim first = ProcessClaim::acquire("first node");
  EXPECT_TRUE(first.held());
  EXPECT_EQ(ProcessClaim::holder(), "first node");
  try {
    ProcessClaim::acquire("second node");
    FAIL() << "the second acquire did not throw";
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "one localization per process: already held by first node");
  }
  // The failed acquire leaves the first claim as it was
  EXPECT_EQ(ProcessClaim::holder(), "first node");
}

TEST(ProcessClaim, ReleaseAllowsANewAcquire) {
  {
    ProcessClaim claim = ProcessClaim::acquire("scoped");
  }
  EXPECT_EQ(ProcessClaim::holder(), "");

  ProcessClaim claim = ProcessClaim::acquire("explicit");
  claim.release();
  claim.release();  // idempotent
  EXPECT_FALSE(claim.held());
  EXPECT_EQ(ProcessClaim::holder(), "");
  EXPECT_NO_THROW(ProcessClaim::acquire("again"));
}

TEST(ProcessClaim, MoveKeepsOneClaim) {
  ProcessClaim a = ProcessClaim::acquire("moved");
  ProcessClaim b = std::move(a);
  EXPECT_FALSE(a.held());  // NOLINT(bugprone-use-after-move): checking the moved-from state
  EXPECT_TRUE(b.held());
  a.release();  // a moved-from claim releases nothing
  EXPECT_EQ(ProcessClaim::holder(), "moved");
  EXPECT_THROW(ProcessClaim::acquire("other"), std::runtime_error);
  b.release();
  EXPECT_EQ(ProcessClaim::holder(), "");
}

TEST(ProcessClaim, RacingThreadsGiveOneWinner) {
  constexpr int kThreads = 16;
  constexpr int kRounds = 50;
  for (int round = 0; round < kRounds; round++) {
    std::atomic_int ready{0};
    std::atomic_int winners{0};
    std::atomic_int losers{0};
    std::vector<ProcessClaim> claims;
    std::mutex claims_mutex;
    std::vector<std::thread> threads;
    for (int i = 0; i < kThreads; i++) {
      threads.emplace_back([&, i] {
        ready++;
        while (ready.load() < kThreads) {
          std::this_thread::yield();
        }
        try {
          ProcessClaim claim = ProcessClaim::acquire("thread " + std::to_string(i));
          winners++;
          // Kept past the end of the thread, so no later thread of this round can win
          std::lock_guard<std::mutex> lock(claims_mutex);
          claims.push_back(std::move(claim));
        } catch (const std::runtime_error&) {
          losers++;
        }
      });
    }
    for (auto& thread : threads) {
      thread.join();
    }
    ASSERT_EQ(winners.load(), 1) << "round " << round;
    ASSERT_EQ(losers.load(), kThreads - 1) << "round " << round;
    claims.clear();
    ASSERT_EQ(ProcessClaim::holder(), "") << "round " << round;
  }
}
