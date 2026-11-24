#include <limits>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include <glim_localization/odometry/quality_monitor.hpp>

using namespace glim_localization;

namespace {

constexpr double kGood = 0.8;
constexpr double kBad = 0.1;

QualityMonitorParams small_params() {
  QualityMonitorParams params;
  params.tracking_frames = 3;
  params.degraded_frames = 2;
  params.lost_frames = 6;
  return params;
}

void feed(QualityMonitor& monitor, double ratio, int count) {
  for (int i = 0; i < count; i++) {
    monitor.update(ratio);
  }
}

}  // namespace

TEST(QualityMonitorTest, StartsInitializing) {
  EXPECT_EQ(QualityMonitor().state(), LocalizationState::INITIALIZING);
}

TEST(QualityMonitorTest, EntersTrackingAfterConsecutiveGoodFrames) {
  QualityMonitor monitor(small_params());
  feed(monitor, kGood, 2);
  EXPECT_EQ(monitor.state(), LocalizationState::INITIALIZING);
  monitor.update(kBad);
  feed(monitor, kGood, 2);
  EXPECT_EQ(monitor.state(), LocalizationState::INITIALIZING);
  monitor.update(kGood);
  EXPECT_EQ(monitor.state(), LocalizationState::TRACKING);
}

TEST(QualityMonitorTest, ThresholdCountsAsGood) {
  QualityMonitor monitor(small_params());
  feed(monitor, small_params().degraded_inlier_ratio, 3);
  EXPECT_EQ(monitor.state(), LocalizationState::TRACKING);
}

TEST(QualityMonitorTest, DegradesAndRecovers) {
  QualityMonitor monitor(small_params());
  feed(monitor, kGood, 3);
  monitor.update(kBad);
  EXPECT_EQ(monitor.state(), LocalizationState::TRACKING);
  monitor.update(kBad);
  EXPECT_EQ(monitor.state(), LocalizationState::DEGRADED);
  feed(monitor, kGood, 2);
  EXPECT_EQ(monitor.state(), LocalizationState::DEGRADED);
  monitor.update(kGood);
  EXPECT_EQ(monitor.state(), LocalizationState::TRACKING);
}

TEST(QualityMonitorTest, LostFromTrackingAndOnlyResetLeaves) {
  QualityMonitor monitor(small_params());
  feed(monitor, kGood, 3);
  feed(monitor, kBad, 5);
  EXPECT_EQ(monitor.state(), LocalizationState::DEGRADED);
  monitor.update(kBad);
  EXPECT_EQ(monitor.state(), LocalizationState::LOST);

  feed(monitor, kGood, 20);
  EXPECT_EQ(monitor.state(), LocalizationState::LOST);

  monitor.reset();
  EXPECT_EQ(monitor.state(), LocalizationState::INITIALIZING);
  feed(monitor, kGood, 2);
  EXPECT_EQ(monitor.state(), LocalizationState::INITIALIZING);
  monitor.update(kGood);
  EXPECT_EQ(monitor.state(), LocalizationState::TRACKING);
}

TEST(QualityMonitorTest, LostFromInitializing) {
  QualityMonitor monitor(small_params());
  feed(monitor, kBad, 5);
  EXPECT_EQ(monitor.state(), LocalizationState::INITIALIZING);
  monitor.update(kBad);
  EXPECT_EQ(monitor.state(), LocalizationState::LOST);
}

TEST(QualityMonitorTest, GoodFrameInterruptsLostCount) {
  QualityMonitor monitor(small_params());
  feed(monitor, kGood, 3);
  feed(monitor, kBad, 5);
  monitor.update(kGood);
  feed(monitor, kBad, 5);
  EXPECT_EQ(monitor.state(), LocalizationState::DEGRADED);
}

TEST(QualityMonitorTest, NanIsBad) {
  QualityMonitor monitor(small_params());
  feed(monitor, kGood, 3);
  feed(monitor, std::numeric_limits<double>::quiet_NaN(), 2);
  EXPECT_EQ(monitor.state(), LocalizationState::DEGRADED);
}

TEST(QualityMonitorTest, RejectsNonPositiveCounts) {
  for (int field = 0; field < 3; field++) {
    QualityMonitorParams params;
    (field == 0 ? params.tracking_frames : field == 1 ? params.degraded_frames : params.lost_frames) = 0;
    EXPECT_THROW(QualityMonitor{params}, std::invalid_argument);
  }
}

TEST(QualityMonitorTest, StateNames) {
  EXPECT_STREQ(to_string(LocalizationState::INITIALIZING), "INITIALIZING");
  EXPECT_STREQ(to_string(LocalizationState::TRACKING), "TRACKING");
  EXPECT_STREQ(to_string(LocalizationState::DEGRADED), "DEGRADED");
  EXPECT_STREQ(to_string(LocalizationState::LOST), "LOST");
}
