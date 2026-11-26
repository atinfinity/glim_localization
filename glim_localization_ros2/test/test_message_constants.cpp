// Copyright (c) 2026 atinfinity
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// The core does not include the ROS messages, so its enum values are copies of the message
// constants. Pin them here, where both are visible.
#include <gtest/gtest.h>

#include "glim_localization/odometry/odometry_estimation_localization.hpp"
#include "glim_localization_msgs/msg/localization_status.hpp"

namespace
{
using glim_localization::InitSearchReason;
using glim_localization::InitSearchResult;
using glim_localization::LocalizationState;
using glim_localization::ResetReason;
using Msg = glim_localization_msgs::msg::LocalizationStatus;
}  // namespace

TEST(MessageConstants, ResetReasonMatchesMessage) {
  EXPECT_EQ(static_cast<int>(ResetReason::NONE), Msg::RESET_NONE);
  EXPECT_EQ(static_cast<int>(ResetReason::EXTERNAL), Msg::RESET_EXTERNAL);
  EXPECT_EQ(static_cast<int>(ResetReason::IMU_GAP), Msg::RESET_IMU_GAP);
}

TEST(MessageConstants, InitSearchResultMatchesMessage) {
  EXPECT_EQ(static_cast<int>(InitSearchResult::NONE), Msg::INIT_SEARCH_NONE);
  EXPECT_EQ(static_cast<int>(InitSearchResult::ACCEPTED), Msg::INIT_SEARCH_ACCEPTED);
  EXPECT_EQ(static_cast<int>(InitSearchResult::REJECTED), Msg::INIT_SEARCH_REJECTED);
}

TEST(MessageConstants, InitSearchReasonMatchesMessage) {
  EXPECT_EQ(static_cast<int>(InitSearchReason::NONE), Msg::INIT_SEARCH_REASON_NONE);
  EXPECT_EQ(static_cast<int>(InitSearchReason::TRUNCATED), Msg::INIT_SEARCH_REASON_TRUNCATED);
  EXPECT_EQ(static_cast<int>(InitSearchReason::LOW_SCORE), Msg::INIT_SEARCH_REASON_LOW_SCORE);
  EXPECT_EQ(static_cast<int>(InitSearchReason::AMBIGUOUS), Msg::INIT_SEARCH_REASON_AMBIGUOUS);
  EXPECT_EQ(static_cast<int>(InitSearchReason::NO_CANDIDATE), Msg::INIT_SEARCH_REASON_NO_CANDIDATE);
  EXPECT_EQ(static_cast<int>(InitSearchReason::OUT_OF_RANGE), Msg::INIT_SEARCH_REASON_OUT_OF_RANGE);
}

TEST(MessageConstants, StateMatchesMessage) {
  EXPECT_EQ(static_cast<int>(LocalizationState::INITIALIZING), Msg::INITIALIZING);
  EXPECT_EQ(static_cast<int>(LocalizationState::TRACKING), Msg::TRACKING);
  EXPECT_EQ(static_cast<int>(LocalizationState::DEGRADED), Msg::DEGRADED);
  EXPECT_EQ(static_cast<int>(LocalizationState::LOST), Msg::LOST);
}
