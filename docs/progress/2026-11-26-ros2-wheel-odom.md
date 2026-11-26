# Wheel odometry input in R2 ([#14](https://github.com/atinfinity/glim_localization/issues/14))

- **Change**: the R2 node (`glim_localization_node`) subscribes to a `nav_msgs/Odometry` topic and feeds its twist to `OdometryEstimationLocalization::insert_wheel_odom`. `LocalizationStatus.msg` and the diagnostics get the four wheel counters of the core status. No core change. R1 (the GLIM extension) comes next.
- **Config** (`localization_ros` in `config/r2/config_ros.json`):

| Key | Default | Meaning |
| --- | --- | --- |
| `wheel_odom_topic` | `""` | Topic to subscribe to; empty: no subscription |
| `wheel_odom_child_frame_id` | `"base_link"` (code default `""`) | Expected `child_frame_id`; another one is rejected. Empty accepts any (the frame is logged once per activation). A fixed name: it does not follow `base_frame_id` |
| `wheel_odom_time_offset` | `0.0` | Added to the header stamp [s]; must be finite |
| `wheel_odom_qos` | sensor_data, depth 200 | As `imu_qos` |

- **Decisions**:
  - Only `twist.twist` is used, in `child_frame_id`, which must be the base of `wheel_odom_T_imu_base` (odometry config). The pose, `header.frame_id` and the covariances are ignored, and no TF is looked up: the core already holds the base-to-IMU transform, and a twist in the base frame needs nothing else. The pose of a wheel odometry topic is a dead-reckoned integral that the core does not need.
  - The `child_frame_id` check is the one guard against a twist in the wrong frame (a common mix-up with `odom`-frame twists). The shipped config expects `base_link`, the default `base_frame_id`, so a mismatch is caught by default; empty turns the check off.
  - Rejected messages (wrong `child_frame_id`, a zero stamp, a non-finite stamp or twist) are counted and warned about (throttled to one warning per 5 s). The messages the core drops (`insert_wheel_odom` returns false: not newer than the last sample) are counted separately. Both counts, with the number received, go into one log line when the node stops, next to the IMU and point counts.
  - A topic set while `wheel_odom_enabled` is false is a configuration mistake but not fatal: the node warns once and does not subscribe, so the counts stay 0.
  - The conversion (`include/glim_localization_ros2/wheel_odom_input.hpp`) is header only and free of the ROS graph, so it is tested alone and R1 can share it.
  - The QoS is read from `localization_ros` (`read_qos` takes the section now) because the key is ours, not GlimROS's. Depth 200 holds 2 s of a 100 Hz wheel topic.
  - The callback runs on the node's executor and calls the core directly; the samples bypass the frame queue. A frame gets a wheel factor only if the samples covering its interval have arrived when the odometry thread handles it, so the wheel topic must not lag the LiDAR by more than `wheel_odom_max_gap` (the core counts the frames it misses in `wheel_missing`).
  - The counters (`wheel_factors`, `wheel_gated`, `wheel_readmitted`, `wheel_missing`; per frame since the last reset, 0 while disabled) are appended to `LocalizationStatus.msg`, so the existing fields keep their order. `scripts/test_interfaces.sh` pins the new definition.
- **Tests**:
  - `test_wheel_odom_input` (gtest, no ROS graph): the twist and the offset stamp are taken; the pose and `header.frame_id` are ignored; an empty expected frame accepts any; a mismatch, a zero stamp and each non-finite component (and a non-finite offset) are rejected; every reason has a name.
  - `test_localization_publisher` `WheelCountersAreReported`: the four counters reach the message and the diagnostics.
  - `test_r2_wheel_odom` (launch test over the R1 bag, `/imu` and `/points` only, pose by `~/initialize`): with `wheel_odom_enabled` and a topic set, the node subscribes; the test publishes 5 twists stamped after the end of the bag, one with another `child_frame_id` and one with a zero stamp. The stop line must read `received 7, rejected 2, dropped by the odometry 0`; no sample covers a frame, so `wheel_missing` is above 0 and `wheel_factors` stays 0. The synthetic data has no wheel motion, so the effect of the factor on the estimate is not checked here; the core tests cover it.
- **Verification**: `scripts/docker_run.sh scripts/ci.sh`, Docker, amd64: all checks pass (330 core tests, 23 `glim_localization_ros2` tests including the linters, the replay e2e). `test_r2_wheel_odom`: 121 of 121 poses, the expected stop line, `wheel_missing` up to 70, in 18 s.
- **Not in this change**: R1 (the extension), wheel input in the replay dataset (`bag_to_replay.py`) and its evaluation, the twist covariance as the factor noise, and wheel counters in the DEGRADED/LOST rules.
- **No result video**: the change is input plumbing, and the synthetic replay has no wheel input to show.
