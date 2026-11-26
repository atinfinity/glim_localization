# Wheel odometry from a bag into the replay dataset ([#14](https://github.com/atinfinity/glim_localization/issues/14))

- **Change**: `scripts/bag_to_replay.py --wheel-odom-topic <topic>` writes `wheel_odom.csv` from a `nav_msgs/Odometry` topic, so a real bag can be replayed with and without wheel odometry (`glim_localization_replay --wheel-odom on|off`, [#55](https://github.com/atinfinity/glim_localization/pull/55)).
- **Rules**: the same as the ROS nodes (`convert_wheel_odom` in `wheel_odom_input.hpp`), so a bag gives the same samples offline as online:
  - only `twist.twist` is used, in `child_frame_id`; the pose, `header.frame_id` and the covariances are ignored;
  - `--wheel-odom-child-frame` (default `base_link`, as `wheel_odom_child_frame_id` of the ROS configs; `''` accepts any) must match `child_frame_id`;
  - a zero header stamp or a non-finite value is excluded;
  - the stamp is the header stamp plus `--wheel-odom-time-offset` (seconds, finite, at most 1e9 s in magnitude).
- **Offline only**: a stamp that is negative after the offset is excluded and counted, never a conversion error (the dataset has no negative stamps, and a negative header stamp may be brought above 0 by the offset), and a repeated stamp keeps the first message, as the IMU does. The rows are sorted by stamp, so messages out of order in the bag are kept.
- **meta.json**: `num_wheel_odom`, and `conversion.wheel_odom` with the topic, the frame, the offset and the counts (`received`, `child_frame_mismatch`, `zero_stamp`, `non_finite`, `negative_stamp`, `duplicate_stamp`); `null` without the option.
- **Errors**: no usable message is a conversion error (exit code 1) that lists the counts and points at `--wheel-odom-child-frame`, since a wrong frame is the likely cause. A missing topic or a topic of another type is exit code 1. `--wheel-odom-child-frame` or `--wheel-odom-time-offset` without `--wheel-odom-topic`, or a non-finite or too large offset, is exit code 2.
- **Tests** (`scripts/tests/test_bag_to_replay.py`, `test_replay_to_bag.py`): optional by default; the exclusions and counts, the offset, sorting and the first of a repeated stamp; `''` accepts any frame; the errors; and a round trip of `replay_to_bag.py --wheel-odom` back through `bag_to_replay.py` with the same stamps and twists.
- **Verification**: `scripts/docker_run.sh scripts/ci.sh`, Docker, amd64: all checks pass (345 core tests, 23 `glim_localization_ros2` tests, the Python tests, the replay robustness and e2e). The review fixes (a negative header stamp is counted instead of aborting, an empty topic is exit 2) are Python only and were rerun with `test_bag_to_replay.py` and `test_replay_to_bag.py` (51 passed).
- **Open** (#14): the twist covariance as the factor noise, and the DEGRADED/LOST rules with wheel odometry.
- **No result video**: the change is input conversion; the comparison with and without wheel odometry is in [#55](https://github.com/atinfinity/glim_localization/pull/55).
