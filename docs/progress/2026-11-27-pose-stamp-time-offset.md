# Pose stamps are frame time ([#13](https://github.com/atinfinity/glim_localization/issues/13))

One item of [#13](https://github.com/atinfinity/glim_localization/issues/13) said that `points_time_offset` is applied to the frame stamps but not to the pose stamps in the late-pose check and `min_stamp` (R1 too). Another said that no test checks the stamp that the node feeds into the late-pose check. Both are closed here: not shifting is correct, and a launch test pins it.

- **Change**:
  - No change in the stamp handling. Comments in `estimate_bridge.hpp`, `init_pose_delay.hpp`, `localization_node.cpp` and `localization_extension.cpp` say that the pose stamp is frame time and is not shifted.
  - DESIGN 6 ("Late initial pose") replaces the sentence that listed the missing shift as a limitation. docs/ros2.md says the same at `max_init_pose_delay`, `/initialpose` and `~/initialize`.
  - New launch test `test_r2_time_offset.py`. `r2_helpers` gained `glim_ros_overrides` for `make_config_dir`, and `time_offset` for `play_and_wait` and `pose_errors` (the published stamps are ahead of the ground truth by the offset).
- **Decisions**:
  - Pose stamps on `/initialpose` and `~/initialize` are frame time: the cloud header + `glim_ros.points_time_offset`. That is the time base of `~/pose`, `~/status`, TF and the IMU. On a synced system it is also the time base of the ROS clock. The node compares a pose stamp with the frame stamps as it is, in R1 and R2.
  - The first version of this change did the opposite. It took the pose stamp as sensor time and added the offset in R1 and R2. The review rejected it. RViz 2D Pose Estimate, the documented producer, stamps a click with `now()`. When `points_time_offset` maps an unsynced LiDAR clock onto the host and IMU time, `now()` already is frame time, so the shift counted the offset twice:
    - with -1 s, every click was rejected as too late;
    - with +1 s, a second of frames after the click was dropped.
    
    Published stamps are frame time too, so a client that copies them would also be shifted twice.
  - A client that has only a raw cloud header stamp must add `points_time_offset` itself.
- **Verification** (Docker, amd64):
  - `test_r2_time_offset` runs the node with `imu_time_offset` = `points_time_offset` = 5 s on the R1 bag, with `/imu` and `/points` only. It checks three things:
    - A `~/initialize` before the playback, stamped with the frame time of the frame at 1 s (raw + 5), applies at that frame: the first `~/pose` is stamped 6.000. The poses from 3 s on are within 0.1 m and 1 deg of the ground truth.
    - After the playback, a pose at an early frame time is rejected as too late, and so is one stamped with the raw cloud header of the last frame (late by 5 s).
    - A pose at the last frame time is accepted.
  - Mutation check: with the shift added to `request_reset` (stamp + offset unless 0), the test fails at the first check with "11.0 != 6.0": the pose applies 5 s late. With that check skipped, it fails on the late-pose part with "a ~/initialize at the raw cloud header of the last frame was accepted".
  - `scripts/ci.sh` passed: core 370/370, colcon 24/24, all ament linters (cpplint at 100, and at 150 for the derived `localization_node.cpp`; uncrustify; flake8 at 99). In `test_r2_time_offset` (20 s), 111 poses came, the first at 6.000. The worst error was 0.012 m and 0.079 deg over 81 poses.
- **Open**: no R1 launch test for the stamps. `test_r1_extension` runs with offset 0, and a second R1 run with an offset would add another full GLIM run to CI. R1 passes the stamp to `request_reset` as is, like R2.
