# localization_publisher follow-ups: reset while a frame is pending, diagnostics after a clock step ([#10](https://github.com/atinfinity/glim_localization/issues/10))

- **Change**: three of the items of [#10](https://github.com/atinfinity/glim_localization/issues/10), all in `glim_localization_ros2/` (no core or GLIM change).
  - **Reset status while a frame is pending.** A status that is not for a frame (`frame_id < 0`, a reset) now drops the pending frame and counts it in `unpaired_frames()`. Before, a frame whose status came after an EXTERNAL reset status was published, and the re-anchor used that pre-reset pose, so `odom->base` stepped at the next frame (29.9 m in the review probe). The core does not emit this order; the drop makes the publisher safe if it ever does. DESIGN section 6 (Pairing) says so.
  - **Diagnostics after the clock steps back.** The rate limit compared `now - last` with `diagnostics_period`, so after a sim clock stepped back (a bag played again) same-level diagnostics stayed silent until the clock caught up with the last publication. A clock earlier than the last publication now restarts the period.
  - **Tests.** INITIALIZING and DEGRADED now have a test that checks the pose, odometry and TF are published. The rate limit now has a test too.
- **Decisions**:
  - The pending frame is dropped at any reset status, not only an EXTERNAL one, as the issue suggests. For an IMU gap the frame is also from before the reset, and its status after the reset status is not an order the core emits.
  - A status with `frame_id < 0` also comes as LOST when the initial pose search rejects the pose after a reset (`publish_lost`). The drop is harmless then, since no frame is pending, but `frame_id < 0` does not by itself mean an applied reset.
  - A clock that steps back publishes at once and starts a new period from the new time. That is the same as the first publication. Waiting for the old time is wrong, because the old time may never come back. The limit therefore assumes a clock that is monotonic between steps: two interleaved `/clock` sources (a misconfiguration) make every status a step back and publish `/diagnostics` at the frame rate, and a step back to exactly 0 turns the limit off until time moves (the `nanoseconds() != 0` guard). Both are accepted (review finding).
- **Verification**: `scripts/ci.sh` (Docker, amd64): all checks pass, 362 core tests and the colcon tests. New tests in `test_localization_publisher`:
  - `PendingFrameIsDroppedAtAResetStatus`: frames 0 to 2, then frame 3 stored, an EXTERNAL reset status, the status of frame 3, and frame 4 30 m away. Frame 3 is not published, `unpaired_frames()` is 1, and `odom->base` at frame 4 equals the one at frame 2. Without the fix, frame 3 is published and `odom->base` steps by about 30 m.
  - `DiagnosticsAreRateLimitedAndRestartWhenTheClockStepsBack`: `use_sim_time` with `/clock` from the test. At 10 s two TRACKING statuses give one message, and a change to DEGRADED is published at once while the second DEGRADED is held back. At 11.5 s DEGRADED is published again. After the clock steps back to 5 s, DEGRADED is published at once, stamped 5 s, and the next one is held back. Without the fix, the step back gives no message.
  - `InitializingAndDegradedPublishPoseAndTf`: pose, odometry, `map->odom` and `odom->base` at both frames. A mutation that publishes only in TRACKING now fails it.
  - `package.xml` gains `<test_depend>rosgraph_msgs</test_depend>` for the `/clock` message, which the test uses through rclcpp.
- **Open** (left in [#10](https://github.com/atinfinity/glim_localization/issues/10); each needs a decision the issue does not make):
  - *External-odom lookup at epoch-scale stamps*, and the test that should stamp odom from integer nanoseconds. The two go together: a test with integer-nanosecond odom stamps fails on the current floor (3 of 200 lookups in the review probe). Options:
    1. Carry the stamp in integer nanoseconds end to end. Add `int64_t stamp_ns` to `FrameEstimate`, and have the R1 extension and R2 node fill it from the source message stamp. GLIM's `EstimationFrame::stamp` is a double, so the callers would need to keep a map from frame to header stamp. This is exact, but it touches both callers and the bridge.
    2. Look up at a time that is never after the true stamp: subtract the conversion error, derived from the stamp as `std::nextafter(stamp, inf) - stamp` (238 ns between 2^30 and 2^31 s, 477 ns from 2038), not a constant. The double-to-ns error is at most half of that, so no extra floor to 1 us is needed. That is a small local change and the lookup is at most about 0.5 us early, which is negligible for interpolation, but it fails if the odom sample at the frame stamp is the first one, as the floor already does.
    3. Keep the floor and retry the lookup 1 us earlier on an extrapolation error.

    Recommendation: option 2 now, with the test changed to integer-nanosecond odom stamps that cross microsecond boundaries. Option 1 should wait until a caller has the header stamp anyway.
  - *Equal stamps while a frame is pending.* A second frame with the same stamp is dropped as stale, and its status then publishes the first frame with the second frame's state. Options:
    1. Pair by frame id: add `frame_id` to `FrameEstimate` (the callers have `EstimationFrame::id`), and publish only when the ids match.
    2. Drop the pending frame as well when a frame with the same stamp arrives, so neither is published.
    3. Replace the pending frame with the newer one.

    Recommendation: option 1, because it pairs by identity rather than by time. `EstimationFrame::id` restarts at 0 after every reset (`frames.clear()`), so pairing by id alone is safe only together with the drop at a reset status; without it, pair by id and `reset_count`. Option 2 is the smallest safe change if the callers should not change. The core is not expected to emit equal stamps.
  - *A reset status before any frame under `use_sim_time` with no `/clock` is stamped 0.* Options:
    1. Use `status.stamp` (the reset's `min_stamp`) when it is above 0, and the clock otherwise.
    2. Do not publish `~/status` until the clock or a frame gives a time. `/diagnostics` would still go out.
    3. Keep the stamp at 0 and document it.

    Recommendation: option 1 then 3, but only after the R1 wall-time item below: in R1 `min_stamp` is the raw `/initialpose` stamp, so it must be clamped (or bounded the same way) first, or a wall-time pose under bag time would stamp `~/status` with wall time. A stamped `/initialpose` gives a real time, and a stamp of 0 is honest when no time is known. Option 2 hides a reset from a consumer.
  - *Two-container isolation of the colcon gtests* (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`). It still needs a re-run with two CI containers on one Docker network, which this change cannot do.
  - *R1: `/initialpose` with a wall-time stamp under bag time.* `min_stamp` comes from the raw header stamp. When it is far ahead of the data, every later frame is dropped with no message. Options:
    1. Warn when `min_stamp` is more than a threshold (for example 10 s) ahead of the newest IMU stamp the extension has seen. The gyro buffer holds the recent IMU samples; it would need an accessor for the newest stamp.
    2. Also clamp: treat such a stamp as 0, meaning "the next frame".
    3. Apply the R2 rule (`check_init_pose_delay`) in the other direction, as a `max_init_pose_lead`.

    Recommendation: option 2 with a warning. With no IMU sample seen yet (a pose sent before the bag starts) there is nothing to compare, so the stamp is kept. The newest stamp would come from the bridge's `GyroBuffer`, which is cleared on attach. A pose stamped far in the future of the data is almost always a clock mix-up, and "the next frame" is what the user meant. The threshold should be a parameter.
