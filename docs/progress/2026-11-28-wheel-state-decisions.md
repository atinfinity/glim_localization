# Wheel odometry: what is not done ([#14](https://github.com/atinfinity/glim_localization/issues/14))

- **Change**: docs only. Records why two items of [#14](https://github.com/atinfinity/glim_localization/issues/14) are not done, and closes the issue. The third item, wheel extrapolation over an IMU gap reset (`wheel_odom_reset_extrapolation`, default false), landed in [#82](https://github.com/atinfinity/glim_localization/pull/82) ([progress entry](2026-11-28-wheel-gap-extrapolation.md)).
- **Decisions**:
  - **A. No weighting from the wheel or driver covariance.** The wheel factor keeps its sigmas from the config, growing with the distance and angle travelled, and the covariances of `nav_msgs/Odometry` stay ignored.
    - In practice the driver covariances are static or placeholder values. A weighting driven by them would rest on numbers that carry no information.
    - Supporting it would touch every input layer: the ROS adapters (R1 and R2), the replay format (`wheel_odom.csv`), the bag conversion and the core input `insert_wheel_odom`.
  - **B. Wheel odometry does not change the tracking state rules** (TRACKING, DEGRADED, LOST; [DESIGN 5.4](../DESIGN.md#54-quality-monitoring)).
    - A geometrically degenerate scene such as a corridor already stays TRACKING: the scan still lies on the map, so its inlier ratio stays high.
    - A low inlier ratio means the map does not match the scan. Wheel input cannot fix that, so it must not mask it by keeping the state TRACKING.
    - `lost_frames` is already the knob for how long poor matching is tolerated before LOST.
- **Verification**: none; no code changed.
- **Open**: none for [#14](https://github.com/atinfinity/glim_localization/issues/14).
