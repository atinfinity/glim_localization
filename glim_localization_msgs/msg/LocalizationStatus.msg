# Status of the localization, published at the LiDAR rate on ~/status.
# Follows the core struct glim_localization::LocalizationStatus. The stamp is carried in header.

# Stamp is the estimated frame time; frame_id is the map frame.
# A status emitted when a reset is applied carries stamp = the reset's min_stamp, which may be 0
# and is not a frame time. Consumers must not treat it as a pose time.
std_msgs/Header header

# State machine (glim_localization::LocalizationState).
# Estimation has started or was reset; the pose is not yet trusted.
uint8 INITIALIZING=0
# Enough consecutive good frames; the pose is trusted.
uint8 TRACKING=1
# Enough consecutive bad frames; the pose is still published but is suspect.
uint8 DEGRADED=2
# Tracking is lost; left only by an external initialize.
uint8 LOST=3
uint8 state

# Number of pose resets applied so far (external pose or self-reset). Use it to tell runs apart.
# The pose given by the configuration at startup is not counted.
int32 reset_count

# Number of compiled map tiles used by the current frame.
int32 num_tiles

# Number of map factors in the current optimization.
int32 num_map_factors

# Fraction of points that fall in the finest voxels of the used tiles, in [0, 1].
float64 inlier_ratio

# Finest-level error per inlier point. Reported only; no threshold is applied.
float64 residual

# Wall-clock time spent estimating this frame [s]. 0 until the core fills it.
float64 processing_time

# Cause of the most recent reset. RESET_NONE while reset_count is 0 (and until the core fills it).
uint8 RESET_NONE=0
# A pose was given from outside (/initialpose, ~/initialize, ~/relocalize).
uint8 RESET_EXTERNAL=1
# The node reset itself after an IMU gap.
uint8 RESET_IMU_GAP=2
uint8 last_reset_reason

# Result of the search around the most recent pose given from outside (init_yaw_search in the odometry config).
# Cleared by every reset; stays NONE when the search is disabled or the reset is not searched (relocalize, IMU gap).
uint8 INIT_SEARCH_NONE=0
# Every candidate was scored, the best reached the minimum inlier ratio, and it beat the runner-up by more than
# the minimum score gap; the estimation started from its registered pose.
uint8 INIT_SEARCH_ACCEPTED=1
# The time budget ran out before every candidate was scored (init_search_candidates < init_search_candidates_total),
# the best is below the minimum inlier ratio, or the runner-up is within the minimum score gap (ambiguous).
# The state is LOST until a new pose is given.
uint8 INIT_SEARCH_REJECTED=2
uint8 init_search

# Number of candidates scored before the time budget ran out.
int32 init_search_candidates

# Number of candidates generated. A search with init_search_candidates below it was cut short and is rejected.
int32 init_search_candidates_total

# Inlier ratio of the best candidate after registration, in [0, 1], also when rejected.
float64 init_search_score

# Yaw of the best candidate's registered pose relative to the given pose [rad], in [-pi, pi].
float64 init_search_yaw_offset

# Inlier ratio of the best candidate more than two search steps away in yaw from the best, in [0, 1]; 0 if none.
float64 init_search_runner_up_score

# Its yaw relative to the given pose [rad], in [-pi, pi].
float64 init_search_runner_up_yaw_offset

# Wall-clock time of the search [s].
float64 init_search_time
