# CI tiers measured, and the close-out of [#64](https://github.com/atinfinity/glim_localization/issues/64)

- **State of #64**: every task except the last was already done.
  - Docs site inputs count as docs-only, and `ci_local.sh` skips the same way (#65).
  - The required checks `build-and-test` and `build-and-test-arm64` keep reporting, because they sum up the parts with `always()` (#75).
  - The detector tests run in both workflows.
  - A docs-only PR (run 46047003589, PR #84) skipped the parts and passed both required checks in about 20 s.
- **Last task: measure whether more tiers are worth it.** The work splits into tiers by the paths a PR changes.
  - **Time per step**: amd64 `rest` part, run 46069274040. "Run checks" took 371 s in total.

    | Step | Time |
    |---|---|
    | Core build, no ctest | 85 s |
    | colcon build | 121 s |
    | colcon test | 56 s |
    | Python tests (350) | 65 s |
    | Replay e2e and quick robustness | 37 s |
    | Interfaces | 4 s |

    Each job also pulls the image for about 35 s. `rest` sets the wall time; the core shards took 212 to 263 s.
  - **How often each tier would apply**, over the 53 merged PRs, classified by changed paths:

    | Class | PRs |
    |---|---|
    | Docs-only (already skipped) | 10 |
    | Python tooling only (`scripts/*.py`, `scripts/tests/`, with docs) | 3 |
    | `sim/` only, or sim with Python | 3 |
    | `glim_localization_ros2/` only | 3 (5 with `scripts/test_colcon.sh`) |
    | Everything else | about 32 |

- **Decision** (approved by the owner): no further tiers; #64 is closed.
  - **A `tools` tier** (Python and sim) would apply to 6 of 53 PRs, about 11%, and save about 1 to 3 min of wall time each. That is an estimate, not measured.
    - A wrong classification would silently drop the C++ and ROS tests behind a required check.
    - The classifier is code that every PR is judged by, including PRs that change it.
    - Testing it needs a throwaway PR.
    - The gain does not justify that risk and upkeep.
    - One trap: `scripts/replay_to_bag.py` looks like tooling, but `scripts/test_colcon.sh` uses it to build the launch-test fixtures.
  - **A ROS-only tier** gives no wall-time gain, because the `rest` part, which sets the wall time, still runs in full. It would save only runner minutes, and those are free for this public repository.
  - Revisit this if the CI wall time grows a lot, or if tooling-only PRs become common. The numbers above are the baseline.
