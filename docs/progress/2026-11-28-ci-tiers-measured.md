# CI tiers measured, and the close-out of [#64](https://github.com/atinfinity/glim_localization/issues/64)

- **State of [#64](https://github.com/atinfinity/glim_localization/issues/64)**: every task except "measure whether more tiers are worth it" was already done.
  - Docs site inputs count as docs-only, and `ci_local.sh` skips the same way ([#65](https://github.com/atinfinity/glim_localization/pull/65)).
  - The required checks `build-and-test` and `build-and-test-arm64` keep reporting, because they sum up the parts with `always()` ([#75](https://github.com/atinfinity/glim_localization/pull/75)).
  - The detector tests run in both workflows.
  - A docs-only PR ([#84](https://github.com/atinfinity/glim_localization/pull/84), [run 46047003589](https://github.com/atinfinity/glim_localization/actions/runs/46047003589)) skipped the parts and passed both required checks in about 20 s.
- **Verification** (the remaining task: measure whether more tiers are worth it):
  - **Time per step**: amd64 `rest` part of [run 46069274040](https://github.com/atinfinity/glim_localization/actions/runs/46069274040), [#86](https://github.com/atinfinity/glim_localization/pull/86). "Run checks" took 371 s in total.

    | Step, in run order | Time |
    |---|---|
    | Core build, no ctest | 85 s |
    | colcon build | 121 s |
    | colcon test | 56 s |
    | Interfaces | 4 s |
    | Python tests (350) | 65 s |
    | Replay e2e and quick robustness | 37 s |

    - Each job also pulls the image, which takes 17 to 35 s (most jobs take 33 to 35 s).
    - The `rest` job (416 s) sets the wall time. The core shards' "Run checks" took 212 to 263 s.
  - **How often each tier would apply**, over the 53 merged PRs ([#17](https://github.com/atinfinity/glim_localization/pull/17) to [#86](https://github.com/atinfinity/glim_localization/pull/86)), classified by changed paths:

    | Class | PRs |
    |---|---|
    | Docs-only (already skipped) | 10 |
    | Python tooling only (`scripts/*.py`, `scripts/tests/`, with docs) | 3 |
    | Sim tooling: `sim/` (`COLCON_IGNORE`d, including a C++ plugin), `docker/sim/`, `docker/baseline/`, `scripts/sim_*.sh` | 3 |
    | `glim_localization_ros2/` only | 3, or 5 with `scripts/test_colcon.sh` (one of the two also changes the ROS `CMakeLists.txt`) |
    | Everything else | 32 |

    None of the sim-tooling paths is a CI image input. A tools tier would still have to tell them apart from `docker/Dockerfile`, which is.
- **Decisions** (approved by the owner in the working session): no further tiers. The PR that adds this entry closes [#64](https://github.com/atinfinity/glim_localization/issues/64).
  - **A tools tier** (Python and sim) would apply to 6 of 53 PRs, about 11%. It would save an estimated 1 to 3 min of wall time each; this is not measured.
    - A wrong classification would silently drop the C++ and ROS tests behind a required check.
    - The classifier is code that judges every PR, including PRs that change it.
    - Testing it needs a throwaway PR.
    - The gain does not justify that risk and upkeep.
    - One known trap: `scripts/replay_to_bag.py` looks like tooling, but `scripts/test_colcon.sh` uses it to build the launch-test fixtures.
  - **A ROS-only tier** gives no wall-time gain, because the `rest` part, which sets the wall time, still runs in full. It would save only runner minutes, and those are free for a public repository.
  - Revisit this if the CI wall time grows a lot or if tooling-only PRs become common. The numbers above are the baseline.
