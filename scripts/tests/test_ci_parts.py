"""Tests for the parts of scripts/ci.sh (--part) and the CI matrix that runs them."""
import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
CI_SH = REPO / "scripts" / "ci.sh"
BUILD_CORE = REPO / "scripts" / "build_core.sh"
# The scripts ci.sh runs, in the order of a full run.
STEPS = [
    "check_deps.sh",
    "build_core.sh",
    "build_colcon.sh",
    "test_colcon.sh",
    "test_interfaces.sh",
    "test_python.sh",
    "test_replay_e2e.sh",
]
CHECK_JOBS = ["checks", "checks-arm64"]


@pytest.fixture
def stub_ci(tmp_path):
    """A copy of ci.sh next to stub scripts that log their name and the sharding variables."""
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    shutil.copy(CI_SH, scripts / "ci.sh")
    log = tmp_path / "log"
    for name in STEPS:
        stub = scripts / name
        stub.write_text(
            "#!/bin/bash\n"
            f'echo "{name} shard=${{GLIM_LOC_CTEST_SHARD:-}} tests=${{GLIM_LOC_CORE_TESTS:-run}}"'
            f' >> "{log}"\n'
        )
        stub.chmod(0o755)

    def run(*args):
        env = {k: v for k, v in os.environ.items() if not k.startswith(("CCACHE_", "GLIM_LOC_"))}
        result = subprocess.run(
            ["bash", str(scripts / "ci.sh"), *args],
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )
        calls = log.read_text().splitlines() if log.exists() else []
        log.unlink(missing_ok=True)
        return result, calls

    return run


def load_jobs():
    return yaml.safe_load((REPO / ".github" / "workflows" / "ci.yml").read_text())["jobs"]


def matrix_parts(job):
    return load_jobs()[job]["strategy"]["matrix"]["part"]


def full_run():
    return [f"{name} shard= tests=run" for name in STEPS]


def test_default_part_runs_every_check_in_order(stub_ci):
    for args in ([], ["--part", "all"], ["--part=all"]):
        result, calls = stub_ci(*args)
        assert result.returncode == 0, result.stderr
        assert calls == full_run()
        assert result.stdout.rstrip().endswith("PASS: all checks")


def test_skip_deps_still_works(stub_ci):
    result, calls = stub_ci("--skip-deps")
    assert result.returncode == 0, result.stderr
    assert calls == full_run()[1:]
    assert "SKIPPED: dependency check (recorded pass for this image)" in result.stdout


def test_core_parts_run_one_shard_and_only_the_first_checks_deps(stub_ci):
    result, calls = stub_ci("--part", "core-1/3")
    assert result.returncode == 0, result.stderr
    assert calls == ["check_deps.sh shard= tests=run", "build_core.sh shard=1/3 tests=run"]
    assert result.stdout.rstrip().endswith("PASS: part core-1/3")
    assert "PASS: all checks" not in result.stdout

    result, calls = stub_ci("--part=core-3/3")
    assert result.returncode == 0, result.stderr
    assert calls == ["build_core.sh shard=3/3 tests=run"]
    assert result.stdout.rstrip().endswith("PASS: part core-3/3")


def test_rest_skips_the_core_tests_and_runs_everything_else(stub_ci):
    result, calls = stub_ci("--part", "rest")
    assert result.returncode == 0, result.stderr
    assert calls == ["build_core.sh shard= tests=skip", *full_run()[2:]]
    assert result.stdout.rstrip().endswith("PASS: part rest")
    assert "PASS: all checks" not in result.stdout


@pytest.mark.parametrize(
    "args",
    [
        ["--part"],
        ["--part", ""],
        ["--part", "core"],
        ["--part", "core-"],
        ["--part", "core-0/2"],
        ["--part", "core-3/2"],
        ["--part", "core-1/0"],
        ["--part", "core-01/2"],
        ["--part", "core-1/2x"],
        ["--part", "core-1/2/3"],
        ["--part", "Rest"],
        ["--part", "deps"],
        ["--unknown"],
        ["rest"],
    ],
)
def test_bad_arguments_exit_2_before_running_anything(stub_ci, args):
    result, calls = stub_ci(*args)
    assert result.returncode == 2
    assert calls == []
    assert "usage: ci.sh" in result.stderr


@pytest.mark.parametrize("job", CHECK_JOBS)
def test_matrix_runs_every_shard_and_the_rest(job):
    parts = matrix_parts(job)
    shards = [part for part in parts if part != "rest"]
    assert parts.count("rest") == 1
    assert shards, "no core shard"
    n = len(shards)
    assert shards == [f"core-{k}/{n}" for k in range(1, n + 1)]


def test_both_architectures_run_the_same_parts():
    assert matrix_parts("checks") == matrix_parts("checks-arm64")


@pytest.mark.parametrize("job", CHECK_JOBS)
def test_the_matrix_parts_together_run_what_all_runs(stub_ci, job):
    calls = []
    for part in matrix_parts(job):
        result, part_calls = stub_ci("--part", part)
        assert result.returncode == 0, result.stderr
        calls += part_calls
    names = [call.split()[0] for call in calls]
    # Every check once, except the core build, which each part repeats.
    assert sorted(set(names)) == sorted(STEPS)
    for name in STEPS:
        assert names.count(name) == (len(matrix_parts(job)) if name == "build_core.sh" else 1)
    shards = sorted(call.split()[1] for call in calls if call.startswith("build_core.sh") and "tests=run" in call)
    n = len(shards)
    assert shards == sorted(f"shard={k}/{n}" for k in range(1, n + 1))


@pytest.mark.parametrize("job", CHECK_JOBS)
def test_check_job_runs_its_matrix_part(job):
    spec = load_jobs()[job]
    assert spec["strategy"]["fail-fast"] is False
    assert spec["env"]["PART"] == "${{ matrix.part }}"
    runs = [step.get("run", "") for step in spec["steps"]]
    assert any(re.search(r'scripts/ci\.sh --part "\$\{PART\}"', run) for run in runs)


@pytest.mark.parametrize(
    ("required", "checks"), [("build-and-test", "checks"), ("build-and-test-arm64", "checks-arm64")]
)
def test_required_checks_wait_for_every_part(required, checks):
    spec = load_jobs()[required]
    assert spec["needs"] == ["changes", checks]
    assert spec["if"].startswith("always()")
    assert spec["env"]["CHECKS"] == f"${{{{ needs.{checks}.result }}}}"


def run_build_core(tmp_path, **env):
    """build_core.sh with stub cmake and ctest; ctest logs its arguments and stops the script."""
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir(exist_ok=True)
    log = tmp_path / "ctest_args"
    (bin_dir / "cmake").write_text("#!/bin/bash\nexit 0\n")
    (bin_dir / "ctest").write_text(f'#!/bin/bash\necho "$@" > "{log}"\nexit 42\n')
    for tool in ("cmake", "ctest"):
        (bin_dir / tool).chmod(0o755)
    base = {k: v for k, v in os.environ.items() if not k.startswith(("CCACHE_", "GLIM_LOC_"))}
    base["PATH"] = f"{bin_dir}:{base['PATH']}"
    result = subprocess.run(
        ["bash", str(BUILD_CORE)], env={**base, **env}, capture_output=True, text=True, check=False
    )
    return result, (log.read_text().split() if log.exists() else None)


def test_build_core_passes_the_shard_to_ctest(tmp_path):
    result, args = run_build_core(tmp_path, GLIM_LOC_CTEST_SHARD="2/3")
    assert result.returncode == 42, result.stderr
    assert args[args.index("-I") + 1] == "2,,3"
    assert "--no-tests=error" in args

    result, args = run_build_core(tmp_path)
    assert result.returncode == 42, result.stderr
    assert "-I" not in args
    assert "--no-tests=error" in args


@pytest.mark.parametrize(
    "env",
    [
        {"GLIM_LOC_CTEST_SHARD": "0/2"},
        {"GLIM_LOC_CTEST_SHARD": "3/2"},
        {"GLIM_LOC_CTEST_SHARD": "1"},
        {"GLIM_LOC_CTEST_SHARD": "1,,2"},
        {"GLIM_LOC_CORE_TESTS": "no"},
        {"GLIM_LOC_CTEST_SHARD": "1/2", "GLIM_LOC_CORE_TESTS": "skip"},
    ],
)
def test_build_core_rejects_bad_settings_before_building(tmp_path, env):
    result, args = run_build_core(tmp_path, **env)
    assert result.returncode == 1
    assert "FAIL:" in result.stderr
    assert args is None
