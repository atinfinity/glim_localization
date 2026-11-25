"""Map cache of scripts/smoke_os1_128.sh: the key and validation logic on fake map directories, and the
script itself on a fake dataset with fake mapper, map_compiler and replay tools (no real data)."""

import json
import os
import shutil
import stat
import subprocess
import sys
import textwrap

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import smoke_map_cache as cache  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SMOKE = os.path.join(REPO, "scripts", "smoke_os1_128.sh")


# ---------------------------------------------------------------------------------------------
# Unit tests: key, check, seal, install


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as handle:
        handle.write(text)


def traj_text(n=20, t0=100.0):
    return "".join(f"{t0 + 0.1 * i:.9f} {0.5 * i:.6f} 0 0 0 0 0 1\n" for i in range(n))


def fake_entry(path, poses=20, coverage=0.99):
    write(os.path.join(path, "traj_glim.txt"), traj_text(poses))
    summary = {"failed": False, "trajectory_coverage": coverage, "num_trajectory_poses": poses}
    write(os.path.join(path, "mapper_summary.json"), json.dumps(summary))
    write(os.path.join(path, "dump", "000000", "data.txt"), "submap 0\n")
    write(os.path.join(path, "compiled", "tiles", "0.bin"), "tile 0\n")
    write(os.path.join(path, "mapper.log"), "log\n")
    return path


@pytest.fixture
def inputs(tmp_path):
    data = tmp_path / "data"
    write(str(data / "points" / "1.bin"), "a")
    write(str(data / "imu.csv"), "imu")
    config = tmp_path / "config"
    write(str(config / "config_sensors.json"), '{"T_lidar_imu": [0, 0, 0, 0, 0, 0, 1]}')
    write(str(config / "config_odometry.json"), '{"so_name": "libfake_module.so"}')
    tools = tmp_path / "tools"
    write(str(tools / "mapper"), "#!/bin/sh\n")
    libs = tmp_path / "libs"
    write(str(libs / "libfake_module.so"), "module v1")
    return tmp_path


def key_for(base, **override):
    args = {
        "inputs": [f"data={base / 'data'}", f"mapping_config={base / 'config'}"],
        "values": ["mapper_args=--end 80", "prior_end=80"],
        "binaries": [str(base / "tools" / "mapper")],
        "module_configs": [str(base / "config")],
        "search_path": [str(base / "libs")],
    }
    args.update(override)
    return cache.make_key(**args)


def test_key_is_deterministic_and_names_the_changed_component(inputs):
    first = key_for(inputs)
    assert key_for(inputs) == first
    assert f"binary:{os.path.realpath(inputs / 'libs' / 'libfake_module.so')}" in first["components"]

    write(str(inputs / "data" / "points" / "1.bin"), "b")
    changed = key_for(inputs)
    assert changed["key"] != first["key"]
    assert cache.changed_components(first["components"], changed["components"]) == ["input data"]

    assert cache.changed_components(changed["components"], key_for(inputs, values=["mapper_args=--end 70", "prior_end=70"])["components"]) == [
        "value mapper_args", "value prior_end"]

    write(str(inputs / "libs" / "libfake_module.so"), "module v2")
    assert cache.changed_components(changed["components"], key_for(inputs)["components"]) == ["binary libfake_module.so"]


def test_directory_hash_sees_names_contents_and_new_files(inputs):
    data = inputs / "data"
    base = cache.hash_tree(str(data))
    write(str(data / "points" / "2.bin"), "")
    added = cache.hash_tree(str(data))
    assert added != base
    os.rename(data / "points" / "2.bin", data / "points" / "3.bin")
    assert cache.hash_tree(str(data)) not in (base, added)
    with pytest.raises(cache.CacheError):
        cache.hash_tree(str(inputs / "missing"))


def test_missing_module_is_a_component(inputs):
    key = key_for(inputs, search_path=[str(inputs / "nowhere")])
    assert key["components"]["missing:libfake_module.so"] == "not found"


def test_parse_ldd():
    text = textwrap.dedent("""\
        \tlinux-vdso.so.1 (0x0000ffff9a1d0000)
        \tlibglim.so => /opt/glim/lib/libglim.so (0x0000ffff99e00000)
        \tlibgone.so => not found
        \tlibc.so.6 => /lib/aarch64-linux-gnu/libc.so.6 (0x0000ffff99c00000)
        \t/lib/ld-linux-aarch64.so.1 (0x0000ffff9a190000)
        """)
    resolved, missing = cache.parse_ldd(text)
    assert resolved == ["/opt/glim/lib/libglim.so", "/lib/aarch64-linux-gnu/libc.so.6", "/lib/ld-linux-aarch64.so.1"]
    assert missing == ["libgone.so"]


@pytest.fixture
def sealed(inputs):
    key = key_for(inputs)
    entry = fake_entry(str(inputs / "cache" / "map_full"))
    cache.seal(entry, key)
    assert cache.check(entry, key) is None
    return entry, key


def test_check_reports_a_missing_entry_and_a_key_change(inputs, sealed):
    entry, key = sealed
    assert cache.check(str(inputs / "cache" / "map_prefix"), key) == "no cached map"
    other = key_for(inputs, values=["mapper_args=--end 80", "prior_end=79"])
    assert cache.check(entry, other) == "the key changed: value prior_end"


@pytest.mark.parametrize("damage, expected", [
    (lambda e: write(os.path.join(e, "compiled", "tiles", "0.bin"), "tile X\n"), "contents differ"),
    (lambda e: write(os.path.join(e, "dump", "000000", "data.txt"), "sub"), "size differs"),
    (lambda e: os.remove(os.path.join(e, "dump", "000000", "data.txt")), "missing files"),
    (lambda e: write(os.path.join(e, "compiled", "extra.bin"), "x"), "unexpected files"),
    (lambda e: os.remove(os.path.join(e, cache.ENTRY_FILE)), "incomplete"),
    (lambda e: write(os.path.join(e, cache.ENTRY_FILE), "{"), "incomplete"),
])
def test_check_rejects_a_damaged_entry(sealed, damage, expected):
    entry, key = sealed
    damage(entry)
    reason = cache.check(entry, key)
    assert reason is not None and expected in reason


def test_check_revalidates_the_content_even_with_a_matching_manifest(sealed):
    entry, key = sealed
    # A file changed and its manifest entry forged: the content checks still run
    write(os.path.join(entry, "traj_glim.txt"), traj_text(19))
    meta = json.load(open(os.path.join(entry, cache.ENTRY_FILE)))
    meta["files"] = cache.manifest(entry)
    write(os.path.join(entry, cache.ENTRY_FILE), json.dumps(meta))
    assert "19 poses but mapper_summary.json says 20" in cache.check(entry, key)


@pytest.mark.parametrize("traj, summary, expected", [
    (traj_text(20).replace("0.500000", "nan", 1), None, "finite"),
    (traj_text(20).replace("0 0 0 1\n", "0 0 0 2\n", 1), None, "quaternion"),
    ("".join(reversed(traj_text(20).splitlines(True))), None, "not increasing"),
    ("100.0 1 2 3\n" + traj_text(19, 101.0), None, "8 finite numbers"),
    (traj_text(1), {"failed": False, "trajectory_coverage": 1.0}, "fewer than 2"),
    (None, {"failed": True, "trajectory_coverage": 1.0}, "failed: false"),
    (None, {"trajectory_coverage": 1.0}, "failed: false"),
    (None, {"failed": False, "trajectory_coverage": 0.94}, "below 0.95"),
    (None, {"failed": False, "trajectory_coverage": "1"}, "finite number"),
])
def test_seal_refuses_an_invalid_map_or_trajectory(tmp_path, traj, summary, expected):
    entry = fake_entry(str(tmp_path / "build"))
    if traj is not None:
        write(os.path.join(entry, "traj_glim.txt"), traj)
    if summary is not None:
        write(os.path.join(entry, "mapper_summary.json"), json.dumps(summary))
    with pytest.raises(cache.CacheError, match=expected):
        cache.seal(entry, {"key": "k", "components": {}})
    assert not os.path.exists(os.path.join(entry, cache.ENTRY_FILE))


def test_seal_refuses_an_empty_compiled_map(tmp_path):
    entry = fake_entry(str(tmp_path / "build"))
    shutil.rmtree(os.path.join(entry, "compiled"))
    os.makedirs(os.path.join(entry, "compiled", "empty"))
    with pytest.raises(cache.CacheError, match="compiled/ is missing or empty"):
        cache.seal(entry, {"key": "k", "components": {}})


def test_install_replaces_the_entry_and_refuses_an_unsealed_build(inputs, sealed):
    entry, key = sealed
    build = fake_entry(str(inputs / "cache" / ".tmp-map_full-1"), poses=30)
    with pytest.raises(cache.CacheError, match="not sealed"):
        cache.install(build, entry)
    cache.seal(build, key)
    cache.install(build, entry)
    assert not os.path.exists(build)
    assert sorted(os.listdir(inputs / "cache")) == ["map_full"]
    assert cache.check(entry, key) is None
    assert len(open(os.path.join(entry, "traj_glim.txt")).readlines()) == 30


def test_cli_check_exit_codes(inputs, sealed):
    entry, key = sealed
    key_path = str(inputs / "key.json")
    write(key_path, json.dumps(key))
    assert cache.main(["check", "--entry", entry, "--key", key_path]) == 0
    os.remove(os.path.join(entry, "mapper.log"))
    assert cache.main(["check", "--entry", entry, "--key", key_path]) == 1


# ---------------------------------------------------------------------------------------------
# The script on a fake dataset: fake mapper, map_compiler and replay tools write plausible files

T0_NS = 1_000_000_000_000
FRAMES = 400  # 40 s at 10 Hz; a 20 s circle, so the tail revisits the prefix

CIRCLE = """
import math
def pose(t):  # a circle of radius 20 m, 20 s per turn, from the first frame
    a = 2 * math.pi * (t - %(t0)r) / 20.0
    return "%%.9f %%.6f %%.6f 0 0 0 %%.9f %%.9f" %% (t, 20 * math.cos(a), 20 * math.sin(a), math.sin(a / 2), math.cos(a / 2))
""" % {"t0": T0_NS * 1e-9}

FAKE_MAPPER = CIRCLE + """
import argparse, json, os, sys
p = argparse.ArgumentParser()
p.add_argument("--data"); p.add_argument("--config"); p.add_argument("--out"); p.add_argument("--end", type=float)
a = p.parse_args()
with open(os.environ["FAKE_LOG"], "a") as log:
    log.write("mapper %s\\n" % ("prefix" if a.end else "full"))
os.makedirs(os.path.join(a.out, "dump"), exist_ok=True)
open(os.path.join(a.out, "dump", "graph.txt"), "w").write("partial\\n")
if os.environ.get("FAKE_MAPPER_FAIL"):
    sys.exit(1)
stamps = sorted(int(n[:-4]) * 1e-9 for n in os.listdir(os.path.join(a.data, "points")))
stamps = [t for t in stamps if a.end is None or t <= a.end + 1e-6]
open(os.path.join(a.out, "traj_glim.txt"), "w").write("".join(pose(t) + "\\n" for t in stamps))
json.dump({"failed": False, "trajectory_coverage": 1.0, "num_trajectory_poses": len(stamps), "num_submaps": 3},
          open(os.path.join(a.out, "mapper_summary.json"), "w"))
"""

FAKE_COMPILER = """
import os, shutil, sys
shutil.copytree(sys.argv[1], sys.argv[2])
open(os.path.join(sys.argv[2], "map.txt"), "w").write("compiled\\n")
print("wrote 3 submaps")
"""

FAKE_REPLAY = CIRCLE + """
import argparse, json, os
p = argparse.ArgumentParser()
for name in ("--data", "--config", "--map", "--out", "--start", "--init-stamp"):
    p.add_argument(name)
p.add_argument("--init-pose", nargs=7)
a = p.parse_args()
assert os.path.isfile(os.path.join(a.map, "map.txt")), "not a compiled map"
start = float(a.start or a.init_stamp)
stamps = sorted(int(n[:-4]) for n in os.listdir(os.path.join(a.data, "points")))
stamps = [s for s in stamps if s * 1e-9 >= start - 1e-6]
os.makedirs(a.out, exist_ok=True)
open(os.path.join(a.out, "traj_localization.txt"), "w").write("".join(pose(s * 1e-9) + "\\n" for s in stamps))
with open(os.path.join(a.out, "frames.csv"), "w") as f:
    f.write("stamp_ns,result,t_load_ms,t_preprocess_ms,t_odometry_ms\\n")
    f.writelines("%d,estimated,1,1,10\\n" % s for s in stamps)
json.dump({"num_frames": len(stamps)}, open(os.path.join(a.out, "summary.json"), "w"))
print("replayed %d frames" % len(stamps))
"""


def make_tool(path, body):
    write(path, f"#!{sys.executable}\n" + body)
    os.chmod(path, os.stat(path).st_mode | stat.S_IXUSR)


@pytest.fixture
def smoke(tmp_path):
    if shutil.which("flock") is None or shutil.which("realpath") is None:
        pytest.skip("needs flock and GNU realpath (run inside the container)")
    data = tmp_path / "dataset"
    os.makedirs(data / "points")
    for i in range(FRAMES):
        open(data / "points" / f"{T0_NS + i * 100_000_000}.bin", "w").close()
    write(str(data / "imu.csv"), "stamp_ns,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\n")
    tools = tmp_path / "build" / "tools"
    make_tool(str(tools / "glim_localization_mapper"), FAKE_MAPPER)
    make_tool(str(tools / "map_compiler"), FAKE_COMPILER)
    make_tool(str(tools / "glim_localization_replay"), FAKE_REPLAY)
    log = tmp_path / "calls.log"
    cache_dir = tmp_path / "cache"

    def run(*args, expect=0, **env):
        log.write_text("")
        environment = dict(os.environ, GLIM_LOC_PYTHON=sys.executable, SMOKE_BUILD_DIR=str(tmp_path / "build"),
                           SMOKE_CACHE_DIR=str(cache_dir), FAKE_LOG=str(log), PRIOR_END="20", TAIL_START="22",
                           LD_LIBRARY_PATH="")
        environment.update(env)
        result = subprocess.run(["bash", SMOKE, *args, str(data), str(tmp_path / "run")], env=environment,
                                capture_output=True, text=True, check=False)
        output = result.stdout + result.stderr
        assert result.returncode == expect, output
        calls = log.read_text().split()[1::2]
        return output, calls

    run.tmp_path = tmp_path
    run.cache = cache_dir
    run.tools = tools
    return run


def maps(run):
    return json.load(open(run.tmp_path / "run" / "smoke_summary.json"))["maps"]


def test_smoke_cold_warm_force_and_invalidation(smoke):
    output, calls = smoke()
    assert calls == ["full", "prefix"]
    assert "map_full: REBUILDING (no cached map)" in output
    assert "RESULT: PASS" in output
    summary = json.load(open(smoke.tmp_path / "run" / "smoke_summary.json"))
    for step in ("mapper_full", "map_compiler_full", "mapper_prefix", "map_compiler_prefix", "replay_heldout",
                 "evaluate_heldout", "total"):
        assert step in summary["timings_s"]
    assert sorted(os.listdir(smoke.cache)) == [".lock", ".smoke_os1_128_cache", "map_full", "map_prefix"]

    output, calls = smoke()
    assert calls == []
    assert "map_full: REUSED" in output and "map_prefix: REUSED" in output
    assert "replay_heldout" in json.load(open(smoke.tmp_path / "run" / "smoke_summary.json"))["timings_s"]
    assert "RESULT: PASS" in output

    output, calls = smoke("--force")
    assert calls == ["full", "prefix"]
    assert maps(smoke) == {"full": {"action": "rebuilt", "reason": "--force"},
                           "prefix": {"action": "rebuilt", "reason": "--force"}}

    # A config change invalidates both maps; a split change only the prefix
    output, calls = smoke(CONFIG_PATCH="random_downsample_target 5000")
    assert calls == ["full", "prefix"]
    assert maps(smoke)["full"]["reason"] == "the key changed: input mapping_config"
    output, calls = smoke(CONFIG_PATCH="random_downsample_target 5000", PRIOR_END="19")
    assert calls == ["prefix"]
    assert maps(smoke)["prefix"]["reason"] == "the key changed: value mapper_args, value prior_end"

    # A changed binary
    with open(smoke.tools / "map_compiler", "a") as handle:
        handle.write("# v2\n")
    output, calls = smoke()
    assert calls == ["full", "prefix"]
    assert "binary map_compiler" in maps(smoke)["full"]["reason"]


def test_smoke_rejects_a_corrupt_cached_map(smoke):
    smoke()
    with open(smoke.cache / "map_full" / "compiled" / "map.txt", "w") as handle:
        handle.write("corrupted\n")
    output, calls = smoke()
    assert calls == ["full"]
    assert "map_full: REBUILDING (the cached map is corrupt (compiled/map.txt: size differs))" in output
    assert "map_prefix: REUSED" in output


def test_smoke_never_reuses_a_failed_build(smoke):
    output, calls = smoke(expect=1, FAKE_MAPPER_FAIL="1")
    assert calls == ["full"]
    assert sorted(os.listdir(smoke.cache)) == [".lock", ".smoke_os1_128_cache"]  # no entry, no temporary build left

    smoke()
    before = open(smoke.cache / "map_full" / "cache_entry.json").read()
    # A failed forced rebuild keeps the previous complete entry, which is then reused
    output, calls = smoke("--force", expect=1, FAKE_MAPPER_FAIL="1")
    assert sorted(os.listdir(smoke.cache)) == [".lock", ".smoke_os1_128_cache", "map_full", "map_prefix"]
    assert open(smoke.cache / "map_full" / "cache_entry.json").read() == before
    output, calls = smoke()
    assert calls == []


def test_smoke_refuses_a_foreign_cache_dir(smoke):
    os.makedirs(smoke.cache)
    write(str(smoke.cache / "keep.txt"), "not ours")
    output, _ = smoke(expect=1)
    assert "was not created by this script" in output
    assert os.path.exists(smoke.cache / "keep.txt")
