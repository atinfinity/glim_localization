import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import compare_methods  # noqa: E402

MS = 1000000  # ns


def test_latency_pairs_by_stamp():
    clouds = [(100 * MS, 1000 * MS), (200 * MS, 1100 * MS), (300 * MS, 1200 * MS)]
    # The pose of 200 ms is missing; one pose stamp is 0.4 ms off (within the tolerance), one matches no cloud.
    poses = [(100 * MS, 1020 * MS), (300 * MS + 400000, 1250 * MS), (450 * MS, 1300 * MS)]
    latencies, missing, unmatched = compare_methods.latency(clouds, poses, stamp_tol_ns=MS)
    assert latencies == pytest.approx([20.0, 50.0])
    assert missing == 1
    assert unmatched == 1


def test_latency_counts_a_second_pose_of_one_cloud_as_unmatched():
    clouds = [(100 * MS, 1000 * MS)]
    poses = [(100 * MS, 1010 * MS), (100 * MS, 1030 * MS)]
    latencies, missing, unmatched = compare_methods.latency(clouds, poses, stamp_tol_ns=MS)
    assert latencies == pytest.approx([10.0])  # the first received
    assert (missing, unmatched) == (0, 1)


def test_latency_without_poses():
    latencies, missing, unmatched = compare_methods.latency([(1, 2), (3, 4)], [], stamp_tol_ns=MS)
    assert (latencies, missing, unmatched) == ([], 2, 0)


def test_cpu_usage():
    # 1.5 s of CPU (150 ticks at 100 Hz) over 10 s between the first and the last of 101 clouds
    resources = {"clk_tck": 100, "cpu_first": [0, 1000], "cpu_last": [10 * 1000 * MS, 1150]}
    per_scan, percent = compare_methods.cpu_usage(resources, 101)
    assert per_scan == pytest.approx(15.0)
    assert percent == pytest.approx(15.0)
    assert compare_methods.cpu_usage({"clk_tck": 100, "cpu_first": None, "cpu_last": None}, 10) == (None, None)


def test_summarize_and_table():
    runs = [{"latency_median_ms": v, "pose_fraction": 1.0, "ate_trans_rmse_mm": None, "num_clouds": 601}
            for v in (12.0, 10.0, 11.0)]
    summary = compare_methods.summarize(runs)
    assert summary["latency_median_ms"] == {"median": 11.0, "min": 10.0, "max": 12.0, "valid": 3, "runs": 3}
    assert summary["ate_trans_rmse_mm"] == {"valid": 0, "runs": 3}  # no value in any run
    table = compare_methods.format_table({"glim": summary}, {"glim": 3})
    lines = table.splitlines()
    assert lines[0].startswith("| Method | Runs | ATE trans RMSE [mm] |")
    cells = [c.strip() for c in lines[2].strip("|").split("|")]
    header = [c.strip() for c in lines[0].strip("|").split("|")]
    row = dict(zip(header, cells))
    assert row["Method"] == "glim_localization"
    assert row["Runs"] == "3"
    assert row["Latency median [ms]"] == "11.0 (10.0 to 12.0)"
    assert row["Scans with a pose"] == "1.000"  # no range when all runs agree
    assert row["ATE trans RMSE [mm]"] == "n/a [0 of 3 runs]"


def test_summarize_shows_a_failed_run():
    # Run 2 produced no pose, so it has no accuracy; the other two still give a median, and the cell says
    # that one run is missing instead of hiding it.
    runs = [{"ate_trans_rmse_mm": 3.0, "accuracy_error": None},
            {"ate_trans_rmse_mm": None, "accuracy_error": "no poses"},
            {"ate_trans_rmse_mm": 5.0, "accuracy_error": None}]
    summary = compare_methods.summarize(runs)
    assert summary["ate_trans_rmse_mm"] == {"median": 4.0, "min": 3.0, "max": 5.0, "valid": 2, "runs": 3}
    assert "accuracy_error" not in summary  # text, not a metric
    row = compare_methods.format_table({"glim": summary}, {"glim": 3}).splitlines()[2]
    assert "| 4.0 (3.0 to 5.0) [2 of 3 runs] |" in row
    assert compare_methods.cell(None, "{:.1f}") == "n/a"


def write_run(run_dir, offset_m, latency_ms, skip=()):
    """A 10 s straight line at 1 m/s, 10 Hz clouds; poses offset in x by offset_m, some skipped."""
    os.makedirs(run_dir)
    with open(run_dir / "clouds.csv", "w") as clouds, open(run_dir / "poses.csv", "w") as poses, \
            open(run_dir / "traj.txt", "w") as traj:
        clouds.write("stamp_ns,recv_ns\n")
        poses.write("stamp_ns,recv_ns,x,y,z,qx,qy,qz,qw\n")
        for i in range(101):
            stamp = i * 100 * MS
            recv = 5000 * MS + stamp
            clouds.write(f"{stamp},{recv}\n")
            if i in skip:
                continue
            x = 0.1 * i + offset_m
            poses.write(f"{stamp},{recv + int(latency_ms * MS)},{x},0,1,0,0,0,1\n")
            traj.write(f"{stamp * 1e-9:.9f} {x:.9f} 0 1 0 0 0 1\n")
    resources = {"clk_tck": 100, "cpu_first": [5000 * MS, 0], "cpu_last": [15000 * MS, 200],
                 "vm_hwm_kb": 2048, "rss_kb": [[0, 1024], [1, 2000]]}
    with open(run_dir / "resources.json", "w") as handle:
        json.dump(resources, handle)


def test_main_end_to_end(tmp_path):
    with open(tmp_path / "gt.txt", "w") as gt:
        for i in range(1001):  # 100 Hz
            gt.write(f"{i * 0.01:.9f} {0.01 * i:.9f} 0 1 0 0 0 1\n")
    bench = tmp_path / "bench"
    write_run(bench / "glim" / "run1", 0.002, 20.0)
    write_run(bench / "glim" / "run2", 0.004, 30.0)
    write_run(bench / "lidarloc" / "run1", 0.010, 5.0, skip=(50, 51))
    os.makedirs(bench / "bag")  # not a method
    assert compare_methods.main(["--bench", str(bench), "--gt", str(tmp_path / "gt.txt"), "--settle", "2"]) == 0

    report = json.load(open(bench / "benchmark.json"))
    assert list(report["methods"]) == ["glim", "lidarloc"]
    glim = report["methods"]["glim"]
    run1 = glim["runs"]["run1"]
    assert run1["ate_trans_rmse_mm"] == pytest.approx(2.0, abs=1e-3)
    assert run1["num_evaluated"] == 81  # 2 s settle from the first cloud: stamps 2.0 to 10.0 s
    assert run1["rpe_trans_rmse_mm"] == pytest.approx(0.0, abs=1e-3)
    assert run1["latency_median_ms"] == pytest.approx(20.0)
    assert run1["cpu_ms_per_scan"] == pytest.approx(20.0)  # 2 s of CPU over 100 scan periods
    assert run1["cpu_percent"] == pytest.approx(20.0)
    assert run1["peak_rss_mib"] == pytest.approx(2.0)
    assert glim["summary"]["latency_median_ms"] == {"median": 25.0, "min": 20.0, "max": 30.0,
                                                    "valid": 2, "runs": 2}
    lidarloc = report["methods"]["lidarloc"]["runs"]["run1"]
    assert lidarloc["missing_poses"] == 2
    assert lidarloc["pose_fraction"] == pytest.approx(99 / 101)
    assert lidarloc["ate_rot_rmse_deg"] == pytest.approx(0.0, abs=1e-6)
    table = open(bench / "benchmark.md").read()
    assert "| glim_localization | 2 | 3.0 (2.0 to 4.0) |" in table
    assert "lidar_localization_ros2 NDT (IMU, default) | 1 |" in table
    assert not math.isnan(lidarloc["ate_trans_rmse_mm"])


def test_svg(tmp_path):
    summary = {"latency_median_ms": {"median": 10.0, "min": 9.0, "max": 11.0},
               "latency_p95_ms": {"median": 12.0, "min": 12.0, "max": 12.0},
               "ate_trans_rmse_mm": {"median": 3.0, "min": 3.0, "max": 3.5}}
    path = tmp_path / "chart.svg"
    compare_methods.plot_svg({"glim": summary, "lidarloc": summary}, str(path))
    text = path.read_text()
    assert text.startswith("<svg ") and text.rstrip().endswith("</svg>")
    assert "10.0 (p95 12.0)" in text and "glim_localization" in text
