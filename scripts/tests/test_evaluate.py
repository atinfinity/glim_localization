import json
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import evaluate  # noqa: E402


def quat_z(angle_rad):
    return (0.0, 0.0, math.sin(angle_rad / 2), math.cos(angle_rad / 2))  # x y z w


def write_tum(path, stamps, positions, quats):
    with open(path, "w") as handle:
        for t, p, q in zip(stamps, positions, quats):
            handle.write(f"{t:.9f} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")


@pytest.fixture
def straight_line(tmp_path):
    """A 20 m straight line at 2 m/s, 10 Hz, 10 s."""
    stamps = np.arange(0, 100) * 0.1
    gt_positions = np.stack([2.0 * stamps, np.zeros_like(stamps), np.ones_like(stamps)], axis=1)
    gt_quats = [quat_z(0.0)] * len(stamps)
    write_tum(tmp_path / "gt.txt", stamps, gt_positions, gt_quats)
    return tmp_path, stamps, gt_positions, gt_quats


def run(tmp_path, *extra, est="est.txt", frames=None):
    argv = ["--est", str(tmp_path / est), "--gt", str(tmp_path / "gt.txt"), "--out", str(tmp_path / "eval"), *extra]
    if frames:
        argv += ["--frames", str(frames)]
    return evaluate.main(argv)


def test_constant_offset_gives_known_ate(straight_line):
    tmp_path, stamps, gt_positions, _ = straight_line
    offset = np.array([0.03, 0.04, 0.0])  # 0.05 m
    write_tum(tmp_path / "est.txt", stamps, gt_positions + offset, [quat_z(math.radians(0.5))] * len(stamps))

    assert run(tmp_path, "--settle", "1") == 0
    report = json.load(open(tmp_path / "eval" / "eval.json"))
    assert report["errors"]["ate_trans_m"]["max"] == pytest.approx(0.05, abs=1e-6)
    assert report["errors"]["ate_trans_m"]["rmse"] == pytest.approx(0.05, abs=1e-6)
    assert report["errors"]["ate_rot_deg"]["max"] == pytest.approx(0.5, abs=1e-4)
    # A constant offset does not change relative rotation. The constant yaw offset turns each
    # relative translation by 0.5 deg, so RPE translation is 2 sin(0.25 deg) per meter travelled
    # (evo may pair poses a little more than 1 m apart).
    assert report["errors"]["rpe_rot_deg"]["max"] == pytest.approx(0.0, abs=1e-4)
    assert report["errors"]["rpe_trans_m"]["mean"] == pytest.approx(2 * math.sin(math.radians(0.25)), rel=0.02)
    assert report["trajectory"]["num_matched"] == 100
    assert report["trajectory"]["num_evaluated"] == 90  # 10 poses within the 1 s settle time
    assert (tmp_path / "eval" / "eval.txt").read_text().rstrip().endswith("PASS")


def test_scale_error_gives_known_rpe(straight_line):
    tmp_path, stamps, gt_positions, gt_quats = straight_line
    scaled = gt_positions * np.array([1.01, 1.0, 1.0])
    write_tum(tmp_path / "est.txt", stamps, scaled, gt_quats)

    assert run(tmp_path) == 0
    report = json.load(open(tmp_path / "eval" / "eval.json"))
    # Every 1 m of ground truth motion is 1.01 m in the estimate
    assert report["errors"]["rpe_trans_m"]["mean"] == pytest.approx(0.01, abs=2e-4)
    assert report["errors"]["ate_trans_m"]["max"] == pytest.approx(0.01 * 2.0 * 9.9, abs=1e-6)


def test_thresholds_set_exit_code(straight_line):
    tmp_path, stamps, gt_positions, _ = straight_line
    write_tum(tmp_path / "est.txt", stamps, gt_positions + np.array([0.0, 0.2, 0.0]), [quat_z(math.radians(2.0))] * len(stamps))

    assert run(tmp_path, "--max-trans", "0.3", "--max-rot-deg", "3") == 0
    assert run(tmp_path, "--max-trans", "0.1", "--max-rot-deg", "3") == 1
    assert run(tmp_path, "--max-trans", "0.3", "--max-rot-deg", "1") == 1
    report = json.load(open(tmp_path / "eval" / "eval.json"))
    assert len(report["failures"]) == 1
    assert "rotation" in report["failures"][0]


def test_unaligned_estimate_is_not_realigned(straight_line):
    tmp_path, stamps, gt_positions, gt_quats = straight_line
    write_tum(tmp_path / "est.txt", stamps, gt_positions + np.array([5.0, 0.0, 0.0]), gt_quats)
    assert run(tmp_path, "--max-trans", "1") == 1


def test_pairs_only_within_tolerance(straight_line):
    tmp_path, stamps, gt_positions, gt_quats = straight_line
    # Estimates exist only for every other frame and are 5 ms late (inside the 10 ms tolerance)
    write_tum(tmp_path / "est.txt", stamps[::2] + 0.005, gt_positions[::2], gt_quats[::2])
    # Half of the frames have no estimate: the coverage check fails unless it is relaxed
    frames = tmp_path / "frames.csv"
    frames_csv(frames, stamps, ["estimated" if i % 2 == 0 else "dropped" for i in range(100)])
    assert run(tmp_path, frames=frames) == 1
    assert run(tmp_path, "--min-coverage", "0.5", frames=frames) == 0
    assert json.load(open(tmp_path / "eval" / "eval.json"))["trajectory"]["num_matched"] == 50

    # 20 ms late is outside it
    write_tum(tmp_path / "est.txt", stamps + 0.02, gt_positions, gt_quats)
    assert run(tmp_path) == 2


def test_bad_inputs_exit_with_2(straight_line):
    tmp_path, stamps, gt_positions, gt_quats = straight_line
    assert run(tmp_path, est="missing.txt") == 2
    (tmp_path / "empty.txt").write_text("")
    assert run(tmp_path, est="empty.txt") == 2
    write_tum(tmp_path / "est.txt", stamps[:1], gt_positions[:1], gt_quats[:1])
    assert run(tmp_path, "--settle", "0") == 2  # one pose is not enough
    write_tum(tmp_path / "est.txt", stamps, gt_positions, gt_quats)
    assert run(tmp_path, "--settle", "50") == 2


def test_processing_time_statistics(straight_line):
    tmp_path, stamps, gt_positions, gt_quats = straight_line
    write_tum(tmp_path / "est.txt", stamps, gt_positions, gt_quats)
    frames = tmp_path / "frames.csv"
    (tmp_path / "summary.json").write_text(json.dumps({"num_frames": 100}))
    with open(frames, "w") as handle:
        handle.write(
            "stamp_ns,result,frame_id,state,reset_count,num_points_raw,num_points_preprocessed,num_imu_fed,"
            "t_load_ms,t_preprocess_ms,t_odometry_ms,inlier_ratio,residual,num_map_factors,num_tiles\n"
        )
        for i, t in enumerate(stamps):
            slow = i % 10 == 0  # 10 % of the frames take 150 ms, the others 50 ms in total
            odometry = 140.0 if slow else 40.0
            result = "dropped" if i == 5 else "estimated"
            handle.write(f"{round(t * 1e9)},{result},{i},TRACKING,0,100,50,10,5.0,5.0,{odometry},0.9,0.1,10,3\n")

    assert run(tmp_path, frames=frames) == 0
    timing = json.load(open(tmp_path / "eval" / "eval.json"))["timing"]
    assert timing["frames_by_result"] == {"estimated": 99, "dropped": 1}
    assert timing["frame_period_s"] == pytest.approx(0.1)
    assert timing["data_duration_s"] == pytest.approx(9.9)
    assert timing["overrun_ratio"] == pytest.approx(0.1)  # 150 ms frames only
    assert timing["total_ms"]["max"] == pytest.approx(150.0)
    assert timing["total_ms"]["median"] == pytest.approx(50.0)
    # (10 * 150 + 90 * 50) ms over 9.9 s
    assert timing["realtime_factor"] == pytest.approx(6.0 / 9.9)
    assert timing["odometry_ms"]["median"] == pytest.approx(40.0)
    assert "processing time / data length" in (tmp_path / "eval" / "eval.txt").read_text()


def test_plot_option_writes_png(straight_line):
    tmp_path, stamps, gt_positions, gt_quats = straight_line
    write_tum(tmp_path / "est.txt", stamps, gt_positions + 0.01, gt_quats)
    assert run(tmp_path, "--plot") == 0
    assert (tmp_path / "eval" / "eval.png").stat().st_size > 0


def test_low_coverage_fails(straight_line):
    tmp_path, stamps, gt_positions, gt_quats = straight_line
    frames = tmp_path / "frames.csv"
    # 3 estimated poses of 100 frames
    write_tum(tmp_path / "est.txt", stamps[:3], gt_positions[:3], gt_quats[:3])
    frames_csv(frames, stamps, ["estimated"] * 3 + ["dropped"] * 97)
    assert run(tmp_path, frames=frames) == 1
    report = json.load(open(tmp_path / "eval" / "eval.json"))
    assert report["trajectory"]["coverage"] == pytest.approx(0.03)
    assert "coverage" in report["failures"][0]

    # A run that died halfway: the frames after the failure are `diverged`/`dropped`
    write_tum(tmp_path / "est.txt", stamps[:50], gt_positions[:50], gt_quats[:50])
    frames_csv(frames, stamps, ["estimated"] * 50 + ["diverged"] * 50)
    assert run(tmp_path, frames=frames) == 1


def test_leading_skipped_frames_count_as_missing(straight_line):
    tmp_path, stamps, gt_positions, gt_quats = straight_line
    frames = tmp_path / "frames.csv"
    write_tum(tmp_path / "est.txt", stamps[90:], gt_positions[90:], gt_quats[90:])
    frames_csv(frames, stamps, ["skipped_no_imu"] * 90 + ["estimated"] * 10)
    assert run(tmp_path, frames=frames) == 1
    assert json.load(open(tmp_path / "eval" / "eval.json"))["trajectory"]["coverage"] == pytest.approx(0.1)


def test_leading_dropped_frames_are_excluded(straight_line):
    # Frames before --init-stamp are `dropped` by design and must not count as missing
    tmp_path, stamps, gt_positions, gt_quats = straight_line
    frames = tmp_path / "frames.csv"
    write_tum(tmp_path / "est.txt", stamps[50:], gt_positions[50:], gt_quats[50:])
    frames_csv(frames, stamps, ["dropped"] * 50 + ["estimated"] * 50)
    assert run(tmp_path, frames=frames) == 0
    assert json.load(open(tmp_path / "eval" / "eval.json"))["trajectory"]["coverage"] == pytest.approx(1.0)
    # Nothing was estimated at all
    frames_csv(frames, stamps, ["dropped"] * 100)
    assert run(tmp_path, frames=frames) == 2


def test_pose_on_the_window_boundary_is_evaluated(straight_line):
    # The first frame is estimated at 0 s with --settle 3: the pose at exactly 3.0 s starts the window.
    # The bound for the evaluated poses and for the expected frames is the same, so coverage stays 1.0.
    tmp_path, stamps, gt_positions, gt_quats = straight_line
    frames = tmp_path / "frames.csv"
    write_tum(tmp_path / "est.txt", stamps, gt_positions, gt_quats)
    frames_csv(frames, stamps, ["estimated"] * 100)
    for settle in ("3.0", "2.9999999999", "3.0000000004"):
        assert run(tmp_path, "--settle", settle, frames=frames) == 0, settle
        report = json.load(open(tmp_path / "eval" / "eval.json"))["trajectory"]
        assert report["coverage"] == pytest.approx(1.0), settle
        assert report["num_unmatched_estimated"] == 0


def test_incomplete_run_is_rejected(straight_line):
    tmp_path, stamps, gt_positions, gt_quats = straight_line
    frames = tmp_path / "frames.csv"
    write_tum(tmp_path / "est.txt", stamps[:50], gt_positions[:50], gt_quats[:50])
    # An aborted run: frames.csv is cut short and there is no summary.json
    frames_csv(frames, stamps[:50], ["estimated"] * 50, summary=False)
    assert run(tmp_path, frames=frames) == 2
    # A summary of a longer run does not match a truncated frames.csv
    (tmp_path / "summary.json").write_text(json.dumps({"num_frames": 100}))
    assert run(tmp_path, frames=frames) == 2
    # The aborted run can still be inspected explicitly
    assert run(tmp_path, "--allow-incomplete", frames=frames) == 0
    report = json.load(open(tmp_path / "eval" / "eval.json"))
    assert report["incomplete"] is True
    assert report["trajectory"]["num_evaluated"] == 50
    assert "incomplete" in (tmp_path / "eval" / "eval.txt").read_text()


def test_tracking_ratio_shortfall_is_rejected_like_an_aborted_run(straight_line):
    tmp_path, stamps, gt_positions, gt_quats = straight_line
    frames = tmp_path / "frames.csv"
    write_tum(tmp_path / "est.txt", stamps, gt_positions, gt_quats)
    frames_csv(frames, stamps, ["estimated"] * 100)

    def summary(ratio, minimum):
        (tmp_path / "summary.json").write_text(json.dumps({"num_frames": 100, "tracking_ratio": ratio, "min_tracking_ratio": minimum}))

    summary(0.9, 0.9)  # At the gate
    assert run(tmp_path, frames=frames) == 0
    summary(0.89, 0.9)  # The replay exited 1 on this
    assert run(tmp_path, frames=frames) == 2
    summary(0.0, 0.0)  # The gate was disabled
    assert run(tmp_path, frames=frames) == 0
    summary(0.89, 0.9)
    assert run(tmp_path, "--allow-incomplete", frames=frames) == 0


def test_without_frames_coverage_is_not_checked(tmp_path):
    # A 10 Hz estimate against a 100 Hz ground truth, and a run that was limited with --end
    stamps = np.arange(0, 1000) * 0.01
    positions = np.stack([2.0 * stamps, np.zeros_like(stamps), np.ones_like(stamps)], axis=1)
    quats = [quat_z(0.0)] * len(stamps)
    write_tum(tmp_path / "gt.txt", stamps, positions, quats)
    write_tum(tmp_path / "est.txt", stamps[::10], positions[::10], quats[::10])
    assert run(tmp_path) == 0
    report = json.load(open(tmp_path / "eval" / "eval.json"))
    assert report["trajectory"]["coverage"] is None
    assert report["trajectory"]["expected_source"] == "none"
    assert "coverage: not checked, pass --frames" in (tmp_path / "eval" / "eval.txt").read_text()
    write_tum(tmp_path / "est.txt", stamps[:600], positions[:600], quats[:600])  # --end 6
    assert run(tmp_path) == 0


def test_unmatched_estimates_are_reported(straight_line):
    tmp_path, stamps, gt_positions, gt_quats = straight_line
    # Every second estimate is 50 ms off the ground truth stamps
    est_stamps = np.where(np.arange(100) % 2 == 0, stamps, stamps + 0.05)
    write_tum(tmp_path / "est.txt", est_stamps, gt_positions, gt_quats)
    assert run(tmp_path, "--min-coverage", "0.55") == 1
    report = json.load(open(tmp_path / "eval" / "eval.json"))
    assert report["trajectory"]["num_unmatched_estimated"] == 50
    assert any("no ground truth" in f for f in report["failures"])


def frames_csv(path, stamps, results, summary=True):
    if summary:  # A complete run: the tool writes summary.json next to frames.csv
        (path.parent / "summary.json").write_text(json.dumps({"num_frames": len(stamps)}))
    with open(path, "w") as handle:
        handle.write("stamp_ns,result,t_load_ms,t_preprocess_ms,t_odometry_ms\n")
        for t, result in zip(stamps, results):
            handle.write(f"{round(t * 1e9)},{result},1.0,1.0,5.0\n")


def test_frames_csv_counts_dropped_frames_as_missing(straight_line):
    tmp_path, stamps, gt_positions, gt_quats = straight_line
    # The ground truth is much longer than the replayed part: 40 frames, 10 of them not estimated
    keep = np.arange(40) % 4 != 3
    write_tum(tmp_path / "est.txt", stamps[:40][keep], gt_positions[:40][keep], [gt_quats[0]] * int(keep.sum()))
    frames_csv(tmp_path / "frames.csv", stamps[:40], ["estimated" if k else "dropped" for k in keep])
    assert run(tmp_path, frames=tmp_path / "frames.csv") == 1
    report = json.load(open(tmp_path / "eval" / "eval.json"))
    assert report["trajectory"]["expected_source"] == "frames.csv"
    assert report["trajectory"]["num_expected_in_window"] == 40
    assert report["trajectory"]["coverage"] == pytest.approx(0.75)
    # Without frames.csv there is no reference for the number of poses, so only the matching is checked
    assert run(tmp_path) == 0
    # The same estimate passes when the dropped frames are accepted
    assert run(tmp_path, "--min-coverage", "0.7", frames=tmp_path / "frames.csv") == 0


def test_malformed_frames_and_nan_pose_exit_2(straight_line):
    tmp_path, stamps, gt_positions, gt_quats = straight_line
    write_tum(tmp_path / "est.txt", stamps, gt_positions, gt_quats)
    (tmp_path / "frames.csv").write_text("stamp_ns,result\n1,estimated\n")
    assert run(tmp_path, frames=tmp_path / "frames.csv") == 2
    (tmp_path / "frames.csv").write_text("stamp_ns,result,t_load_ms,t_preprocess_ms,t_odometry_ms\nabc,estimated,1,1,1\n")
    assert run(tmp_path, frames=tmp_path / "frames.csv") == 2
    (tmp_path / "frames.csv").write_text("stamp_ns,result,t_load_ms,t_preprocess_ms,t_odometry_ms\n")
    assert run(tmp_path, frames=tmp_path / "frames.csv") == 2

    bad = gt_positions.copy()
    bad[50, 0] = float("nan")
    write_tum(tmp_path / "est.txt", stamps, bad, gt_quats)
    assert run(tmp_path) == 2
    quats = list(gt_quats)
    quats[10] = (0.0, 0.0, float("nan"), 1.0)
    write_tum(tmp_path / "est.txt", stamps, gt_positions, quats)
    assert run(tmp_path) == 2
