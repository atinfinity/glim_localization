"""Tests for scripts/replay_to_bag.py: round trip through bag_to_replay.py and the /initialpose messages."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))
import bag_to_replay  # noqa: E402
import replay_to_bag  # noqa: E402
import test_bag_to_replay as tools  # noqa: E402

from rosbags.rosbag2 import Reader, StoragePlugin  # noqa: E402


def make_dataset(tmp_path, frames=5):
    clouds = []
    for frame in range(frames):
        msg, _, _ = tools.layout("ouster", frame)
        clouds.append((tools.BASE + frame * 100_000_000, msg))
    bag = tmp_path / "src_bag"
    tools.write_bag(bag, clouds, tools.imu_stream(), StoragePlugin.SQLITE3)
    dataset = tmp_path / "dataset"
    assert bag_to_replay.main([str(bag), "--out", str(dataset)]) == 0
    stamps = sorted(int(n[:-4]) for n in os.listdir(dataset / "points"))
    with open(dataset / "traj_gt.txt", "w") as f:
        for i, ns in enumerate(stamps):
            f.write(f"{ns // 10**9}.{ns % 10**9:09d} {i * 0.1} 0 0.5 0 0 0 1\n")
    return dataset, stamps


def read_messages(bag):
    out = {}
    with Reader(bag) as reader:
        for conn, ns, raw in reader.messages():
            out.setdefault(conn.topic, []).append((ns, tools.TS.deserialize_cdr(raw, conn.msgtype)))
    return out


@pytest.mark.parametrize("storage", ["sqlite3", "mcap"])
def test_round_trip(tmp_path, storage):
    dataset, stamps = make_dataset(tmp_path)
    bag = tmp_path / "bag"
    assert replay_to_bag.main([str(dataset), "--out", str(bag), "--storage", storage]) == 0
    messages = read_messages(bag)
    assert sorted(messages) == ["/imu", "/points"]
    assert [ns for ns, _ in messages["/points"]] == stamps

    again = tmp_path / "again"
    assert bag_to_replay.main([str(bag), "--out", str(again)]) == 0
    for name in sorted(os.listdir(dataset / "points")):
        a = tools.read_bin(dataset / "points" / name)
        b = tools.read_bin(again / "points" / name)
        assert a[2] == b[2]
        np.testing.assert_allclose(a[5][:, :4], b[5][:, :4], atol=1e-6)


def test_initial_pose_from_ground_truth(tmp_path):
    dataset, stamps = make_dataset(tmp_path)
    bag = tmp_path / "bag"
    args = [str(dataset), "--out", str(bag), "--initial-pose", "0:0.3,-0.2,0.1,3", "--initial-pose", "0.25"]
    assert replay_to_bag.main(args) == 0
    poses = read_messages(bag)["/initialpose"]
    assert len(poses) == 2

    ns, first = poses[0]
    assert ns == stamps[0] - 1_000_000
    assert first.header.frame_id == "map"
    assert (first.header.stamp.sec * 10**9 + first.header.stamp.nanosec) == ns
    p = first.pose.pose
    assert (p.position.x, p.position.y, p.position.z) == pytest.approx((0.3, -0.2, 0.6))
    assert p.orientation.z == pytest.approx(np.sin(np.radians(1.5)))

    # 0.25 s after the first frame: the first frame at or after that time is frame 3 (stamp + 0.3 s), x = 0.3
    ns, second = poses[1]
    assert ns == stamps[3] - 1_000_000
    assert second.pose.pose.position.x == pytest.approx(0.3)
    assert second.pose.pose.orientation.w == pytest.approx(1.0)


def test_drop_imu(tmp_path):
    dataset, stamps = make_dataset(tmp_path)
    full = tmp_path / "full"
    assert replay_to_bag.main([str(dataset), "--out", str(full)]) == 0
    all_imu = [ns for ns, _ in read_messages(full)["/imu"]]

    bag = tmp_path / "bag"
    assert replay_to_bag.main([str(dataset), "--out", str(bag), "--drop-imu", "0.1:0.2"]) == 0
    messages = read_messages(bag)
    kept = [ns for ns, _ in messages["/imu"]]
    lo, hi = stamps[0] + 100_000_000, stamps[0] + 300_000_000
    assert kept == [ns for ns in all_imu if not lo <= ns < hi]
    assert len(kept) < len(all_imu)
    assert [ns for ns, _ in messages["/points"]] == stamps  # the clouds are all kept

    # The largest gap is the hole
    gaps = np.diff(kept)
    assert gaps.max() >= 200_000_000


def test_drop_imu_bad_arguments(tmp_path, capsys):
    dataset, _ = make_dataset(tmp_path)
    for bad in ("0.1", "-1:0.2", "0.1:0", "a:b"):
        with pytest.raises(SystemExit):
            replay_to_bag.main([str(dataset), "--out", str(tmp_path / "b"), "--drop-imu", bad])
    # A window after the data removes nothing: an error, not a silent full bag
    assert replay_to_bag.main([str(dataset), "--out", str(tmp_path / "b"), "--drop-imu", "100:1"]) == 1
    assert "removed no IMU sample" in capsys.readouterr().err
    assert not (tmp_path / "b").exists()


def test_bad_arguments(tmp_path, capsys):
    dataset, _ = make_dataset(tmp_path)
    bag = tmp_path / "bag"
    assert replay_to_bag.main([str(dataset), "--out", str(bag), "--initial-pose", "100"]) == 1
    assert "no frame at or after" in capsys.readouterr().err
    assert not bag.exists()
    with pytest.raises(SystemExit):
        replay_to_bag.main([str(dataset), "--out", str(tmp_path / "b2"), "--initial-pose", "-1"])
    (tmp_path / "exists").mkdir()
    assert replay_to_bag.main([str(dataset), "--out", str(tmp_path / "exists")]) == 2
