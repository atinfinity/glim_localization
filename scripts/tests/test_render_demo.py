import math
import os
import struct
import sys

import numpy as np
import pytest
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import render_demo  # noqa: E402

FRAMES_HEADER = (
    "stamp_ns,result,frame_id,state,reset_count,num_points_raw,num_points_preprocessed,num_imu_fed,"
    "t_load_ms,t_preprocess_ms,t_odometry_ms,inlier_ratio,residual,num_map_factors,num_tiles\n"
)


def write_pcd(path, points, encoding="binary"):
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA {encoding}\n"
    )
    with open(path, "wb") as handle:
        handle.write(header.encode())
        for i, p in enumerate(points):
            if encoding == "ascii":
                handle.write(f"{p[0]} {p[1]} {p[2]} {i}\n".encode())
            else:
                handle.write(struct.pack("<4f", *p, float(i)))


def write_scan(path, stamp_ns, points):
    with open(path, "wb") as handle:
        handle.write(struct.pack("<4sIqIIII", b"GLRP", 1, stamp_ns, len(points), 0, 20, 0))
        for p in points:
            handle.write(struct.pack("<5f", *p, 0.0, 0.0))


@pytest.fixture
def replay(tmp_path):
    """A 1 m square room, 4 frames along x at 10 Hz; the second frame is dropped."""
    data, run = tmp_path / "data", tmp_path / "run"
    (data / "points").mkdir(parents=True)
    run.mkdir()
    walls = [(x, y, 1.0) for x in np.linspace(-1, 3, 21) for y in (-1.0, 1.0)]
    write_pcd(tmp_path / "map.pcd", walls + [(0.0, 0.0, 0.0)])
    stamps = [0, 100_000_000, 200_000_000, 300_000_000]
    rows, traj, gt = [], [], []
    for k, stamp in enumerate(stamps):
        x = 0.5 * k
        write_scan(data / "points" / f"{stamp:019d}.bin", stamp, [(0.0, 1.0, 1.0), (0.0, -1.0, 1.0)])
        gt.append(f"{stamp * 1e-9:.9f} {x} 0 0 0 0 0 1\n")
        result = "dropped" if k == 1 else "estimated"
        state = "INITIALIZING" if k == 0 else "TRACKING"
        rows.append(f"{stamp},{result},{k},{state},0,2,2,3,0.1,0.1,1.0,{0.9 if k else 'nan'},0.01,4,1\n")
        if result == "estimated":
            traj.append(f"{stamp * 1e-9:.9f} {x + 0.01} 0 0 0 0 0 1\n")
    (data / "traj_gt.txt").write_text("".join(gt))
    (run / "traj_localization.txt").write_text("".join(traj))
    (run / "frames.csv").write_text(FRAMES_HEADER + "".join(rows))
    return tmp_path


def args(base, out, *extra):
    return ["--data", str(base / "data"), "--run", str(base / "run"), "--map", str(base / "map.pcd"), "--out", str(out), "--width", "200", *extra]


def test_gif_has_one_frame_per_estimate(replay):
    out = replay / "media" / "demo.gif"
    assert render_demo.main(args(replay, out, "--z-min", "0.5")) == 0
    image = Image.open(out)
    assert image.format == "GIF"
    assert image.n_frames == 3  # the dropped frame is not drawn
    assert image.size[0] == 200


def test_webp_and_stride(replay):
    out = replay / "demo.webp"
    assert render_demo.main(args(replay, out, "--stride", "2", "--no-gt")) == 0
    image = Image.open(out)
    assert image.format == "WEBP"
    assert image.n_frames == 2


def test_pcd_ascii_and_binary_agree(tmp_path):
    points = [(1.5, -2.0, 0.25), (3.0, 4.0, -1.0)]
    write_pcd(tmp_path / "a.pcd", points, "ascii")
    write_pcd(tmp_path / "b.pcd", points, "binary")
    np.testing.assert_allclose(render_demo.read_pcd_xyz(tmp_path / "a.pcd"), points)
    np.testing.assert_allclose(render_demo.read_pcd_xyz(tmp_path / "b.pcd"), points)


def test_scan_goes_to_the_map_frame(tmp_path):
    write_scan(tmp_path / "s.bin", 5, [(1.0, 0.0, 0.0)])
    scan = render_demo.read_scan(tmp_path / "s.bin")
    T_world_imu = render_demo.pose_matrix([2.0, 0.0, 0.0, 0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
    # T_lidar_imu: the LiDAR is 0.5 m ahead of the IMU, so the IMU is at x = -0.5 in the LiDAR frame
    T_imu_lidar = np.linalg.inv(render_demo.pose_matrix([-0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))
    np.testing.assert_allclose(render_demo.transform(T_world_imu @ T_imu_lidar, scan), [[2.0, 1.5, 0.0]], atol=1e-12)


def test_bad_inputs_exit_2(replay, tmp_path):
    assert render_demo.main(args(replay, tmp_path / "demo.png")) == 2
    (replay / "run" / "traj_localization.txt").write_text("0.000000000 0 0 0 0 0 0 1\n")
    assert render_demo.main(args(replay, tmp_path / "demo.gif")) == 2  # estimated frames without a pose
    with open(tmp_path / "bad.bin", "wb") as handle:
        handle.write(b"XXXX" + bytes(28))
    with pytest.raises(render_demo.InputError):
        render_demo.read_scan(tmp_path / "bad.bin")
