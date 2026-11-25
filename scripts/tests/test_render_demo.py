import json
import math
import os
import struct
import sys

import numpy as np
import pytest
from PIL import Image, ImageColor, ImageSequence

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
    return ["--data", str(base / "data"), "--run", str(base / "run"), "--map", str(base / "map.pcd"), "--out", str(out), "--width", "200", "--scale", "1", "--font", "DejaVu Sans", *extra]


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


def test_z_slice_keeps_only_the_band(replay):
    points = np.array([[0.0, 0.0, 0.5], [1.0, 0.0, 1.0], [2.0, 0.0, 2.9], [3.0, 0.0, 3.0]])
    np.testing.assert_array_equal(render_demo.z_filter(points, 1.0, 3.0), points[1:3])
    np.testing.assert_array_equal(render_demo.z_filter(points, None, 3.0), points[:3])
    # The walls of the fixture are at z = 1, so a slice from 1 to 3 still draws them
    assert render_demo.main(args(replay, replay / "slice.gif", "--z-min", "1.0", "--z-max", "3.0")) == 0
    assert Image.open(replay / "slice.gif").n_frames == 3
    # A slice without map points is an input error
    assert render_demo.main(args(replay, replay / "empty.gif", "--z-min", "5", "--z-max", "6")) == 2
    with pytest.raises(SystemExit):
        render_demo.parse_args(args(replay, replay / "bad.gif", "--z-min", "3", "--z-max", "1"))


@pytest.mark.parametrize("encoding", ["ascii", "binary"])
def test_non_finite_points_are_dropped(tmp_path, encoding):
    points = [(1.0, 2.0, 3.0), (math.nan, 0.0, 0.0), (0.0, math.inf, 0.0), (0.0, 0.0, -math.inf), (4.0, 5.0, 6.0)]
    write_pcd(tmp_path / "m.pcd", points, encoding)
    np.testing.assert_array_equal(render_demo.read_pcd_xyz(tmp_path / "m.pcd"), [points[0], points[4]])
    write_scan(tmp_path / "s.bin", 0, points)
    np.testing.assert_array_equal(render_demo.read_scan(tmp_path / "s.bin"), [points[0], points[4]])


def test_bad_frames_and_unwritable_output_exit_2(replay):
    frames = replay / "run" / "frames.csv"
    good = frames.read_text()
    lines = good.splitlines(keepends=True)
    frames.write_text(lines[0] + "0.5" + lines[1][1:] + "".join(lines[2:]))  # a non-integer stamp
    assert render_demo.main(args(replay, replay / "a.gif")) == 2
    frames.write_text(lines[0] + "0,estimated,0\n")  # a short row
    assert render_demo.main(args(replay, replay / "b.gif")) == 2
    frames.write_text(good)
    (replay / "not_a_dir").write_text("")
    assert render_demo.main(args(replay, replay / "not_a_dir" / "c.gif")) == 2


def test_missing_font_falls_back_with_a_warning(replay, capsys):
    out = replay / "font.gif"
    assert render_demo.main(args(replay, out, "--font", "No Such Font Family 25", "--scale", "1")) == 0
    assert "warning: font 'No Such Font Family 25' not found, using DejaVu Sans" in capsys.readouterr().err
    assert Image.open(out).size[0] == 200


def pose_pixels(path):
    """Pixels of the pose color (dot and heading arrow) in the first frame."""
    rgb = np.asarray(Image.open(path).convert("RGB"))
    pose = tuple(int(render_demo.POSE_COLOR[k:k + 2], 16) for k in (1, 3, 5))
    return int(np.all(rgb == pose, axis=2).sum())


def test_scale_and_palette_keep_drawing_colors(replay, monkeypatch):
    palettes = []
    gif_palette = render_demo.gif_palette

    def capture(images, colors):
        palette = gif_palette(images, colors)
        palettes.append(palette.getpalette())
        return palette

    monkeypatch.setattr(render_demo, "gif_palette", capture)
    out = replay / "scaled.gif"
    assert render_demo.main(args(replay, out, "--scale", "2")) == 0
    image = Image.open(out)
    assert image.size[0] == 400
    # Every pixel of every frame has a color of the palette built for it (no new color from saving).
    entries = {tuple(palettes[-1][k:k + 3]) for k in range(0, len(palettes[-1]), 3)}
    for frame in ImageSequence.Iterator(image):
        drawn = {color for _, color in frame.convert("RGB").getcolors(1 << 20)}
        assert drawn <= entries, drawn - entries
    assert pose_pixels(out) > 0
    # --scale scales the drawing, not only the canvas: the pose is about twice as wide at 2x (its pixel
    # count, about 4 times; exact-color pixels only, so antialiased edges make the 1x count a bit low).
    single = replay / "single.gif"
    assert render_demo.main(args(replay, single)) == 0
    assert 1.6 < math.sqrt(pose_pixels(out) / pose_pixels(single)) < 3.0


def test_palette_starts_with_the_exact_drawing_colors():
    # Gray frames, so the drawing colors come from the fixed entries, not from the samples.
    images = [Image.new("RGB", (8, 8), "#808080") for _ in range(3)]
    palette = render_demo.gif_palette(images, 256).getpalette()
    fixed = render_demo.fixed_colors()
    assert {"#000000", "#ffffff", render_demo.POSE_COLOR, render_demo.POSE_EDGE_COLOR, render_demo.SCAN_COLOR, render_demo.ESTIMATE_COLOR} <= set(fixed)
    assert set(render_demo.STATE_COLORS.values()) <= set(fixed)
    expected = [int(color[k:k + 2], 16) for color in fixed for k in (1, 3, 5)]
    assert palette[: len(expected)] == expected
    assert len(palette) == 768


def test_points_behind_the_camera_are_culled():
    R = render_demo.look_at([0.0, 0.0, 0.0], [1.0, 0.0, 0.0])
    camera = render_demo.Camera(R, [0.0, 0.0, 0.0], 100.0, [50.0, 40.0], near=0.3)
    uv, depth = camera.project([[5.0, 0.0, 0.0], [-5.0, 0.0, 0.0], [0.1, 0.0, 0.0], [5.0, -1.0, 1.0]])
    np.testing.assert_allclose(depth, [5.0, -5.0, 0.1, 5.0])
    np.testing.assert_allclose(uv[0], [50.0, 40.0])  # on the axis: the principal point
    assert np.isnan(uv[1]).all() and np.isnan(uv[2]).all()  # behind the camera, and in front of the near plane
    # -y in the world is to the right of a camera looking along +x, and +z is up (smaller v)
    np.testing.assert_allclose(uv[3], [50.0 + 20.0, 40.0 - 20.0])


def test_look_at_is_a_rotation_and_rejects_a_vertical_view():
    R = render_demo.look_at([3.0, -4.0, 5.0], [0.0, 0.0, 0.0])
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(R) == pytest.approx(1.0)
    np.testing.assert_allclose(R[2], -np.array([3.0, -4.0, 5.0]) / np.linalg.norm([3.0, -4.0, 5.0]))
    assert R[1, 2] < 0  # image down is world down
    with pytest.raises(ValueError):
        render_demo.look_at([0.0, 0.0, 5.0], [0.0, 0.0, 0.0])


def test_painter_order_draws_far_points_first():
    depth = np.array([2.0, 9.0, 5.0, 9.0, 1.0])
    np.testing.assert_array_equal(render_demo.painter_order(depth), [1, 3, 2, 0, 4])  # equal depths keep their order


def test_fit_camera_puts_the_corners_inside_the_margin():
    corners = np.array([[x, y, z] for x in (0.0, 20.0) for y in (-3.0, 3.0) for z in (0.0, 3.0)])
    eye = np.array([10.0, -25.0, 20.0])
    R = render_demo.look_at(eye, corners.mean(axis=0))
    focal, center = render_demo.fit_camera(R, eye, corners, 400, 200, 10)
    uv, _ = render_demo.Camera(R, eye, focal, center).project(corners)
    assert uv[:, 0].min() >= 10 - 1e-6 and uv[:, 0].max() <= 390 + 1e-6
    assert uv[:, 1].min() >= 10 - 1e-6 and uv[:, 1].max() <= 190 + 1e-6
    # It fills one of the two directions
    assert min(uv[:, 0].min(), uv[:, 1].min()) == pytest.approx(10.0)


def test_heights_and_support():
    np.testing.assert_array_equal(render_demo.height_levels([-1.0, 0.0, 1.49, 1.5, 3.0, 9.0], 0.0, 3.0), [0, 0, 7, 8, 15, 15])
    floor = [(x, y, 0.0) for x in np.arange(0.0, 4.0, 0.2) for y in np.arange(-1.0, 1.0, 0.2)]
    box_top = [(x, y, 0.6) for x in np.arange(2.0, 3.0, 0.2) for y in np.arange(-0.4, 0.4, 0.2)]
    positions = np.array([[1.0, 0.0, 1.0], [2.4, 0.0, 1.0], [2.4, 0.0, 0.5], [9.0, 0.0, 1.0]])
    # Over the floor, over the box top, below the box top, and outside the map (the lowest height)
    np.testing.assert_allclose(render_demo.support_heights(np.array(floor + box_top), positions), [0.0, 0.6, 0.0, 0.0])


def test_palette_has_the_exact_height_and_intro_colors():
    fixed = render_demo.fixed_colors()
    assert len(render_demo.HEIGHT_COLORS) == render_demo.HEIGHT_LEVELS
    assert set(render_demo.HEIGHT_COLORS) <= set(fixed) and render_demo.INIT_GUESS_COLOR in fixed
    assert len(fixed) <= 256
    palette = render_demo.gif_palette([Image.new("RGB", (8, 8), color) for color in ("#808080", "#123456")], 256)
    # A drawing color maps to its own entry exactly, without dithering
    for color in (render_demo.HEIGHT_COLORS[0], render_demo.HEIGHT_COLORS[-1], render_demo.INIT_GUESS_COLOR, render_demo.POSE_COLOR):
        image = Image.new("RGB", (4, 4), color)
        assert render_demo.to_palette(image, palette).convert("RGB").getpixel((1, 1)) == ImageColor.getrgb(color)


@pytest.mark.parametrize("view", ["oblique", "chase", "top"])
def test_every_view_renders(replay, view):
    out = replay / f"{view}.gif"
    assert render_demo.main(args(replay, out, "--view", view)) == 0
    image = Image.open(out)
    assert image.n_frames == 3
    assert image.size[0] == 200
    assert pose_pixels(out) > 0


def test_bad_view_options_exit_2(replay):
    for extra in (["--view", "side"], ["--aspect", "5"], ["--elevation", "90"], ["--distance", "0"], ["--intro-frames", "0"], ["--margin", "-1"]):
        with pytest.raises(SystemExit) as error:
            render_demo.parse_args(args(replay, replay / "bad.gif", *extra))
        assert error.value.code == 2, extra


def write_init_search(run, search):
    pose = [0.0, 0.0, 0.0, 0.0, 0.0, math.sin(math.pi / 6), math.cos(math.pi / 6)]  # yaw 60 deg
    (run / "replay_args.json").write_text(json.dumps({"effective": {"init_T_world_imu_xyz_qxyzw": pose}}))
    (run / "summary.json").write_text(json.dumps({"init_search": search}))


def test_init_search_intro(replay):
    run = replay / "run"
    write_init_search(run, {"result": "ACCEPTED", "yaw_offset_deg": -60.0, "score": 0.97, "candidates": 24, "seconds": 0.5})
    T_given, search = render_demo.read_init_search(str(run))
    np.testing.assert_allclose(T_given[:3, 0], [0.5, math.sqrt(3) / 2, 0.0], atol=1e-12)
    assert search["yaw_offset_deg"] == -60.0 and search["candidates"] == 24
    out = replay / "intro.gif"
    assert render_demo.main(args(replay, out, "--init-search", "--intro-frames", "4")) == 0
    # The two held intro images (identical frames are merged into one longer frame), then the 3 estimates
    durations = [frame.info["duration"] for frame in ImageSequence.Iterator(Image.open(out))]
    assert durations[:2] == [400, 400] and len(durations) == 5


def test_init_search_without_an_accepted_search_exits_2(replay):
    run = replay / "run"
    assert render_demo.main(args(replay, replay / "a.gif", "--init-search")) == 2  # no replay_args.json, no summary.json
    write_init_search(run, {"result": "LOW_SCORE", "yaw_offset_deg": 0.0, "score": 0.2, "candidates": 24, "seconds": 0.5})
    assert render_demo.main(args(replay, replay / "b.gif", "--init-search")) == 2
    write_init_search(run, {"result": "ACCEPTED", "score": 0.9})
    assert render_demo.main(args(replay, replay / "c.gif", "--init-search")) == 2
