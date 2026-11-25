import json
import os
import struct
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import bag_to_replay  # noqa: E402

rosbags = pytest.importorskip("rosbags")
from rosbags.rosbag2 import StoragePlugin, Writer  # noqa: E402
from rosbags.typesys import Stores, get_typestore  # noqa: E402

TS = get_typestore(Stores.ROS2_JAZZY)
Time = TS.types["builtin_interfaces/msg/Time"]
Header = TS.types["std_msgs/msg/Header"]
Field = TS.types["sensor_msgs/msg/PointField"]
Cloud = TS.types["sensor_msgs/msg/PointCloud2"]
Imu = TS.types["sensor_msgs/msg/Imu"]
Vec3 = TS.types["geometry_msgs/msg/Vector3"]
Quat = TS.types["geometry_msgs/msg/Quaternion"]

NP = {Field.UINT32: "<u4", Field.FLOAT32: "<f4", Field.FLOAT64: "<f8"}
NUM_POINTS = 50


def make_time(ns):
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def make_cloud(stamp_ns, time_values, time_name, time_type, xyz=None):
    if xyz is None:
        rng = np.random.default_rng(int(stamp_ns % 1000))
        xyz = rng.uniform(-5, 5, size=(NUM_POINTS, 3)).astype("<f4")
    fields = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")]
    flist = [
        Field(name="x", offset=0, datatype=Field.FLOAT32, count=1),
        Field(name="y", offset=4, datatype=Field.FLOAT32, count=1),
        Field(name="z", offset=8, datatype=Field.FLOAT32, count=1),
        Field(name="intensity", offset=12, datatype=Field.FLOAT32, count=1),
    ]
    if time_name is not None:
        fields.append((time_name, NP[time_type]))
        flist.append(Field(name=time_name, offset=16, datatype=time_type, count=1))
    arr = np.zeros(len(xyz), dtype=np.dtype(fields))
    arr["x"], arr["y"], arr["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    arr["intensity"] = 7.0
    if time_name is not None:
        arr[time_name] = time_values
    return Cloud(
        header=Header(stamp=make_time(stamp_ns), frame_id="lidar"),
        height=1,
        width=len(xyz),
        fields=flist,
        is_bigendian=False,
        point_step=arr.dtype.itemsize,
        row_step=arr.dtype.itemsize * len(xyz),
        data=np.frombuffer(arr.tobytes(), dtype=np.uint8),
        is_dense=True,
    )


def make_imu(stamp_ns, acc=(0.0, 0.0, 9.8), gyro=(0.0, 0.0, 0.1)):
    return Imu(
        header=Header(stamp=make_time(stamp_ns), frame_id="imu"),
        orientation=Quat(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=np.zeros(9),
        angular_velocity=Vec3(x=gyro[0], y=gyro[1], z=gyro[2]),
        angular_velocity_covariance=np.zeros(9),
        linear_acceleration=Vec3(x=acc[0], y=acc[1], z=acc[2]),
        linear_acceleration_covariance=np.zeros(9),
    )


def write_bag(path, clouds, imus, plugin, extra=()):
    """Write /points and /imu. `extra` is a list of (topic, msgtype, record ns, message), also used by scripts/replay_to_bag.py."""
    with Writer(path, version=9, storage_plugin=plugin) as writer:
        cc = writer.add_connection("/points", "sensor_msgs/msg/PointCloud2", typestore=TS)
        ic = writer.add_connection("/imu", "sensor_msgs/msg/Imu", typestore=TS)
        for ns, msg in clouds:
            writer.write(cc, ns, TS.serialize_cdr(msg, "sensor_msgs/msg/PointCloud2"))
        for ns, msg in imus:
            writer.write(ic, ns, TS.serialize_cdr(msg, "sensor_msgs/msg/Imu"))
        connections = {}
        for topic, msgtype, ns, msg in extra:
            if (topic, msgtype) not in connections:
                connections[(topic, msgtype)] = writer.add_connection(topic, msgtype, typestore=TS)
            writer.write(connections[(topic, msgtype)], ns, TS.serialize_cdr(msg, msgtype))


BASE = 1_700_000_000_000_000_000  # an absolute wall clock stamp in ns
SEC = 1_700_000_000


def imu_stream(n=200):
    return [(BASE + i * 5_000_000, make_imu(BASE + i * 5_000_000)) for i in range(n)]


def layout(name, frame):
    """Return (message, expected first point stamp ns, expected max t) for frame 0/1 of one sensor layout."""
    base = BASE + frame * 100_000_000
    ramp = np.linspace(0.0, 0.09, NUM_POINTS)
    if name == "ouster":  # UINT32 ns relative to the frame stamp
        return make_cloud(base, (ramp * 1e9).astype("<u4"), "t", Field.UINT32), base, 0.09
    if name == "velodyne":  # FLOAT32 s relative, centred on the frame stamp (negative values)
        rel = (ramp - 0.045).astype("<f4")
        return make_cloud(base, rel, "time", Field.FLOAT32), base - 45_000_000, 0.09
    if name == "hesai":  # FLOAT64 s absolute
        absolute = SEC + frame * 0.1 + ramp
        return make_cloud(base + 12_345, absolute, "timestamp", Field.FLOAT64), int(round((SEC + frame * 0.1) * 1e9)), 0.09
    if name == "livox":  # FLOAT64 ns absolute
        absolute = (SEC + frame * 0.1 + ramp) * 1e9
        return make_cloud(base, absolute, "timestamp", Field.FLOAT64), int(round((SEC + frame * 0.1) * 1e9)), 0.09
    return make_cloud(base, None, None, None), base, 0.0


def read_bin(path):
    raw = open(path, "rb").read()
    magic, version, stamp, n, flags, step, _ = struct.unpack("<4sIqIIII", raw[:32])
    pts = np.frombuffer(raw[32:], dtype="<f4").reshape(n, 5)
    return magic, version, stamp, flags, step, pts


@pytest.mark.parametrize("plugin", [StoragePlugin.SQLITE3, StoragePlugin.MCAP], ids=["db3", "mcap"])
@pytest.mark.parametrize("name", ["ouster", "velodyne", "hesai", "livox", "none"])
def test_time_layouts(tmp_path, name, plugin):
    clouds = []
    expected = []
    for frame in range(3):
        msg, stamp, tmax = layout(name, frame)
        clouds.append((BASE + frame * 100_000_000, msg))
        expected.append((stamp, tmax))
    bag = tmp_path / "bag"
    write_bag(bag, clouds, imu_stream(), plugin)
    out = tmp_path / "out"
    assert bag_to_replay.main([str(bag), "--out", str(out)]) == 0

    meta = json.loads((out / "meta.json").read_text())
    assert meta["format"] == "glim_localization_replay"
    assert meta["num_frames"] == 3
    assert meta["per_point_time"] == (name != "none")
    files = sorted(os.listdir(out / "points"))
    assert len(files) == 3
    for file, (stamp, tmax) in zip(files, expected):
        magic, version, stamp_ns, flags, step, pts = read_bin(out / "points" / file)
        assert (magic, version, step) == (b"GLRP", 1, 20)
        assert file == f"{stamp_ns:019d}.bin"
        # Float64 absolute layouts lose a little to the float rounding of the source, hence 20 us
        assert abs(stamp_ns - stamp) <= 20_000
        t = pts[:, 3]
        assert t.min() == 0.0 or name == "none"
        assert t.max() == pytest.approx(tmax, abs=1e-4)
        assert (pts[:, 4] == 7.0).all()
        assert bool(flags & 1) == (name != "none")


def test_imu_sorted_deduplicated_and_g_check(tmp_path):
    imus = [(BASE + i * 5_000_000, make_imu(BASE + i * 5_000_000)) for i in (3, 1, 2, 2, 0)]
    msg, _, _ = layout("ouster", 0)
    bag = tmp_path / "bag"
    write_bag(bag, [(BASE, msg)], imus, StoragePlugin.SQLITE3)
    out = tmp_path / "out"
    assert bag_to_replay.main([str(bag), "--out", str(out)]) == 0
    rows = (out / "imu.csv").read_text().splitlines()
    assert rows[0] == bag_to_replay.IMU_HEADER
    stamps = [int(r.split(",")[0]) for r in rows[1:]]
    assert stamps == sorted(set(stamps)) and len(stamps) == 4
    assert json.loads((out / "meta.json").read_text())["conversion"]["excluded"]["imu_duplicate_stamp"] == 1

    # g units: an error without --acc-scale, converted with it
    g_imus = [(BASE + i * 5_000_000, make_imu(BASE + i * 5_000_000, acc=(0, 0, 1.0))) for i in range(10)]
    bag2 = tmp_path / "bag2"
    write_bag(bag2, [(BASE, msg)], g_imus, StoragePlugin.SQLITE3)
    assert bag_to_replay.main([str(bag2), "--out", str(tmp_path / "out2")]) == 1
    assert bag_to_replay.main([str(bag2), "--out", str(tmp_path / "out3"), "--acc-scale", "9.80665"]) == 0
    first = (tmp_path / "out3" / "imu.csv").read_text().splitlines()[1].split(",")
    assert float(first[3]) == pytest.approx(9.80665)


def test_non_finite_points_are_dropped_and_counted(tmp_path):
    rng = np.random.default_rng(0)
    xyz = rng.uniform(-5, 5, size=(NUM_POINTS, 3)).astype("<f4")
    xyz[3, 0] = np.nan
    xyz[7, 2] = np.inf
    msg = make_cloud(BASE, np.linspace(0, 0.09, NUM_POINTS).astype("<f4"), "time", Field.FLOAT32, xyz)
    bag = tmp_path / "bag"
    write_bag(bag, [(BASE, msg)], imu_stream(), StoragePlugin.SQLITE3)
    out = tmp_path / "out"
    assert bag_to_replay.main([str(bag), "--out", str(out)]) == 0
    meta = json.loads((out / "meta.json").read_text())
    assert meta["conversion"]["excluded"]["points_non_finite"] == 2
    (file,) = os.listdir(out / "points")
    assert read_bin(out / "points" / file)[5].shape[0] == NUM_POINTS - 2


def test_unsupported_time_type_and_errors(tmp_path, capsys):
    msg = make_cloud(BASE, np.arange(NUM_POINTS, dtype="<u4"), "t", Field.UINT32)
    msg.fields[-1] = Field(name="t", offset=16, datatype=Field.INT32, count=1)  # signed 32 bit is not allowed
    bag = tmp_path / "bad"
    write_bag(bag, [(BASE, msg)], imu_stream(), StoragePlugin.SQLITE3)
    assert bag_to_replay.main([str(bag), "--out", str(tmp_path / "o1")]) == 1
    assert "unsupported datatype" in capsys.readouterr().err

    assert bag_to_replay.main([str(tmp_path / "missing"), "--out", str(tmp_path / "o2")]) == 1
    good, _, _ = layout("ouster", 0)
    bag2 = tmp_path / "good"
    write_bag(bag2, [(BASE, good)], imu_stream(), StoragePlugin.SQLITE3)
    assert bag_to_replay.main([str(bag2), "--out", str(tmp_path / "o3"), "--points-topic", "/nope"]) == 1
    assert bag_to_replay.main([str(bag2), "--out", str(tmp_path / "o4"), "--acc-scale", "-1"]) == 2
    # A non-empty output directory is refused
    assert bag_to_replay.main([str(bag2), "--out", str(tmp_path / "o3")]) == 1


def test_contradicting_later_frame_is_an_error(tmp_path, capsys):
    first, _, _ = layout("ouster", 0)
    # The second frame looks absolute although the first was decided relative
    second = make_cloud(BASE + 100_000_000, (np.linspace(0.0, 1.5, NUM_POINTS) * 1e9).astype("<u4"), "t", Field.UINT32)
    bag = tmp_path / "bag"
    write_bag(bag, [(BASE, first), (BASE + 100_000_000, second)], imu_stream(), StoragePlugin.SQLITE3)
    assert bag_to_replay.main([str(bag), "--out", str(tmp_path / "o")]) == 1
    assert "contradicts" in capsys.readouterr().err


DATATYPE = {"<u4": Field.UINT32, "<f4": Field.FLOAT32, "<f8": Field.FLOAT64}


def build_cloud(stamp_ns, columns):
    """columns: list of (name, numpy dtype string, values). Fields are laid out in the given order."""
    arr = np.zeros(len(columns[0][2]), dtype=np.dtype([(n, d) for n, d, _ in columns]))
    flist, offset = [], 0
    for name, dtype, values in columns:
        arr[name] = values
        flist.append(Field(name=name, offset=offset, datatype=DATATYPE[dtype], count=1))
        offset += np.dtype(dtype).itemsize
    return Cloud(
        header=Header(stamp=make_time(stamp_ns), frame_id="lidar"),
        height=1,
        width=len(arr),
        fields=flist,
        is_bigendian=False,
        point_step=arr.dtype.itemsize,
        row_step=arr.dtype.itemsize * len(arr),
        data=np.frombuffer(arr.tobytes(), dtype=np.uint8),
        is_dense=True,
    )


def xyz_columns(dtype="<f4"):
    rng = np.random.default_rng(1)
    pts = rng.uniform(-5, 5, size=(NUM_POINTS, 3))
    return [("x", dtype, pts[:, 0]), ("y", dtype, pts[:, 1]), ("z", dtype, pts[:, 2])]


RAMP = np.linspace(0.0, 0.09, NUM_POINTS)


def convert_two_frames(tmp_path, first, second):
    bag = tmp_path / "bag"
    write_bag(bag, [(BASE, first), (BASE + 100_000_000, second)], imu_stream(), StoragePlugin.SQLITE3)
    return bag_to_replay.main([str(bag), "--out", str(tmp_path / "out")])


def test_relative_mode_then_epoch_values_is_an_error(tmp_path, capsys):
    first = build_cloud(BASE, xyz_columns() + [("t", "<f8", RAMP)])
    second = build_cloud(BASE + 100_000_000, xyz_columns() + [("t", "<f8", SEC + 0.1 + RAMP)])
    assert convert_two_frames(tmp_path, first, second) == 1
    assert "contradicts the relative" in capsys.readouterr().err


def test_absolute_mode_then_relative_values_is_an_error(tmp_path, capsys):
    first = build_cloud(BASE, xyz_columns() + [("t", "<f8", SEC + RAMP)])
    second = build_cloud(BASE + 100_000_000, xyz_columns() + [("t", "<f8", RAMP)])
    assert convert_two_frames(tmp_path, first, second) == 1
    assert "contradicts the absolute" in capsys.readouterr().err


def test_absolute_stamp_must_be_near_the_header(tmp_path, capsys):
    # The mode is right, but the epoch is hours away from the header stamp
    first = build_cloud(BASE, xyz_columns() + [("t", "<f8", SEC + 3 * 3600 + RAMP)])
    bag = tmp_path / "bag"
    write_bag(bag, [(BASE, first)], imu_stream(), StoragePlugin.SQLITE3)
    assert bag_to_replay.main([str(bag), "--out", str(tmp_path / "out")]) == 1
    err = capsys.readouterr().err
    assert "header stamp" in err
    assert "clock offset" in err
    assert "cannot fix" in err  # The message must not send the user to a useless override


def test_negative_stamps_are_rejected(tmp_path, capsys):
    cloud = build_cloud(-1_000_000_000, xyz_columns() + [("t", "<f8", RAMP)])
    bag = tmp_path / "bag"
    write_bag(bag, [(-1_000_000_000, cloud)], imu_stream(), StoragePlugin.SQLITE3)
    assert bag_to_replay.main([str(bag), "--out", str(tmp_path / "out")]) == 1
    assert "negative" in capsys.readouterr().err
    # A negative IMU stamp is rejected as well
    good = build_cloud(BASE, xyz_columns() + [("t", "<f8", RAMP)])
    bag2 = tmp_path / "bag2"
    write_bag(bag2, [(BASE, good)], [(-5, make_imu(-5))], StoragePlugin.SQLITE3)
    assert bag_to_replay.main([str(bag2), "--out", str(tmp_path / "out2")]) == 1
    assert "negative" in capsys.readouterr().err


def test_last_matching_time_field_wins(tmp_path):
    first = build_cloud(BASE, xyz_columns() + [("timestamp", "<f4", RAMP), ("t", "<f4", RAMP * 0.5)])
    bag = tmp_path / "bag"
    write_bag(bag, [(BASE, first)], imu_stream(), StoragePlugin.SQLITE3)
    assert bag_to_replay.main([str(bag), "--out", str(tmp_path / "out")]) == 0
    (file,) = os.listdir(tmp_path / "out" / "points")
    assert read_bin(tmp_path / "out" / "points" / file)[5][:, 3].max() == pytest.approx(0.045, abs=1e-4)
    assert json.loads((tmp_path / "out" / "meta.json").read_text())["conversion"]["time_field"]["field"] == "t"


def test_range_checks_apply_after_narrowing_to_float32(tmp_path, capsys):
    # 0.99999999 s is below 1 as float64 but rounds to 1.0 as float32, which the reader rejects
    times = RAMP.copy()
    times[-1] = 0.99999999
    first = build_cloud(BASE, xyz_columns() + [("t", "<f8", times)])
    bag = tmp_path / "bag"
    write_bag(bag, [(BASE, first)], imu_stream(), StoragePlugin.SQLITE3)
    assert bag_to_replay.main([str(bag), "--out", str(tmp_path / "out")]) == 1
    assert "float32" in capsys.readouterr().err


def test_coordinates_that_overflow_float32_are_dropped(tmp_path):
    columns = xyz_columns("<f8") + [("t", "<f4", RAMP.astype("<f4"))]
    columns[0][2][4] = 1e300
    first = build_cloud(BASE, columns)
    bag = tmp_path / "bag"
    write_bag(bag, [(BASE, first)], imu_stream(), StoragePlugin.SQLITE3)
    assert bag_to_replay.main([str(bag), "--out", str(tmp_path / "out")]) == 0
    assert json.loads((tmp_path / "out" / "meta.json").read_text())["conversion"]["excluded"]["points_non_finite"] == 1
    (file,) = os.listdir(tmp_path / "out" / "points")
    pts = read_bin(tmp_path / "out" / "points" / file)[5]
    assert pts.shape[0] == NUM_POINTS - 1 and np.isfinite(pts).all()


# Point clock offset (--point-clock-offset): the per-point times run on a sensor clock that is OFFSET_S behind the
# header and IMU clock. The header stamps the first point of each frame on the host clock.
OFFSET_S = 3 * 3600 + 0.25


def offset_bag(tmp_path, frames=3, drift_s=0.0, name="bag"):
    clouds = []
    for frame in range(frames):
        header = BASE + frame * 100_000_000
        sensor = SEC + frame * 0.1 - OFFSET_S + frame * drift_s
        clouds.append((header, build_cloud(header, xyz_columns() + [("t", "<f8", sensor + RAMP)])))
    bag = tmp_path / name
    write_bag(bag, clouds, imu_stream(), StoragePlugin.SQLITE3)
    return bag


def stamps(out):
    return [read_bin(out / "points" / f)[2] for f in sorted(os.listdir(out / "points"))]


def test_clock_offset_is_rejected_by_default_with_a_hint(tmp_path, capsys):
    bag = offset_bag(tmp_path)
    assert bag_to_replay.main([str(bag), "--out", str(tmp_path / "out")]) == 1
    err = capsys.readouterr().err
    assert "different clocks" in err and "--point-clock-offset" in err


def test_negative_clock_offset(tmp_path):
    # The point clock is ahead of the header clock: a negative offset, given as a plain argument
    clouds = []
    for frame in range(2):
        header = BASE + frame * 100_000_000
        clouds.append((header, build_cloud(header, xyz_columns() + [("t", "<f8", SEC + frame * 0.1 + 7.5 + RAMP)])))
    bag = tmp_path / "bag"
    write_bag(bag, clouds, imu_stream(), StoragePlugin.SQLITE3)
    out = tmp_path / "out"
    assert bag_to_replay.main([str(bag), "--out", str(out), "--point-clock-offset", "-7.5"]) == 0
    assert all(abs(s - (BASE + i * 100_000_000)) <= 20_000 for i, s in enumerate(stamps(out)))
    assert json.loads((out / "meta.json").read_text())["conversion"]["point_clock_offset"]["offset_ns"] == -7_500_000_000


@pytest.mark.parametrize("value", [str(OFFSET_S), "estimate"])
def test_clock_offset_given_or_estimated(tmp_path, capsys, value):
    bag = offset_bag(tmp_path)
    out = tmp_path / "out"
    assert bag_to_replay.main([str(bag), "--out", str(out), "--point-clock-offset", value]) == 0
    logged = capsys.readouterr().out
    assert "point clock offset" in logged
    for frame, stamp in enumerate(stamps(out)):
        assert abs(stamp - (BASE + frame * 100_000_000)) <= 20_000  # float64 rounding of the source, as above
    offset = json.loads((out / "meta.json").read_text())["conversion"]["point_clock_offset"]
    assert offset["source"] == ("estimated" if value == "estimate" else "given")
    assert offset["estimated_from_frame"] == (0 if value == "estimate" else None)
    assert abs(offset["offset_ns"] - OFFSET_S * 1e9) <= 20_000
    assert offset["definition"] == "header clock = point clock + offset"
    assert abs(offset["residual_s"]["max"]) < 1e-4 and abs(offset["residual_s"]["min"]) < 1e-4


def test_no_offset_records_null_in_meta(tmp_path):
    msg, _, _ = layout("hesai", 0)
    bag = tmp_path / "bag"
    write_bag(bag, [(BASE, msg)], imu_stream(), StoragePlugin.SQLITE3)
    assert bag_to_replay.main([str(bag), "--out", str(tmp_path / "out")]) == 0
    assert json.loads((tmp_path / "out" / "meta.json").read_text())["conversion"]["point_clock_offset"] is None


def test_clock_offset_drift_is_caught(tmp_path, capsys):
    # 0.6 s of drift per frame: frame 2 is 1.2 s away from its header after the offset from frame 0
    bag = offset_bag(tmp_path, drift_s=0.6)
    assert bag_to_replay.main([str(bag), "--out", str(tmp_path / "out"), "--point-clock-offset", "estimate"]) == 1
    err = capsys.readouterr().err
    assert "frame 2" in err and "after the point clock offset" in err


def test_wrong_clock_offset_is_caught(tmp_path, capsys):
    bag = offset_bag(tmp_path)
    assert bag_to_replay.main([str(bag), "--out", str(tmp_path / "out"), "--point-clock-offset", str(OFFSET_S + 5)]) == 1
    assert "after the point clock offset" in capsys.readouterr().err


@pytest.mark.parametrize("value", ["abc", "nan", "inf", "-inf", "1e11", ""])
def test_clock_offset_input_is_validated(tmp_path, capsys, value):
    bag = offset_bag(tmp_path)
    # The = form, so that argparse does not take "-inf" for an option
    assert bag_to_replay.main([str(bag), "--out", str(tmp_path / "out"), f"--point-clock-offset={value}"]) == 2
    assert "--point-clock-offset" in capsys.readouterr().err
    assert not (tmp_path / "out").exists()


def test_clock_offset_with_relative_mode_is_refused(tmp_path, capsys):
    bag = offset_bag(tmp_path)
    args = [str(bag), "--out", str(tmp_path / "o1"), "--point-clock-offset", "1", "--time-mode", "relative"]
    assert bag_to_replay.main(args) == 2
    # Detected relative times (Ouster layout) and clouds without a time field cannot take an offset either
    msg, _, _ = layout("ouster", 0)
    bag2 = tmp_path / "rel"
    write_bag(bag2, [(BASE, msg)], imu_stream(), StoragePlugin.SQLITE3)
    assert bag_to_replay.main([str(bag2), "--out", str(tmp_path / "o2"), "--point-clock-offset", "estimate"]) == 1
    assert "absolute per-point times only" in capsys.readouterr().err
    msg, _, _ = layout("none", 0)
    bag3 = tmp_path / "none"
    write_bag(bag3, [(BASE, msg)], imu_stream(), StoragePlugin.SQLITE3)
    assert bag_to_replay.main([str(bag3), "--out", str(tmp_path / "o3"), "--point-clock-offset", "0.5"]) == 1
    assert "no time field" in capsys.readouterr().err
