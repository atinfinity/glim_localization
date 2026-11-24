"""The occupancy grid from map_compiler, read back with the rules of nav2_map_server.

Runs the built map_compiler (GLIM_LOC_MAP_COMPILER, default /ws/build/core/tools/map_compiler). Inside the
container (GLIM_LOC_PYTHON set) a missing binary is a failure; elsewhere the tests skip.
"""

import os
import subprocess

import pytest
import yaml

MAP_COMPILER = os.environ.get("GLIM_LOC_MAP_COMPILER", "/ws/build/core/tools/map_compiler")

# nav_msgs/OccupancyGrid values
OCCUPIED, FREE, UNKNOWN = 100, 0, -1


@pytest.fixture
def map_compiler():
    if not os.path.isfile(MAP_COMPILER):
        if os.environ.get("GLIM_LOC_PYTHON"):
            pytest.fail(f"map_compiler not found at {MAP_COMPILER}; build the core first (scripts/build_core.sh)")
        pytest.skip(f"map_compiler not found at {MAP_COMPILER}")
    return MAP_COMPILER


def write_floor_ply(path):
    """A 3 x 2 m floor at z = 5 from (20, 30), every 0.1 m at cell-centred offsets.

    A 0.5 x 0.5 m box at z = 6 sits in the corner with the highest y and lowest x, a 0.5 x 0.5 m hole in the
    floor in the corner with the lowest y and highest x, and a ceiling at z = 9 everywhere.
    """
    points = []
    for i in range(30):
        for j in range(20):
            x, y = 20.05 + 0.1 * i, 30.05 + 0.1 * j
            if not (i >= 25 and j < 5):
                points.append((x, y, 5.0))
            points.append((x, y, 9.0))
            if i < 5 and j >= 15:
                points.append((x, y, 6.0))
    with open(path, "w") as handle:
        handle.write("ply\nformat ascii 1.0\ncomment synthetic\n")
        handle.write(f"element vertex {len(points)}\n")
        handle.write("property double x\nproperty double y\nproperty double z\nproperty uchar red\nend_header\n")
        for x, y, z in points:
            handle.write(f"{x!r} {y!r} {z!r} 128\n")


def read_pgm(path):
    """Width, height and rows (top row first) of a binary (P5) 8-bit PGM, comments allowed."""
    with open(path, "rb") as handle:
        data = handle.read()
    tokens = []
    pos = 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos].decode())
    pos += 1  # The single whitespace after the max value
    assert tokens[0] == "P5"
    width, height, max_value = int(tokens[1]), int(tokens[2]), int(tokens[3])
    assert max_value == 255
    pixels = data[pos:]
    assert len(pixels) == width * height
    return width, height, [list(pixels[r * width:(r + 1) * width]) for r in range(height)]


def load_like_map_server(yaml_path):
    """Load a map YAML and its image as nav2_map_server's map_io does (loadMapYaml and loadMapFromFile).

    Returns (info, data) where data is the nav_msgs/OccupancyGrid row-major array (row 0 at the lowest y).
    """
    with open(yaml_path) as handle:
        doc = yaml.safe_load(handle)

    # Required keys and their types (map_io reads them with yaml-cpp as<...>())
    for key in ("image", "resolution", "origin", "negate", "occupied_thresh", "free_thresh"):
        assert key in doc, key
    assert isinstance(doc["image"], str) and doc["image"]
    assert isinstance(doc["resolution"], float) and doc["resolution"] > 0
    assert isinstance(doc["origin"], list) and len(doc["origin"]) == 3
    assert all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in doc["origin"])
    assert isinstance(doc["negate"], int) and doc["negate"] in (0, 1)
    assert isinstance(doc["occupied_thresh"], float) and isinstance(doc["free_thresh"], float)
    assert 0.0 <= doc["free_thresh"] < doc["occupied_thresh"] <= 1.0
    mode = doc.get("mode", "trinary")
    assert mode in ("trinary", "scale", "raw")
    assert mode == "trinary"

    # A relative image path is relative to the YAML file.
    image = doc["image"]
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(yaml_path), image)
    width, height, rows = read_pgm(image)

    data = [UNKNOWN] * (width * height)
    for row, pixels in enumerate(rows):
        for x, pixel in enumerate(pixels):
            shade = pixel / 255.0
            occ = shade if doc["negate"] else 1.0 - shade
            if doc["occupied_thresh"] < occ:
                value = OCCUPIED
            elif occ < doc["free_thresh"]:
                value = FREE
            else:
                value = UNKNOWN
            # The top image row is the highest y.
            data[(height - row - 1) * width + x] = value
    return {"width": width, "height": height, "resolution": doc["resolution"], "origin": doc["origin"]}, data


def compile_map(map_compiler, tmp_path, *options):
    ply = tmp_path / "floor.ply"
    write_floor_ply(ply)
    return subprocess.run([map_compiler, str(ply), str(tmp_path / "map"), *options], capture_output=True, text=True)


def test_occupancy_grid_loads_like_map_server(map_compiler, tmp_path):
    prefix = tmp_path / "out" / "floor"
    result = compile_map(map_compiler, tmp_path, "--grid-size", "2", "--occupancy-grid", str(prefix), "--occupancy-resolution", "0.5",
                         "--occupancy-z-min", "5.5", "--occupancy-z-max", "8")
    assert result.returncode == 0, result.stdout + result.stderr
    assert (tmp_path / "map" / "map_index.json").is_file()

    info, data = load_like_map_server(str(prefix) + ".yaml")
    assert info["width"] == 6 and info["height"] == 4
    assert info["resolution"] == 0.5
    assert info["origin"] == [20.0, 30.0, 0.0]

    def cell(x, y):
        """The value of the cell holding map-frame (x, y), indexed as a map_server client does."""
        i = int((x - info["origin"][0]) // info["resolution"])
        j = int((y - info["origin"][1]) // info["resolution"])
        return data[j * info["width"] + i]

    assert cell(20.2, 31.8) == OCCUPIED  # The box at the highest y
    assert cell(22.8, 30.2) == UNKNOWN   # The hole at the lowest y
    assert cell(20.2, 30.2) == FREE
    assert cell(22.8, 31.8) == FREE
    assert data.count(OCCUPIED) == 1 and data.count(UNKNOWN) == 1 and data.count(FREE) == 22


def test_occupancy_grid_requires_the_band(map_compiler, tmp_path):
    result = compile_map(map_compiler, tmp_path, "--occupancy-grid", str(tmp_path / "floor"))
    assert result.returncode == 1
    assert "must both be set" in result.stdout + result.stderr
    assert not (tmp_path / "map").exists()
    assert not (tmp_path / "floor.pgm").exists()
