"""Tests for the launch file helpers of glim_localization_ros2 (launch/launch_utils.py), without ROS."""
import importlib.util
import json
import os
from pathlib import Path

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[2] / "glim_localization_ros2"
_spec = importlib.util.spec_from_file_location("launch_utils", PACKAGE / "launch" / "launch_utils.py")
lu = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(lu)


# --- -p name:=value ---------------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "name, value, expected",
    [
        ("s", "", "''"),
        ("s", "123", "'123'"),
        ("s", "true", "'true'"),
        ("s", "a: b", "'a: b'"),
        ("s", "it's", "'it''s'"),
        ("s", "/data//map", "'/data//map'"),
        ("b", True, "true"),
        ("b", False, "false"),
        ("i", 3, "3"),
        ("i", -7, "-7"),
        ("d", 2.0, "2.0"),
        ("d", 0.25, "0.25"),
        ("d", 1e20, "1e+20"),
        ("d", float("inf"), ".inf"),
        ("d", float("-inf"), "-.inf"),
        ("d", float("nan"), ".nan"),
        # Integers of GlimROS's double parameters stay doubles
        ("delay", 2, "2.0"),
        ("start_offset", 0, "0.0"),
        ("playback_until", 3.5, "3.5"),
        ("debug", True, "true"),
        ("ints", [1, 2, 3], "[1, 2, 3]"),
        ("doubles", [1, 2.5], "[1.0, 2.5]"),
        ("strings", ["a", "1", ""], "['a', '1', '']"),
        ("bools", [True, False], "[true, false]"),
    ],
)
def test_param_values_keep_their_type(name, value, expected):
    assert lu.to_param_value(name, value) == expected


def rcl_yaml(text):
    """How a -p value reads back as YAML (rcl reads it with a YAML parser, quoted scalars as strings)."""
    return yaml.safe_load(f"v: {text}")["v"]


@pytest.mark.parametrize("value", ["", "123", "true", "a: b", "it's", "#x", "[1]", 2.0, 0.1, 3, True, [1, 2], ["x", "1"]])
def test_param_values_read_back_as_yaml(value):
    assert rcl_yaml(lu.to_param_value("p", value)) == value
    assert type(rcl_yaml(lu.to_param_value("p", value))) is type(value)


@pytest.mark.parametrize(
    "value, message",
    [
        ({"a": 1}, "nested mapping"),
        ([], "empty list"),
        ([1, "a"], "one kind"),
        ([[1]], "one kind"),
        (None, "no value"),
    ],
)
def test_unsupported_param_values_are_rejected(value, message):
    with pytest.raises(ValueError, match=message):
        lu.to_param_value("p", value)


def test_ros_arguments_and_params_file_priority():
    content = yaml.safe_load(
        "/**:\n  ros__parameters:\n    delay: 1\n    dump_path: ''\n"
        "glim_ros:\n  ros__parameters:\n    delay: 2\n"
        "other_node:\n  ros__parameters:\n    delay: 9\n")
    params = lu.params_from_yaml(content)
    assert params == {"delay": 2, "dump_path": ""}
    assert lu.to_ros_arguments(params) == ["-p", "delay:=2.0", "-p", "dump_path:=''"]
    assert lu.params_from_yaml(None) == {}


# --- JSON with comments -----------------------------------------------------------------------------------------


def test_strip_json_comments_respects_strings():
    text = """{
  // a line comment
  "path": "/data//map", /* a block
  comment */ "url": "http://x/*y*/", "quote": "a\\"//b", "n": 1 // trailing
}"""
    assert json.loads(lu.strip_json_comments(text)) == {"path": "/data//map", "url": "http://x/*y*/", "quote": 'a"//b', "n": 1}


def test_strip_json_comments_rejects_an_open_block():
    with pytest.raises(ValueError, match="unterminated"):
        lu.strip_json_comments('{"a": 1 /* no end')


def test_shipped_configs_parse():
    for path in sorted((PACKAGE / "config").glob("r*/*.json")):
        assert isinstance(lu.load_json_with_comments(path), dict), path


# --- map_path override ------------------------------------------------------------------------------------------


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


def test_config_with_map_edits_the_odometry_config_named_by_config_json(tmp_path):
    src = tmp_path / "src"
    write(src / "config.json", '{"global": {"config_path": "", // c\n "config_odometry": "odom/my_odometry.json"}}')
    write(src / "odom" / "my_odometry.json", '{"odometry_estimation": {"map_path": "/old//map", "x": "a//b"}} // c')
    write(src / "config_odometry_localization_cpu.json", '{"odometry_estimation": {"map_path": "/untouched"}}')

    dst = Path(lu.config_with_map(str(src), "relative/map", str(tmp_path / "work")))
    odometry = json.loads((dst / "odom" / "my_odometry.json").read_text())
    assert odometry["odometry_estimation"] == {"map_path": os.path.abspath("relative/map"), "x": "a//b"}
    assert json.loads((dst / "config_odometry_localization_cpu.json").read_text())["odometry_estimation"]["map_path"] == "/untouched"
    # config.json needs no rewrite; the source is not modified
    assert (dst / "config.json").read_text() == (src / "config.json").read_text()
    assert "/old//map" in (src / "odom" / "my_odometry.json").read_text()


def test_config_with_map_follows_files_outside_the_config_dir(tmp_path):
    # As config/r2: the odometry and other files are in ../r1
    write(tmp_path / "r1" / "odometry.json", '{"odometry_estimation": {"map_path": ""}}')
    write(tmp_path / "r1" / "sensors.json", "{}")
    write(tmp_path / "r2" / "config.json", '{"global": {"config_path": "", "config_odometry": "../r1/odometry.json", "config_sensors": "../r1/sensors.json"}}')

    dst = Path(lu.config_with_map(str(tmp_path / "r2"), "/maps/a", str(tmp_path / "work")))
    config = json.loads((dst / "config.json").read_text())["global"]
    assert config["config_odometry"] == lu.ODOMETRY_OVERRIDE_FILE
    # GLIM reads <config dir>/<name>
    assert (dst / config["config_sensors"]).resolve() == (tmp_path / "r1" / "sensors.json").resolve()
    assert json.loads((dst / config["config_odometry"]).read_text())["odometry_estimation"]["map_path"] == "/maps/a"


@pytest.mark.parametrize(
    "config_json, message",
    [
        (None, "no config.json"),
        ('{"global": {}}', "global.config_odometry"),
        ('{"global": {"config_odometry": "missing.json"}}', "does not exist"),
    ],
)
def test_config_with_map_errors(tmp_path, config_json, message):
    src = tmp_path / "src"
    src.mkdir()
    if config_json is not None:
        write(src / "config.json", config_json)
    with pytest.raises(RuntimeError, match=message):
        lu.config_with_map(str(src), "/maps/a", str(tmp_path / "work"))


def test_shipped_r1_config_gets_the_map(tmp_path):
    dst = Path(lu.config_with_map(str(PACKAGE / "config" / "r1"), "/maps/a", str(tmp_path)))
    odometry = lu.load_json_with_comments(lu.odometry_config_path(str(dst)))
    assert odometry["odometry_estimation"]["map_path"] == "/maps/a"
