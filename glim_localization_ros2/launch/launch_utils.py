"""Helpers of the launch files of glim_localization_ros2 (not a launch file itself).

Standard library and PyYAML only, so that they can be tested without ROS (scripts/tests/test_launch_utils.py).
The launch files load this file by path, next to them in share/glim_localization_ros2/launch.
"""
import json
import math
import os
import shutil

# Parameters of GLIM's glim_ros node that GlimROS declares as double: an integer in a params_file (`delay: 2`) is
# passed as a double, since `-p delay:=2` would be an integer override and declare_parameter<double> would throw
DOUBLE_PARAMS = frozenset({'delay', 'start_offset', 'playback_duration', 'playback_until'})

# The node keys of a params_file that apply to GLIM's node, in increasing priority. Other keys are ignored.
PARAMS_FILE_NODE_KEYS = ('/**', 'glim_ros', '/glim_ros')

# The file the odometry config with the map path goes to when the config's odometry file is outside its directory
ODOMETRY_OVERRIDE_FILE = 'config_odometry_map_override.json'


def strip_json_comments(text):
    """`text` without the // and /* */ comments that GLIM's JSON loader accepts. Comment marks inside strings stay."""
    out = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c == '"':
            # A string, up to the closing quote that is not escaped
            j = i + 1
            while j < n and text[j] != '"':
                j += 2 if text[j] == '\\' else 1
            out.append(text[i:j + 1])
            i = j + 1
        elif text.startswith('//', i):
            j = text.find('\n', i)
            i = n if j < 0 else j
        elif text.startswith('/*', i):
            j = text.find('*/', i + 2)
            if j < 0:
                raise ValueError('unterminated /* comment')
            out.append(' ')
            i = j + 2
        else:
            out.append(c)
            i += 1
    return ''.join(out)


def load_json_with_comments(path):
    with open(path) as f:
        return json.loads(strip_json_comments(f.read()))


def odometry_config_path(config_dir):
    """The odometry config that GLIM loads for config_dir: global.config_odometry of its config.json, relative to it."""
    config_json = os.path.join(config_dir, 'config.json')
    if not os.path.isfile(config_json):
        raise RuntimeError(f"config_path '{config_dir}' has no config.json")
    name = (load_json_with_comments(config_json).get('global') or {}).get('config_odometry')
    if not isinstance(name, str) or not name:
        raise RuntimeError(f'{config_json} does not name the odometry config (global.config_odometry)')
    path = os.path.normpath(os.path.join(config_dir, name))
    if not os.path.isfile(path):
        raise RuntimeError(f"the odometry config '{path}' named by {config_json} does not exist")
    return path


def _inside(path, directory):
    return os.path.commonpath([os.path.realpath(path), os.path.realpath(directory)]) == os.path.realpath(directory)


def config_with_map(config_dir, map_path, work_dir):
    """A copy of config_dir in work_dir whose odometry config (see odometry_config_path) loads map_path. Returns its path.

    GLIM reads each global.config_* file as <config dir>/<name>. Names that point outside config_dir (such as
    ../r1/...) are rewritten in the copy to reach the same files from the copy.
    """
    config_dir = os.path.abspath(config_dir)
    odometry_src = odometry_config_path(config_dir)
    dst = os.path.join(work_dir, 'config')
    shutil.copytree(config_dir, dst)

    config_json = load_json_with_comments(os.path.join(config_dir, 'config.json'))
    global_config = config_json['global']
    rewritten = False
    for key, name in global_config.items():
        if key == 'config_path' or not isinstance(name, str) or not name:
            continue
        target = os.path.normpath(os.path.join(config_dir, name))
        if not _inside(target, config_dir):
            global_config[key] = os.path.relpath(os.path.realpath(target), os.path.realpath(dst))
            rewritten = True

    if _inside(odometry_src, config_dir):
        odometry_dst = os.path.join(dst, os.path.relpath(odometry_src, config_dir))
    else:
        odometry_dst = os.path.join(dst, ODOMETRY_OVERRIDE_FILE)
        global_config['config_odometry'] = ODOMETRY_OVERRIDE_FILE
        rewritten = True
    if rewritten:
        with open(os.path.join(dst, 'config.json'), 'w') as f:
            json.dump(config_json, f, indent=2)

    odometry = load_json_with_comments(odometry_src)
    odometry.setdefault('odometry_estimation', {})['map_path'] = os.path.abspath(map_path)
    with open(odometry_dst, 'w') as f:
        json.dump(odometry, f, indent=2)
    return dst


def params_from_yaml(content):
    """The ros__parameters for GLIM's node in a parsed params_file: those under /**, then glim_ros, then /glim_ros."""
    if content is None:
        return {}
    if not isinstance(content, dict):
        raise ValueError('params_file must be a mapping of node names')
    params = {}
    for key in PARAMS_FILE_NODE_KEYS:
        node = content.get(key) or {}
        node_params = node.get('ros__parameters') if isinstance(node, dict) else None
        if not isinstance(node, dict) or not isinstance(node_params or {}, dict):
            raise ValueError(f"params_file: '{key}' must be a mapping with a ros__parameters mapping")
        params.update(node_params or {})
    return params


def read_params_file(path):
    import yaml  # only the launch files need it

    with open(path) as f:
        return params_from_yaml(yaml.safe_load(f))


def _scalar(name, value, as_double=False):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, int) and not as_double:
        return str(value)
    if isinstance(value, (int, float)):
        value = float(value)
        if math.isnan(value):
            return '.nan'
        if math.isinf(value):
            return '.inf' if value > 0 else '-.inf'
        text = repr(value)
        # repr keeps a '.' or an exponent ("2.0", "1e+20"); both read back as a double
        return text if any(c in text for c in '.e') else text + '.0'
    if isinstance(value, str):
        # Single-quoted YAML: the value stays a string ("", "123", "true", "a: b"); a quote is doubled
        return "'" + value.replace("'", "''") + "'"
    raise ValueError(f"params_file: unsupported value of '{name}': {value!r}")


def to_param_value(name, value):
    """`value` of parameter `name` as the right-hand side of `-p name:=value` (read as YAML by rcl), keeping its type.

    Strings are quoted, doubles keep a decimal point, lists become YAML flow sequences of one type. Integers of the
    double parameters of GlimROS (DOUBLE_PARAMS) become doubles. Nested mappings are rejected: write them as dotted names.
    """
    as_double = name in DOUBLE_PARAMS
    if isinstance(value, dict):
        raise ValueError(f"params_file: '{name}' is a nested mapping, which the R1 launch file does not pass; write each entry as '{name}.<key>'")
    if isinstance(value, (list, tuple)):
        if not value:
            raise ValueError(f"params_file: '{name}' is an empty list, whose type ROS cannot tell")
        kinds = {bool if isinstance(v, bool) else float if isinstance(v, (int, float)) else type(v) for v in value}
        if len(kinds) != 1 or kinds.pop() not in (bool, float, str):
            raise ValueError(f"params_file: '{name}' must be a list of booleans, numbers or strings, all of one kind: {value!r}")
        # A list mixing ints and floats is a list of doubles
        as_double = as_double or any(isinstance(v, float) for v in value)
        return '[' + ', '.join(_scalar(name, v, as_double) for v in value) + ']'
    if value is None:
        raise ValueError(f"params_file: '{name}' has no value")
    return _scalar(name, value, as_double)


def to_ros_arguments(params):
    """`-p name:=value` for each parameter, for Node(ros_arguments=...)."""
    return [arg for name, value in params.items() for arg in ('-p', f'{name}:={to_param_value(name, value)}')]
