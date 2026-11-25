# Copyright (c) 2026 atinfinity
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
R1 end to end: glim_rosbag with the localization extension over a synthetic bag.

Headless; no display.

Inputs come from scripts/test_colcon.sh (see there), through environment variables:
  GLIM_R1_BAG   the bag made by scripts/replay_to_bag.py (with two /initialpose messages)
  GLIM_R1_DATA  the replay dataset the bag was made from (traj_gt.txt is the ground truth)
  GLIM_R1_MAP   the map dump of the same scene

Assertions are on counts and bounds, not on timing. glim_rosbag keeps running after the bag ends
(auto_quit:=false), and the test waits for the last pose with a long deadline that FAILS the test
when passed.
"""
import atexit
import bisect
import json
import math
import os
import re
import shutil
import subprocess
import tempfile
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from glim_localization_msgs.msg import LocalizationStatus
import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

# Acceptance bounds for the ROS 2 interface
MIN_POSE_RATIO = 0.9
SETTLE_S = 3.0
MAX_TRANS_M = 0.1
MAX_ROT_DEG = 1.0
EXPECTED_RESETS = 2  # the two /initialpose messages in the bag
DEADLINE_S = 180.0  # fails the test; not a success path
# Library file names (as ldd prints them) that a headless build must not link. Not a substring
# match: libglim is fine.
GUI_LIBS = re.compile(
    r'^lib(GL|GLX|GLU|GLEW|GLESv2|GLdispatch|EGL|OpenGL|X11|X11-xcb|xcb)[._-]'
    r'|(?i:iridescence|glfw)')

NODE_NAME = 'glim_ros'  # GlimROS node, so ~/pose is /glim_ros/pose


def require_env(name):
    value = os.environ.get(name)
    if not value or not os.path.exists(value):
        raise RuntimeError(
            f'{name} is not set or does not exist: run this test through scripts/test_colcon.sh')
    return value


def make_config_dir(dst, map_path):
    """Copy config/r1 to dst with the map path filled in."""
    src = os.path.join(get_package_share_directory('glim_localization_ros2'), 'config', 'r1')
    shutil.copytree(src, dst)
    path = os.path.join(dst, 'config_odometry_localization_cpu.json')
    with open(path) as f:
        # The shipped configs start with a // provenance comment, which GLIM's own loader accepts
        config = json.loads('\n'.join(line.split('//')[0] for line in f))
    config['odometry_estimation']['map_path'] = map_path
    with open(path, 'w') as f:
        json.dump(config, f)
    return dst


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    bag = require_env('GLIM_R1_BAG')
    require_env('GLIM_R1_DATA')
    # The config copy and the dump go to a directory of this test run, removed when the test
    # process exits (pass or fail)
    work_dir = tempfile.mkdtemp(prefix='glim_r1_')
    atexit.register(shutil.rmtree, work_dir, ignore_errors=True)
    config_dir = make_config_dir(os.path.join(work_dir, 'config'), require_env('GLIM_R1_MAP'))
    glim_rosbag = os.path.join(get_package_prefix('glim_ros'), 'lib', 'glim_ros', 'glim_rosbag')
    # glim_rosbag saves a dump on exit, by default to /tmp/dump: a path of its own, so that
    # parallel tests share no output path. GlimROS also writes a fixed /tmp/glim_log.log, only with
    # debug:=true (not set here), and no parameter moves it: GlimROS-based launch tests that set
    # debug would share that file under ctest -j.
    dump_path = os.path.join(work_dir, 'dump')
    proc = launch.actions.ExecuteProcess(
        # delay: the bag is read 2 s after the node is up, so that the test's subscribers are
        # connected first
        cmd=[glim_rosbag, bag, '--ros-args', '-p', f'config_path:={config_dir}',
             '-p', f'dump_path:={dump_path}', '-p', 'auto_quit:=false', '-p', 'delay:=2.0'],
        output='screen',
    )
    return launch.LaunchDescription([proc, launch_testing.actions.ReadyToTest()]), {'proc': proc}


def read_tum(path):
    rows = [line.split() for line in open(path) if line.strip() and not line.startswith('#')]
    return [[float(v) for v in r[:8]] for r in rows]


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rot_angle_deg(qa, qb):
    dot = abs(sum(a * b for a, b in zip(qa, qb)))
    dot /= math.sqrt(sum(a * a for a in qa) * sum(b * b for b in qb))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def stamp_s(t):
    return t.sec + t.nanosec * 1e-9


class TestR1Extension(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('r1_test_listener')
        qos = QoSProfile(depth=5000, reliability=ReliabilityPolicy.RELIABLE)
        cls.poses, cls.statuses, cls.transforms = [], [], []
        cls.node.create_subscription(
            PoseWithCovarianceStamped, f'/{NODE_NAME}/pose', cls.poses.append, qos)
        cls.node.create_subscription(
            LocalizationStatus, f'/{NODE_NAME}/status', cls.statuses.append, qos)
        cls.node.create_subscription(
            TFMessage, '/tf', lambda m: cls.transforms.extend(m.transforms), qos)
        cls.gt = read_tum(os.path.join(require_env('GLIM_R1_DATA'), 'traj_gt.txt'))
        cls.gt_stamps = [row[0] for row in cls.gt]

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_for_last_pose(self, proc_info, proc):
        """
        Spin until a pose near the last ground truth stamp arrives.

        Fails, never passes, when the deadline is reached.
        """
        import time

        last = self.gt_stamps[-1]
        deadline = time.monotonic() + DEADLINE_S
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.poses and stamp_s(self.poses[-1].header.stamp) >= last - 0.25:
                # Drain what is already queued: status and TF follow the pose
                for _ in range(20):
                    rclpy.spin_once(self.node, timeout_sec=0.05)
                return
            # proc_info holds a ProcessStarted event while the process runs, and a ProcessExited
            # event (with returncode) after
            self.assertIsNone(
                getattr(proc_info[proc], 'returncode', None),
                'glim_rosbag exited before the last pose was published')
        self.fail(
            f'the last pose (stamp {last:.2f}) did not arrive within {DEADLINE_S} s; '
            f'got {len(self.poses)} poses')

    def match_gt(self, stamp):
        k = bisect.bisect_left(self.gt_stamps, stamp)
        candidates = [i for i in (k - 1, k) if 0 <= i < len(self.gt)]
        best = min(candidates, key=lambda i: abs(self.gt_stamps[i] - stamp))
        return self.gt[best] if abs(self.gt_stamps[best] - stamp) < 0.005 else None

    def test_localization(self, proc_info, proc_output, proc):
        self.wait_for_last_pose(proc_info, proc)
        num_frames = len(self.gt)

        # Only our extension was loaded: no viewer, which would publish a second odom->base
        output = b''.join(chunk.text for chunk in proc_output[proc]).decode(errors='replace')
        self.assertIn('load libglim_localization_extension.so', output)
        self.assertNotIn('viewer', output.lower())

        # Pose coverage
        self.assertGreaterEqual(
            len(self.poses), MIN_POSE_RATIO * num_frames,
            f'{len(self.poses)} poses for {num_frames} frames')

        # Accuracy against the ground truth after the settling time
        t0 = self.gt_stamps[0]
        checked = 0
        worst_trans, worst_rot = 0.0, 0.0
        for pose in self.poses:
            stamp = stamp_s(pose.header.stamp)
            self.assertEqual(pose.header.frame_id, 'map')
            if stamp < t0 + SETTLE_S:
                continue
            gt = self.match_gt(stamp)
            self.assertIsNotNone(gt, f'no ground truth near {stamp}')
            p, q = pose.pose.pose.position, pose.pose.pose.orientation
            trans = math.dist((p.x, p.y, p.z), gt[1:4])
            rot = rot_angle_deg((q.x, q.y, q.z, q.w), gt[4:8])
            worst_trans, worst_rot = max(worst_trans, trans), max(worst_rot, rot)
            checked += 1
        self.assertGreaterEqual(
            checked, MIN_POSE_RATIO * (num_frames - SETTLE_S * 10),
            'too few poses after the settling time')
        self.assertLessEqual(worst_trans, MAX_TRANS_M, f'translation error {worst_trans:.3f} m')
        self.assertLessEqual(worst_rot, MAX_ROT_DEG, f'rotation error {worst_rot:.3f} deg')

        # TF: map->odom and odom->base_link at (nearly) every frame, composing to the published
        # pose
        by_stamp = {}
        for tf in self.transforms:
            key = (tf.header.stamp.sec, tf.header.stamp.nanosec)
            by_stamp.setdefault(key, {})[(tf.header.frame_id, tf.child_frame_id)] = tf.transform
        both = {
            s: d for s, d in by_stamp.items()
            if ('map', 'odom') in d and ('odom', 'base_link') in d}
        self.assertGreaterEqual(
            len(both), MIN_POSE_RATIO * num_frames,
            f'{len(both)} stamps with both map->odom and odom->base_link')
        for pose in self.poses[-5:]:
            d = both.get((pose.header.stamp.sec, pose.header.stamp.nanosec))
            self.assertIsNotNone(d, 'no TF at a pose stamp')
            a, b = d[('map', 'odom')], d[('odom', 'base_link')]
            ra = quat_to_matrix(a.rotation.x, a.rotation.y, a.rotation.z, a.rotation.w)
            rb = quat_to_matrix(b.rotation.x, b.rotation.y, b.rotation.z, b.rotation.w)
            ta = (a.translation.x, a.translation.y, a.translation.z)
            tb = (b.translation.x, b.translation.y, b.translation.z)
            composed_t = [sum(ra[i][k] * tb[k] for k in range(3)) + ta[i] for i in range(3)]
            composed_r = matmul(ra, rb)
            q = pose.pose.pose.orientation
            rp = quat_to_matrix(q.x, q.y, q.z, q.w)
            p = pose.pose.pose.position
            self.assertLess(math.dist(composed_t, (p.x, p.y, p.z)), 1e-4)
            self.assertLess(
                max(abs(composed_r[i][j] - rp[i][j]) for i in range(3) for j in range(3)), 1e-4)

        # /initialpose reached request_reset: both bag messages were applied, as external resets
        self.assertTrue(self.statuses, 'no status')
        final = self.statuses[-1]
        self.assertEqual(final.reset_count, EXPECTED_RESETS)
        self.assertEqual(final.last_reset_reason, LocalizationStatus.RESET_EXTERNAL)
        self.assertEqual(final.state, LocalizationStatus.TRACKING)
        print(
            f'R1: {len(self.poses)}/{num_frames} poses, '
            f'worst error {worst_trans:.3f} m {worst_rot:.3f} deg, '
            f'reset_count {final.reset_count}')

    def test_config_lists_only_our_extension(self):
        config_dir = os.path.join(
            get_package_share_directory('glim_localization_ros2'), 'config', 'r1')
        text = '\n'.join(
            line.split('//')[0] for line in open(os.path.join(config_dir, 'config_ros.json')))
        ros = json.loads(text)['glim_ros']
        self.assertEqual(ros['extension_modules'], ['libglim_localization_extension.so'])
        self.assertFalse(ros['enable_local_mapping'])
        self.assertFalse(ros['enable_global_mapping'])
        for name, section in (
                ('config_sub_mapping_none.json', 'sub_mapping'),
                ('config_global_mapping_none.json', 'global_mapping')):
            config = json.load(open(os.path.join(config_dir, name)))
            self.assertEqual(config[section]['so_name'], '')

    def test_no_gui_libraries(self):
        lib_dir = os.path.join(get_package_prefix('glim_localization_ros2'), 'lib')
        for name in ('libglim_localization_extension.so', 'libodometry_localization_cpu.so'):
            path = os.path.join(lib_dir, name)
            # The odometry module is installed by glim_localization
            if not os.path.exists(path):
                path = os.path.join(get_package_prefix('glim_localization'), 'lib', name)
            ldd = subprocess.run(['ldd', path], capture_output=True, text=True, check=True).stdout
            self.assertNotIn('not found', ldd, f'{name} has unresolved libraries:\n{ldd}')
            for line in ldd.splitlines():
                soname = line.strip().split(' ')[0]
                self.assertIsNone(GUI_LIBS.search(soname), f'{name} links {soname}:\n{ldd}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # The test ends glim_rosbag with SIGINT; any other failure is an error
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[launch_testing.asserts.EXIT_OK, -2, 130])
