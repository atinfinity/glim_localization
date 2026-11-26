# Copyright (c) 2026 atinfinity
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Shared parts of the R2 launch tests (test_r2_node.py, test_r2_imu_gap.py).

Not a test module itself.
"""
import atexit
import bisect
import importlib.util
import json
import math
import os
import re
import shutil
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from glim_localization_msgs.msg import LocalizationStatus
import launch
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosbag2_interfaces.srv import Resume
from tf2_msgs.msg import TFMessage

NODE = '/glim_localization'
SERVICE_TIMEOUT_S = 30.0  # fails the test; not a success path
DEADLINE_S = 180.0  # fails the test; not a success path
# Spun before resuming the player, after its publishers are visible here and (with node_matched)
# the node has matched them. It covers the other direction: a reliable volatile publisher sends
# nothing to a reader it has not matched yet, and the player's writers match the node's readers
# independently of the node's matched event. Not redundant.
DISCOVERY_MARGIN_S = 2.0
# After the bag ended, how long the node may go without a new pose before the test fails
PLAYER_GRACE_S = 20.0
# The topics of the synthetic bags (scripts/replay_to_bag.py)
BAG_TOPICS = ['/imu', '/points', '/initialpose']


def require_env(name):
    value = os.environ.get(name)
    if not value or not os.path.exists(value):
        raise RuntimeError(
            f'{name} is not set or does not exist: run this test through scripts/test_colcon.sh')
    return value


def load_launch_utils():
    """
    Load the installed launch/launch_utils.py as the launch files load it.

    It reads JSON with comments and makes the R1 map_path override.
    """
    path = os.path.join(
        get_package_share_directory('glim_localization_ros2'), 'launch', 'launch_utils.py')
    spec = importlib.util.spec_from_file_location('glim_localization_launch_utils', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def strip_comments(path):
    return load_launch_utils().load_json_with_comments(path)


def make_config_dir():
    """
    Copy config/r2 (and the r1 files it refers to) with reliable, deep input queues.

    The installed config subscribes with the sensor_data profile (best effort), as GLIM does. A
    test must not lose a message in DDS: a lost IMU sample is an IMU gap and changes reset_count.
    map_path is left to the ROS parameter. The copy is removed when the test process exits.
    """
    share = os.path.join(get_package_share_directory('glim_localization_ros2'), 'config')
    root = tempfile.mkdtemp(prefix='glim_r2_config_')
    atexit.register(shutil.rmtree, root, ignore_errors=True)
    shutil.copytree(os.path.join(share, 'r1'), os.path.join(root, 'r1'))
    shutil.copytree(os.path.join(share, 'r2'), os.path.join(root, 'r2'))
    path = os.path.join(root, 'r2', 'config_ros.json')
    config = strip_comments(path)
    config['glim_ros']['imu_qos'] = {
        'profile': 'default', 'reliability': 'reliable', 'depth': 10000}
    config['glim_ros']['points_qos'] = {
        'profile': 'default', 'reliability': 'reliable', 'depth': 1000}
    with open(path, 'w') as f:
        json.dump(config, f)
    return os.path.join(root, 'r2')


def node_process(config_dir, map_path):
    exe = os.path.join(
        get_package_prefix('glim_localization_ros2'), 'lib', 'glim_localization_ros2',
        'glim_localization_node')
    return launch.actions.ExecuteProcess(
        cmd=[
            exe, '--ros-args', '-p', 'use_sim_time:=true', '-p', f'config_path:={config_dir}',
            '-p', f'map_path:={map_path}'],
        output='screen',
    )


def wait_for_node_matched(proc_output, timeout=SERVICE_TIMEOUT_S):
    """
    Return a `node_matched` for R2Client.play_and_wait.

    It waits in the launch output for the node's log line "matched publishers on <topic>: N"
    (N >= 1) of each topic. Only for a test whose node meets one
    player: a line from an earlier player would also match. Needs an RMW with matched events (Fast
    DDS, Cyclone DDS and Connext have them); without them the node logs "matched events not
    supported on <topic>" and this times out. One deadline covers all topics.
    """
    def wait(topics):
        end = time.monotonic() + timeout
        for topic in topics:
            proc_output.assertWaitFor(
                expected_output=re.compile(rf'matched publishers on {re.escape(topic)}: [1-9]'),
                timeout=max(end - time.monotonic(), 0.1), stream='stderr')
    return wait


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


class R2Client:
    """A test node that drives the lifecycle of glim_localization and records its outputs."""

    def __init__(self, name, data_dir, target=NODE):
        """
        Create the client node.

        `target` is the fully qualified name of the node under test (GLIM's R1 node is /glim_ros).
        """
        self.node = rclpy.create_node(name)
        qos = QoSProfile(depth=5000, reliability=ReliabilityPolicy.RELIABLE)
        self.poses, self.statuses, self.transforms = [], [], []
        self.node.create_subscription(
            PoseWithCovarianceStamped, f'{target}/pose', self.poses.append, qos)
        self.node.create_subscription(
            LocalizationStatus, f'{target}/status', self.statuses.append, qos)
        self.node.create_subscription(
            TFMessage, '/tf', lambda m: self.transforms.extend(m.transforms), qos)
        self.change_state = self.node.create_client(ChangeState, f'{target}/change_state')
        self.get_state = self.node.create_client(GetState, f'{target}/get_state')
        self.set_parameters = self.node.create_client(SetParameters, f'{target}/set_parameters')
        self.resume_player = self.node.create_client(Resume, '/rosbag2_player/resume')
        self.gt = read_tum(os.path.join(data_dir, 'traj_gt.txt'))
        self.gt_stamps = [row[0] for row in self.gt]

    def destroy(self):
        self.node.destroy_node()

    def call(self, client, request, timeout=SERVICE_TIMEOUT_S):
        """Call a service and return the response; raise when it does not answer in time."""
        if not client.wait_for_service(timeout_sec=timeout):
            raise AssertionError(f'service {client.srv_name} is not available')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            raise AssertionError(f'service {client.srv_name} did not answer within {timeout} s')
        return future.result()

    def state(self):
        return self.call(self.get_state, GetState.Request()).current_state.id

    def transition(self, transition_id):
        request = ChangeState.Request()
        request.transition = Transition(id=transition_id)
        return self.call(self.change_state, request).success

    def set_string(self, name, value):
        request = SetParameters.Request()
        request.parameters = [Parameter(
            name=name,
            value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value))]
        results = self.call(self.set_parameters, request).results
        if not results or not results[0].successful:
            raise AssertionError(f'failed to set {name}')

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_for_subscribers(self, topics, timeout=SERVICE_TIMEOUT_S):
        """Wait until the node has subscribed to each topic, so that the bag is not lost."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if all(self.node.count_subscribers(t) >= 1 for t in topics):
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        raise AssertionError(f'no subscriber on {topics} within {timeout} s')

    def _start_player(self, bag, topics, extra_args, node_matched=None):
        """
        Start `ros2 bag play` paused and resume it once its publishers of `topics` are visible.

        It resumes a margin after they are visible here.

        The topics are reliable and volatile, so a message sent before the node has matched the
        player is lost (a fixed --delay was not enough on a loaded host, and the lost /initialpose
        left the node without a pose). That this test node sees the player says nothing about the
        node under test, whose discovery of the player can lag by seconds (issue #20: the first
        1.8 s of the bag, with the /initialpose at 0 s, never reached the node). `node_matched`, if
        given, blocks until the node under test has matched the player's publishers (see
        wait_for_node_matched).
        """
        player = subprocess.Popen(
            ['ros2', 'bag', 'play', bag, '--clock', '100', '--start-paused',
             '--disable-keyboard-controls'] + extra_args)
        try:
            end = time.monotonic() + SERVICE_TIMEOUT_S
            while not all(self.node.count_publishers(t) >= 1 for t in topics):
                if time.monotonic() > end:
                    raise AssertionError(
                        f'the player did not advertise {topics} within {SERVICE_TIMEOUT_S} s')
                if player.poll() is not None:
                    raise AssertionError(
                        f'ros2 bag play exited with {player.returncode} before playing')
                rclpy.spin_once(self.node, timeout_sec=0.1)
            if node_matched:
                node_matched(topics)
            self.spin_for(DISCOVERY_MARGIN_S)
            # Raises when the player does not answer
            self.call(self.resume_player, Resume.Request())
        except BaseException:
            player.kill()
            player.wait()
            raise
        return player

    @staticmethod
    def _stop_player(player):
        if player.poll() is None:
            player.kill()
            player.wait()

    def play_to_end(self, bag, topics, rate=1.0):
        """Play only `topics` of the bag at `rate`; spin until it exits and a bit longer."""
        player = self._start_player(bag, topics, ['--rate', str(rate), '--topics'] + list(topics))
        try:
            deadline = time.monotonic() + DEADLINE_S
            while player.poll() is None:
                if time.monotonic() > deadline:
                    raise AssertionError(f'ros2 bag play did not finish within {DEADLINE_S} s')
                rclpy.spin_once(self.node, timeout_sec=0.1)
            if player.returncode != 0:
                raise AssertionError(f'ros2 bag play exited with {player.returncode}')
            self.spin_for(2.0)  # the node drains its queue
        finally:
            self._stop_player(player)

    def play_and_wait(self, bag, topics, during=None, node_matched=None):
        """
        Play `topics` of the bag with /clock until a pose near the last ground truth stamp arrives.

        Only `topics` are played: the shared fixture bag also has /wheel_odom, which no R2 test
        plays.

        `during`, if given, is called after every spin while the bag plays (it may call services).
        Fails, never passes, when a deadline is reached; fails early, with the number of poses,
        when the bag has ended and no pose came for PLAYER_GRACE_S (for example a lost or rejected
        initial pose). The player exits at the end of the bag; the node does not. `node_matched`:
        see _start_player.
        """
        player = self._start_player(bag, topics, ['--topics'] + list(topics), node_matched)
        try:
            last = self.gt_stamps[-1]
            deadline = time.monotonic() + DEADLINE_S
            ended_at = None
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if during:
                    during()
                if self.poses and stamp_s(self.poses[-1].header.stamp) >= last - 0.25:
                    # Drain what is already queued: status and TF follow the pose
                    self.spin_for(1.0)
                    break
                if ended_at is None and player.poll() is not None:
                    ended_at = time.monotonic()
                    pose_count = len(self.poses)
                elif (ended_at is not None and len(self.poses) == pose_count and
                      time.monotonic() - ended_at > PLAYER_GRACE_S):
                    if self.poses:
                        newest = f', the newest at {stamp_s(self.poses[-1].header.stamp):.2f}'
                    else:
                        newest = ' (was the initial pose lost or rejected?)'
                    raise AssertionError(
                        f'the bag ended and no pose came for {PLAYER_GRACE_S} s: '
                        f'got {len(self.poses)} poses{newest}')
                elif ended_at is not None and len(self.poses) != pose_count:
                    ended_at, pose_count = time.monotonic(), len(self.poses)
            else:
                raise AssertionError(
                    f'the last pose (stamp {last:.2f}) did not arrive within {DEADLINE_S} s; '
                    f'got {len(self.poses)} poses')
            player.wait(timeout=SERVICE_TIMEOUT_S)
            if player.returncode != 0:
                raise AssertionError(f'ros2 bag play exited with {player.returncode}')
        finally:
            self._stop_player(player)

    def wait_until(self, condition, what, timeout=SERVICE_TIMEOUT_S):
        """Spin until condition() is true; fail with `what` after timeout."""
        end = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > end:
                raise AssertionError(f'{what} within {timeout} s')
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def match_gt(self, stamp):
        k = bisect.bisect_left(self.gt_stamps, stamp)
        candidates = [i for i in (k - 1, k) if 0 <= i < len(self.gt)]
        best = min(candidates, key=lambda i: abs(self.gt_stamps[i] - stamp))
        return self.gt[best] if abs(self.gt_stamps[best] - stamp) < 0.005 else None

    def pose_error_list(self, after):
        """
        Return the errors of the poses stamped at or after `after`.

        A list of (stamp, translation error [m], rotation error [deg]).
        """
        errors = []
        for pose in self.poses:
            stamp = stamp_s(pose.header.stamp)
            if stamp < after:
                continue
            gt = self.match_gt(stamp)
            if gt is None:
                raise AssertionError(f'no ground truth near {stamp}')
            p, q = pose.pose.pose.position, pose.pose.pose.orientation
            errors.append((
                stamp, math.dist((p.x, p.y, p.z), gt[1:4]),
                rot_angle_deg((q.x, q.y, q.z, q.w), gt[4:8])))
        return errors

    def pose_errors(self, after):
        """
        Return the worst errors of the poses stamped at or after `after`, and how many.

        The worst translation [m] and rotation [deg] error.
        """
        errors = self.pose_error_list(after)
        return (
            max((e[1] for e in errors), default=0.0), max((e[2] for e in errors), default=0.0),
            len(errors))

    def describe_errors(self, after, max_trans, max_rot):
        """Describe the poses over either bound, for a failure message."""
        bad = [
            f'{s:.2f}: {t:.3f} m {r:.3f} deg' for s, t, r in self.pose_error_list(after)
            if t > max_trans or r > max_rot]
        return f'{len(bad)} poses over the bounds: ' + ', '.join(bad[:20])

    def coverage_report(self):
        """
        Tell which ground truth frames have no pose, and what ~/status said, for a failure message.

        The missing frames are given as runs of seconds from the first frame, so a lost start (no
        pose until the bag's next /initialpose) reads differently from frames dropped one by one.
        """
        t0 = self.gt_stamps[0]
        posed = {round(stamp_s(p.header.stamp) - t0, 2) for p in self.poses}
        missing = [round(s - t0, 2) for s in self.gt_stamps if round(s - t0, 2) not in posed]
        runs = []
        for s in missing:
            if runs and s - runs[-1][1] < 0.15:
                runs[-1][1] = s
            else:
                runs.append([s, s])
        states = {}
        for status in self.statuses:
            states[status.state] = states.get(status.state, 0) + 1
        if self.statuses:
            first_status = f'{stamp_s(self.statuses[0].header.stamp) - t0:.2f} s'
        else:
            first_status = 'none'
        last_reset = self.statuses[-1].reset_count if self.statuses else '-'
        missing_runs = ', '.join(f'{a:.1f}-{b:.1f}' if a != b else f'{a:.1f}' for a, b in runs)
        return (
            f"missing frames (s from the first frame): {missing_runs or 'none'}; "
            f'{len(self.statuses)} statuses, first at {first_status}, '
            f'by state {dict(sorted(states.items()))}, last reset_count {last_reset}')

    def tf_pairs(self):
        """
        Return the transforms of the stamps that have both map->odom and odom->base_link.

        As {stamp: {(parent, child): transform}}.
        """
        by_stamp = {}
        for tf in self.transforms:
            stamp = (tf.header.stamp.sec, tf.header.stamp.nanosec)
            by_stamp.setdefault(stamp, {})[(tf.header.frame_id, tf.child_frame_id)] = tf.transform
        return {
            s: d for s, d in by_stamp.items()
            if ('map', 'odom') in d and ('odom', 'base_link') in d}

    def tf_composes_to_pose(self, pairs, pose):
        """Tell whether map->odom * odom->base_link at the pose stamp equals the pose (1e-4)."""
        d = pairs.get((pose.header.stamp.sec, pose.header.stamp.nanosec))
        if d is None:
            return False
        a, b = d[('map', 'odom')], d[('odom', 'base_link')]
        ra = quat_to_matrix(a.rotation.x, a.rotation.y, a.rotation.z, a.rotation.w)
        rb = quat_to_matrix(b.rotation.x, b.rotation.y, b.rotation.z, b.rotation.w)
        tb = (b.translation.x, b.translation.y, b.translation.z)
        ta = (a.translation.x, a.translation.y, a.translation.z)
        composed_t = [sum(ra[i][k] * tb[k] for k in range(3)) + v for i, v in enumerate(ta)]
        composed_r = matmul(ra, rb)
        q, p = pose.pose.pose.orientation, pose.pose.pose.position
        rp = quat_to_matrix(q.x, q.y, q.z, q.w)
        rot_error = max(abs(composed_r[i][j] - rp[i][j]) for i in range(3) for j in range(3))
        return math.dist(composed_t, (p.x, p.y, p.z)) < 1e-4 and rot_error < 1e-4


__all__ = ['State', 'Transition', 'LocalizationStatus', 'PoseWithCovarianceStamped']
