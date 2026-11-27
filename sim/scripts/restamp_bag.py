#!/usr/bin/env python3
"""Copy a sim_eval bag with the log time of every message set to its header stamp.

  restamp_bag.py <bag> <out bag>

record.sh records faster than real time, so the log times of its bag are wall times of the recorder
while the header stamps are simulation time from 0. `ros2 bag play --clock` publishes /clock from the
log time; after this copy, /clock, the header stamps and use_sim_time nodes agree. Every topic of the
bag has a std_msgs/Header. The output is MCAP.
"""
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    src, dst = sys.argv[1], sys.argv[2]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=src, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topics = reader.get_all_topics_and_types()
    types = {t.name: get_message(t.type) for t in topics}
    msgs = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        stamp = deserialize_message(data, types[topic]).header.stamp
        msgs.append((stamp.sec * 1000000000 + stamp.nanosec, topic, data))
    msgs.sort(key=lambda m: m[0])
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=dst, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for i, t in enumerate(topics):
        writer.create_topic(rosbag2_py.TopicMetadata(i, t.name, t.type, 'cdr'))
    for stamp, topic, data in msgs:
        writer.write(topic, data, stamp)
    print(f'restamp_bag: {len(msgs)} messages, {msgs[0][0] * 1e-9:.3f} to {msgs[-1][0] * 1e-9:.3f} s')


if __name__ == '__main__':
    main()
