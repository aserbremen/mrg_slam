import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from rosbags.typesys import get_types_from_msg, register_types

from pose_graph_msgs.msg import KeyedScan
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

# Register the custom message types
CUSTOM_MESSAGE_PATHS = [
    '/home/andi/code/cpp/ros2_ws_hdl_graph_slam/src/lamp/pose_graph_msgs/msg/KeyedScan.msg'
]


class RosbagProcessor(Node):
    def __init__(self) -> None:
        super().__init__('rosbag_processor')

        self.playback_rate = self.declare_parameter('playback_rate', 1).get_parameter_value().integer_value
        self.robot_name = self.declare_parameter('robot_name', 'husky1').get_parameter_value().string_value
        self.dataset_dir = self.declare_parameter('dataset_dir', '').get_parameter_value().string_value
        if self.dataset_dir == '':
            print('Please specify the dataset directory parameter <dataset_dir>')
            exit(1)

        self.keyed_scan_bag_dir = os.path.join(self.dataset_dir, 'rosbag', self.robot_name)
        self.odometry_bag_dir = os.path.join(self.dataset_dir, 'ground_truth', self.robot_name + '_odom')
        if not os.path.exists(self.keyed_scan_bag_dir):
            print('Keyed scan bag directory does not exist: {}'.format(self.keyed_scan_bag_dir))
        if not os.path.exists(self.odometry_bag_dir):
            print('Odometry bag directory does not exist: {}'.format(self.odometry_bag_dir))

        # generator objects
        self.keyed_scan_msgs = get_msgs_from_bag(self.keyed_scan_bag_dir, 'keyed_scans')
        self.odometry_msgs = get_msgs_from_bag(self.odometry_bag_dir, 'odometry')
        self.point_cloud2_publisher = self.create_publisher(PointCloud2, '/velodyne_points', 10)
        self.odometry_publisher = self.create_publisher(Odometry, '/odometry', 10)

        self.timer = self.create_timer(1.0 / self.playback_rate, self.process_rosbags)

    def process_rosbags(self):

        for keyed_scan_msg in self.keyed_scan_msgs:
            # print(keyed_scan_msg)
            pointcloud = keyed_scan_msg.scan
            print(pointcloud)
            # Problem you cannot publish a PointCloud2 because conversion between rosbags.types and ROS2 messages is unclear
            # self.point_cloud2_publisher.publish(pointcloud2)
            # pointcloud_stamp = pointcloud.header.stamp
            # print(pointcloud_stamp)


def register_custom_msgs():
    add_types = {}

    for custom_message_path in CUSTOM_MESSAGE_PATHS:
        msgpath = Path(custom_message_path)
        msgdef = msgpath.read_text(encoding='utf-8')
        add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

    register_types(add_types)


def guess_msgtype(path: Path) -> str:
    """Guess the message type from the path of the message definition."""
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)


def check_rosbag(bag_dir, topic_name):
    with Reader(bag_dir) as reader:
        # topic and message types
        for connection in reader.connections:
            print(connection.topic, connection.msgtype)

        for connection, timestamp, rawdata in reader.messages():
            if topic_name in connection.topic:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                print(msg)


def get_msgs_from_bag(bag_dir, topic_name):
    with Reader(bag_dir) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if topic_name in connection.topic:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                yield msg


def main(args=None):
    register_custom_msgs()
    rclpy.init(args=args)

    ros_bag_processor = RosbagProcessor()
    try:
        rclpy.spin(ros_bag_processor)
    except KeyboardInterrupt:
        pass
    finally:
        ros_bag_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
