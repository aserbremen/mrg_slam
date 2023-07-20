import os

import rclpy
from rclpy.node import Node

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

import numpy as np


# https://answers.ros.org/question/358686/how-to-read-a-bag-file-in-ros2/
class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        # create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of: type_of for id_of, name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
        self.topic_msg_message = {name_of: get_message(type_of) for id_of, name_of, type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp, data in rows]


class RosbagProcessor(Node):
    def __init__(self) -> None:
        super().__init__('rosbag_processor')

        self.playback_rate = self.declare_parameter('playback_rate', 1).get_parameter_value().integer_value
        self.robot_name = self.declare_parameter('robot_name', 'husky1').get_parameter_value().string_value
        self.dataset_dir = self.declare_parameter('dataset_dir', '').get_parameter_value().string_value

        if self.dataset_dir == '':
            print('Please specify the dataset directory parameter <dataset_dir> like this: --ros-args -p dataset_dir:=/path/to/dataset')
            exit(1)

        self.keyed_scan_bag_path = os.path.join(self.dataset_dir, 'rosbag', self.robot_name, self.robot_name + '.db3')
        self.odometry_bag_path = os.path.join(self.dataset_dir, 'ground_truth', self.robot_name + '_odom', self.robot_name + '_odom.db3')
        if not os.path.exists(self.keyed_scan_bag_path):
            print('Keyed scan bag does not exist: {}'.format(self.keyed_scan_bag_path))
            exit(1)
        if not os.path.exists(self.odometry_bag_path):
            print('Odometry bag does not exist: {}'.format(self.odometry_bag_path))
            exit(1)

        keyed_scan_parser = BagFileParser(self.keyed_scan_bag_path)
        keyed_scans_topic_name = '/' + self.robot_name + '/lamp/keyed_scans'
        print("Trying to get all messages from ros2 bag {} with topic name {}".format(self.keyed_scan_bag_path,  keyed_scans_topic_name))
        self.keyed_scans_msgs = keyed_scan_parser.get_messages(keyed_scans_topic_name)
        self.keyed_scands_stamps = np.array([msg[0] for msg in self.keyed_scans_msgs])

        odometry_parser = BagFileParser(self.odometry_bag_path)
        odometry_topic_name = '/' + self.robot_name + '/lo_frontend/odometry'
        print("Trying to get all messages from ros2 bag {} with topic name {}".format(self.odometry_bag_path,  odometry_topic_name))
        self.odometry_msgs = odometry_parser.get_messages(odometry_topic_name)
        # odometry msg stamp is given in the header
        self.odometry_stamps = np.array([msg[1].header.stamp.sec * 1e9 + msg[1].header.stamp.nanosec for msg in self.odometry_msgs])

        self.keyed_scan_counter = 0
        self.odometry_counter = 0

        point_cloud2_topic_name = '/' + self.robot_name + '/prefiltering/filtered_points'
        self.point_cloud2_publisher = self.create_publisher(PointCloud2, point_cloud2_topic_name, 10)
        print('Publishing PointCloud2 messages on topic {}'.format(point_cloud2_topic_name))

        odometry_topic_name = '/' + self.robot_name + '/scan_matching_odometry/odom'
        self.odometry_publisher = self.create_publisher(Odometry,  odometry_topic_name, 10)
        print('Publishing Odometry messages on topic {}'.format(odometry_topic_name))

        self.clock_publisher = self.create_publisher(Clock, '/clock', 10)
        print('Publishing Clock messages on topic /clock')

        print('Starting playback with rate {}'.format(self.playback_rate))
        self.timer = self.create_timer(1.0 / self.playback_rate, self.process_rosbags)

    def process_rosbags(self):
        # Get the pointcloud and the corresponding odometry message with the closest timestamp
        pointcloud_stamp = self.keyed_scans_msgs[self.keyed_scan_counter][0]
        closest_odometry_index = np.argmin(np.abs(self.odometry_stamps - pointcloud_stamp))
        odometry_stamp = self.odometry_stamps[closest_odometry_index]

        print('Pointcloud stamp {} closest odometry stamp {}: delta t {}s'.format(
            pointcloud_stamp, int(odometry_stamp), (odometry_stamp - pointcloud_stamp) / 1e9))

        pointcloud = self.keyed_scans_msgs[self.keyed_scan_counter][1].scan
        odometry = self.odometry_msgs[closest_odometry_index][1]
        # Publish the corresponding pointcloud and odometry message
        if pointcloud.header.frame_id == '':
            pointcloud.header.frame_id = self.robot_name + '/velodyne'
        odometry.child_frame_id = self.robot_name + '/base_link'

        # Set the header stamp of pointcloud message
        pointcloud.header.stamp.sec = int(str(pointcloud_stamp)[:len(str(pointcloud_stamp))-9])
        pointcloud.header.stamp.nanosec = int(str(pointcloud_stamp)[len(str(pointcloud_stamp))-9:])

        # Since we are not using a rosbag2 player, we need to publish the clock message ourselves
        clock_msg = Clock()
        clock_msg.clock.sec = pointcloud.header.stamp.sec
        clock_msg.clock.nanosec = pointcloud.header.stamp.nanosec
        self.clock_publisher.publish(clock_msg)

        # Publish the matching pointcloud and odometry message
        self.point_cloud2_publisher.publish(pointcloud)
        self.odometry_publisher.publish(odometry)

        self.keyed_scan_counter += 1


def main(args=None):
    rclpy.init(args=args)

    ros_bag_processor = RosbagProcessor()
    try:
        rclpy.spin(ros_bag_processor)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        ros_bag_processor.destroy_node()


if __name__ == '__main__':
    main()
