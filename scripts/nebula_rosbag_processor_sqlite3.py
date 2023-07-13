import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from pose_graph_msgs.msg import KeyedScan
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


# https://answers.ros.org/question/358686/how-to-read-a-bag-file-in-ros2/
class RosbagProcessor(Node):
    def __init__(self) -> None:
        super().__init__('rosbag_processor')

        self.playback_rate = self.declare_parameter('playback_rate', 1).get_parameter_value().integer_value
        self.robot_name = self.declare_parameter('robot_name', 'husky1').get_parameter_value().string_value
        self.dataset_dir = self.declare_parameter('dataset_dir', '').get_parameter_value().string_value
        if self.dataset_dir == '':
            print('Please specify the dataset directory parameter <dataset_dir>')
            exit(1)

        self.keyed_scan_bag_path = os.path.join(self.dataset_dir, 'rosbag', self.robot_name, self.robot_name + '.db3')
        self.odometry_bag_path = os.path.join(self.dataset_dir, 'ground_truth', self.robot_name + '_odom' + self.robot_name + '_odom.db3')
        if not os.path.exists(self.keyed_scan_bag_path):
            print('Keyed scan bag does not exist: {}'.format(self.keyed_scan_bag_path))
        if not os.path.exists(self.odometry_bag_path):
            print('Odometry bag does not exist: {}'.format(self.odometry_bag_path))

        # generator objects
        self.keyed_scans_connection = sqlite3.connect(self.keyed_scan_bag_path)
        self.odometry_connection = sqlite3.connect(self.odometry_bag_path)
        self.keyed_scans_cursor = self.keyed_scans_connection.cursor()
        self.odometry_cursor = self.odometry_connection.cursor()

        # create a message type map
        keyed_scans_topics_data = self.cursor.execute('SELECT ID, name, type FROM topics').fetchall()
        self.keyed_scans_topic_type = {name_of: type_of for id_of, name_of, type_of in keyed_scans_topics_data}
        self.keyed_scans_topic_id = {name_of: id_of for id_of, name_of, type_of in keyed_scans_topics_data}
        self.keyed_scans_topic_msg_message = {name_of: get_message(type_of) for id_of, name_of, type_of in keyed_scans_topics_data}

        odometry_topics_data = self.cursor.execute('SELECT ID, name, type FROM topics').fetchall()
        self.odometry_topic_type = {name_of: type_of for id_of, name_of, type_of in odometry_topics_data}
        self.odometry_topic_id = {name_of: id_of for id_of, name_of, type_of in odometry_topics_data}
        self.odometry_topic_msg_message = {name_of: get_message(type_of) for id_of, name_of, type_of in odometry_topics_data}

        # self.keyed_scan_msgs = get_msgs_from_bag(self.keyed_scan_bag_path, 'keyed_scans')
        # self.odometry_msgs = get_msgs_from_bag(self.odometry_bag_path, 'odometry')
        self.point_cloud2_publisher = self.create_publisher(PointCloud2, '/velodyne_points', 10)
        self.odometry_publisher = self.create_publisher(Odometry, '/odometry', 10)

        self.timer = self.create_timer(1.0 / self.playback_rate, self.process_rosbags)

    # def __del___(self):
    #     self.keyed_scans_connection.close()
    #     self.odometry_connection.close()

    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        if topic_id is None:
            return
        # Get from the db
        rows = self.

def main(args=None):
    rclpy.init(args=args)

    ros_bag_processor = RosbagProcessor()

    ros_bag_processor.get_messages('keyed_scans')


    try:
        rclpy.spin(ros_bag_processor)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        ros_bag_processor.destroy_node()


if __name__ == '__main__':
    main()
