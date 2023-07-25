import os
import fire
import time

import rclpy
from rclpy.node import Node

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

import numpy as np
import math
import matplotlib.pyplot as plt
from pyquaternion import Quaternion


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


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

        self.playback_rate = self.declare_parameter('rate', 1.0).get_parameter_value().double_value
        self.robot_names = self.declare_parameter('robot_names', ['husky1']).get_parameter_value().string_array_value
        self.dataset_dir = self.declare_parameter('dataset_dir', '').get_parameter_value().string_value
        self.enable_floor_detetction = self.declare_parameter('enable_floor_detetction', False).get_parameter_value().bool_value

        if self.dataset_dir == '':
            print('Please specify the dataset directory parameter <dataset_dir> like this: --ros-args -p dataset_dir:=/path/to/dataset')
            exit(1)

        self.data_dict = {}
        self.setup_playback()

    def setup_playback(self):
        print('Setting up playback for robots: {}'.format(self.robot_names))
        for robot_name in self.robot_names:
            self.setup_robot(robot_name)

        self.clock_publisher = self.create_publisher(Clock, '/clock', 10)
        print('Setting up Clock pubilsher on topic /clock')

    def setup_robot(self, robot_name):

        keyed_scan_bag_path = os.path.join(self.dataset_dir, 'rosbag', robot_name, robot_name + '.db3')
        odometry_bag_path = os.path.join(self.dataset_dir, 'ground_truth', robot_name + '_odom', robot_name + '_odom.db3')
        if not os.path.exists(keyed_scan_bag_path):
            print('Keyed scan bag does not exist: {}'.format(keyed_scan_bag_path))
            exit(1)
        if not os.path.exists(odometry_bag_path):
            print('Odometry bag does not exist: {}'.format(odometry_bag_path))
            exit(1)

        keyed_scans_parser = BagFileParser(keyed_scan_bag_path)
        keyed_scans_topic_name = '/' + robot_name + '/lamp/keyed_scans'
        print("Trying to get all messages from ros2 bag {} with topic name {}".format(keyed_scan_bag_path,  keyed_scans_topic_name))
        scans_msgs = keyed_scans_parser.get_messages(keyed_scans_topic_name)
        # Add the keyed scans data to the data dict
        self.data_dict[robot_name] = {}
        self.data_dict[robot_name]['scans_msgs'] = scans_msgs
        self.data_dict[robot_name]['scans_stamps'] = np.array([msg[0] for msg in scans_msgs])

        odometry_parser = BagFileParser(odometry_bag_path)
        odometry_topic_name = '/' + robot_name + '/lo_frontend/odometry'
        print("Trying to get all messages from ros2 bag {} with topic name {}".format(odometry_bag_path,  odometry_topic_name))
        odometry_msgs = odometry_parser.get_messages(odometry_topic_name)
        # odometry msg stamp is given in the header
        odometry_stamps = np.array([int(msg[1].header.stamp.sec * 1e9 + msg[1].header.stamp.nanosec) for msg in odometry_msgs])
        # Add the odometry to the data dict
        self.data_dict[robot_name]['odometry_msgs'] = odometry_msgs
        self.data_dict[robot_name]['odometry_stamps'] = odometry_stamps

        self.data_dict[robot_name]['scan_counter'] = 0
        point_cloud2_topic_name = '/' + robot_name + '/prefiltering/filtered_points'
        self.data_dict[robot_name]['point_cloud2_publisher'] = self.create_publisher(PointCloud2, point_cloud2_topic_name, 10)
        print('Setting up PointCloud2 publisher on topic {}'.format(point_cloud2_topic_name))
        odometry_topic_name = '/' + robot_name + '/scan_matching_odometry/odom'
        self.data_dict[robot_name]['odometry_publisher'] = self.create_publisher(Odometry,  odometry_topic_name, 10)
        print('Setting up Odometry publisher on topic {}'.format(odometry_topic_name))

        # Setup a filtered_points publisher for floor detection
        if self.enable_floor_detetction:
            self.data_dict[robot_name]['filtered_points_publisher'] = self.create_publisher(
                PointCloud2, '/' + robot_name + '/prefiltering/filtered_points', 10)

    def start_playback(self):
        print('Starting playback with rate {}'.format(self.playback_rate))
        self.timer = self.create_timer(1.0 / self.playback_rate, self.process_rosbags)

    def print_initial_poses(self):

        for robot_name in self.robot_names:

            # pointcloud_stamp = self.scans_msgs[self.scan_counter][0]
            pointcloud_stamp = self.data_dict[robot_name]['scans_stamps'][0]
            closest_odometry_index = np.argmin(np.abs(self.data_dict[robot_name]['odometry_stamps'] - pointcloud_stamp))

            initial_odom = self.data_dict[robot_name]['odometry_msgs'][closest_odometry_index][1]
            position = initial_odom.pose.pose.position
            orientation = initial_odom.pose.pose.orientation
            euler = euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
            print('Robot {} initial pose:'.format(robot_name))
            print('position    {} {} {}'.format(position.x, position.y, position.y))
            print('orientation {} {} {} {}'.format(orientation.x, orientation.y, orientation.z, orientation.w))
            print('euler rpy   {} {} {}'.format(euler[0], euler[1], euler[2]))
            print("Convenient format:")
            print('x: {}\ny: {}\nz: {}\n'.format(position.x, position.y, position.z))
            print('qx: {}\nqy: {}\nqz: {}\nqw: {}\n'.format(orientation.x, orientation.y, orientation.z, orientation.w))
            print('roll: {}\npitch: {}\nyaw: {}\n'.format(euler[0], euler[1], euler[2]))

        exit(0)

    def process_rosbags(self):
        # Get the robot name with the lowest timestamp
        robot_name = min(
            self.data_dict, key=lambda k: self.data_dict[k]['scans_stamps'][self.data_dict[k]['scan_counter']]
            if self.data_dict[k]['scan_counter'] < len(self.data_dict[k]['scans_stamps']) else float('inf'))
        print(robot_name, self.data_dict[robot_name]['scan_counter'], self.data_dict[robot_name]
              ['scans_stamps'][self.data_dict[robot_name]['scan_counter']])

        # Get the pointcloud and the corresponding odometry message with the closest timestamp
        pointcloud_stamp = self.data_dict[robot_name]['scans_stamps'][self.data_dict[robot_name]['scan_counter']]
        closest_odometry_index = np.argmin(np.abs(self.data_dict[robot_name]['odometry_stamps'] - pointcloud_stamp))
        odometry_stamp = self.data_dict[robot_name]['odometry_stamps'][closest_odometry_index]

        print('Pointcloud stamp {} closest odometry stamp {}: delta t {}s'.format(
            pointcloud_stamp, int(odometry_stamp), (odometry_stamp - pointcloud_stamp) / 1e9))

        pointcloud = self.data_dict[robot_name]['scans_msgs'][self.data_dict[robot_name]['scan_counter']][1].scan
        odometry = self.data_dict[robot_name]['odometry_msgs'][closest_odometry_index][1]
        # Publish the corresponding pointcloud and odometry message
        if pointcloud.header.frame_id == '':
            pointcloud.header.frame_id = robot_name + '/velodyne'
        odometry.child_frame_id = robot_name + '/base_link'

        # Set the header stamp of pointcloud message
        pointcloud.header.stamp.sec = int(str(pointcloud_stamp)[:len(str(pointcloud_stamp))-9])
        pointcloud.header.stamp.nanosec = int(str(pointcloud_stamp)[len(str(pointcloud_stamp))-9:])

        # Since we are not using a rosbag2 player, we need to publish the clock message ourselves
        clock_msg = Clock()
        clock_msg.clock.sec = pointcloud.header.stamp.sec
        clock_msg.clock.nanosec = pointcloud.header.stamp.nanosec
        self.clock_publisher.publish(clock_msg)

        # Publish the matching pointcloud and odometry message
        if self.enable_floor_detetction:
            self.data_dict[robot_name]['filtered_points_publisher'].publish(pointcloud)
            # sleep for 0.5 seconds to give the floor detection node time to process the pointcloud
            time.sleep(0.5)

        self.data_dict[robot_name]['point_cloud2_publisher'].publish(pointcloud)
        self.data_dict[robot_name]['odometry_publisher'].publish(odometry)

        self.data_dict[robot_name]['scan_counter'] += 1

        # Exit if all keyed scans have been processed
        if all(self.data_dict[k]['scan_counter'] == len(self.data_dict[k]['scans_stamps']) for k in self.data_dict):
            print('Finished processing all messages from the rosbag')
            exit(0)

    def plot_trajectories(self):

        # Create a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_zlabel('z (m)')
        ax.set_title('Trajectories')

        for robot_name in self.robot_names:
            # Set the color according to tableau palette
            color = 'C' + str(self.robot_names.index(robot_name))
            odom_xyz = np.array([[odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z]
                                for stamp, odom in self.data_dict[robot_name]['odometry_msgs']])

            ax.plot(odom_xyz[:, 0], odom_xyz[:, 1], odom_xyz[:, 2], color=color, label=robot_name)
            # Plot text 'start' and 'end' at the start and end of the trajectory
            z_text_offset = 0.05
            text_size = 15
            ax.text(odom_xyz[0, 0], odom_xyz[0, 1], odom_xyz[0, 2]+z_text_offset, 'start', color=color, size=text_size)
            ax.text(odom_xyz[-1, 0], odom_xyz[-1, 1], odom_xyz[-1, 2]+z_text_offset, 'end', color=color, size=text_size)

        plt.legend()
        plt.show()

        exit(0)

    def print_dataset_info(self):
        for robot_name in self.robot_names:
            print('Robot {}'.format(robot_name))
            print('Number of pointclouds: {}'.format(len(self.data_dict[robot_name]['scans_stamps'])))
            print('Number of odometry messages: {}'.format(len(self.data_dict[robot_name]['odometry_stamps'])))

            num_points_per_scan = np.array([keyed_scan.scan.height * keyed_scan.scan.row_step / 16 for stamp,
                                            keyed_scan in self.data_dict[robot_name]['scans_msgs']])
            print('Average number of points per pointcloud: {}'.format(np.mean(num_points_per_scan)))

            keyframe_odom_indices = [np.argmin(np.abs(self.data_dict[robot_name]['odometry_stamps'] - pcl_stamp))
                                     for pcl_stamp in self.data_dict[robot_name]['scans_stamps']]
            odom_xyz = np.array([[odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z]
                                for stamp, odom in np.take(self.data_dict[robot_name]['odometry_msgs'], keyframe_odom_indices, axis=0)])
            odom_xyz_norms = np.linalg.norm(odom_xyz[1:, :] - odom_xyz[:-1, :], axis=1)
            print('Average keyframe distance {}'.format(np.mean(odom_xyz_norms)))
            print('Max keyframe distance {}'.format(np.max(odom_xyz_norms)))
            print('Min keyframe distance {}'.format(np.min(odom_xyz_norms)))

            # Need w, x, y, z for pyquaternion
            odom_orientations = np.array([
                [odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z]
                for stamp, odom in np.take(self.data_dict[robot_name]['odometry_msgs'],
                                           keyframe_odom_indices, axis=0)])
            delta_rots = np.array([Quaternion.absolute_distance(Quaternion(q1[0], q1[1], q1[2], q1[3]), Quaternion(
                q2[0], q2[1], q2[2], q2[3])) for q1, q2 in zip(odom_orientations[:-1, :], odom_orientations[1:, :])])
            print('Average keyframe rotation {}°'.format(np.rad2deg(np.mean(delta_rots))))
            print('Max keyframe rotation {}°'.format(np.rad2deg(np.max(delta_rots))))
            print('Min keyframe rotation {}°'.format(np.rad2deg(np.min(delta_rots))))

        exit(0)


def play_rosbags(args=None):
    rclpy.init(args=args)

    ros_bag_processor = RosbagProcessor()
    ros_bag_processor.start_playback()
    spin(ros_bag_processor)


def print_initial_poses(args=None):
    rclpy.init(args=args)

    ros_bag_processor = RosbagProcessor()
    ros_bag_processor.print_initial_poses()
    spin(ros_bag_processor)


def plot_trajectories(args=None):
    rclpy.init(args=args)

    ros_bag_processor = RosbagProcessor()
    ros_bag_processor.plot_trajectories()
    spin(ros_bag_processor)


def print_dataset_info(args=None):
    rclpy.init(args=args)

    ros_bag_processor = RosbagProcessor()
    ros_bag_processor.print_dataset_info()
    spin(ros_bag_processor)


def spin(node):
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        node.destroy_node()


def main(args=None):
    fire.Fire()


if __name__ == '__main__':
    main()
