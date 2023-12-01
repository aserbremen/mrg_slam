import os
import fire
import time

import rclpy
from rclpy.node import Node
import rclpy.logging
from tf2_ros import TransformBroadcaster
from rclpy.callback_groups import ReentrantCallbackGroup

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from vamex_slam_msgs.msg import SlamStatus

import numpy as np
import math
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R


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
        # For analysing pointcloud data
        self.sensor_heights = self.declare_parameter('sensor_heights', [0.7]).get_parameter_value().double_array_value
        self.sensor_clip_range = self.declare_parameter('sensor_clip_range', 1.0).get_parameter_value().double_value
        self.result_folder = self.declare_parameter('result_folder', '').get_parameter_value().string_value

        # The slam status callback and the timer callback need to be reentrant, so that the slam status can be updated while the timer is processed
        self.reentrant_callback_group = ReentrantCallbackGroup()

        self.tf_broadcaster = TransformBroadcaster(self)

        # self.setup_playback()

    def setup_playback(self):
        if self.dataset_dir == '':
            print('Please specify the dataset directory parameter <dataset_dir> like this: --ros-args -p dataset_dir:=/path/to/dataset')
            exit(1)

        self.data_dict = {}
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

        # Create the subscription to the slam status in order to stop playback when the algorithms are optimizing or loop closing
        slam_status_topic_name = '/' + robot_name + '/hdl_graph_slam/slam_status'
        self.data_dict[robot_name]['slam_status_subscription'] = self.create_subscription(
            SlamStatus, slam_status_topic_name, self.slam_status_callback, 10, callback_group=self.reentrant_callback_group)
        self.data_dict[robot_name]['slam_status'] = SlamStatus()

        # Setup a filtered_points publisher for floor detection
        if self.enable_floor_detetction:
            self.data_dict[robot_name]['filtered_points_publisher'] = self.create_publisher(
                PointCloud2, '/' + robot_name + '/prefiltering/filtered_points', 10)

    def start_playback(self):
        print('Starting playback with rate {}'.format(self.playback_rate))
        self.timer = self.create_timer(1.0 / self.playback_rate, self.process_rosbags, callback_group=self.reentrant_callback_group)
        self.print_wait_info_once = True

    def print_initial_poses(self):

        for robot_name in self.robot_names:
            pointcloud_stamp = self.data_dict[robot_name]['scans_stamps'][0]
            closest_odometry_index = np.argmin(np.abs(self.data_dict[robot_name]['odometry_stamps'] - pointcloud_stamp))

            initial_odom = self.data_dict[robot_name]['odometry_msgs'][closest_odometry_index][1]
            position = initial_odom.pose.pose.position
            orientation = initial_odom.pose.pose.orientation
            euler = euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
            print('Robot {} initial pose:'.format(robot_name))
            print('position    {:.3f} {:.3f} {:.3f}'.format(position.x, position.y, position.y))
            print('orientation {:.3f} {:.3f} {:.3f} {:.3f}'.format(orientation.x, orientation.y, orientation.z, orientation.w))
            print('euler rpy   {:.3f} {:.3f} {:.3f}'.format(euler[0], euler[1], euler[2]))
            print("Convenient format:")
            print('x: {:.3f}\ny: {:.3f}\nz: {:.3f}\n'.format(position.x, position.y, position.z))
            print('qx: {:.3f}\nqy: {:.3f}\nqz: {:.3f}\nqw: {:.3f}\n'.format(orientation.x, orientation.y, orientation.z, orientation.w))
            print('roll: {:.3f}\npitch: {:.3f}\nyaw: {:.3f}\n'.format(euler[0], euler[1], euler[2]))

        exit(0)

    def publish_transform(self, stamp, frame_id, child_frame_id,  translation, rotation):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]
        self.tf_broadcaster.sendTransform(t)

    def process_rosbags(self):
        # Make sure that this timer is only executed once, reset the timer at the end of this function
        self.timer.cancel()
        # Get the robot name with the lowest timestamp
        robot_name = min(
            self.data_dict, key=lambda k: self.data_dict[k]['scans_stamps'][self.data_dict[k]['scan_counter']]
            if self.data_dict[k]['scan_counter'] < len(self.data_dict[k]['scans_stamps']) else float('inf'))

        # Get the pointcloud and the corresponding odometry message with the closest timestamp
        pointcloud_stamp = self.data_dict[robot_name]['scans_stamps'][self.data_dict[robot_name]['scan_counter']]
        closest_odometry_index = np.argmin(np.abs(self.data_dict[robot_name]['odometry_stamps'] - pointcloud_stamp))
        odometry_stamp = self.data_dict[robot_name]['odometry_stamps'][closest_odometry_index]

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

        # Publish the tf2 transform between model_namespace/odom and model_namespace/base_link (model_namespace/velodyne) as both conincide
        # This is needed for the floor detection output visulization
        self.publish_transform(pointcloud.header.stamp, robot_name + '/odom', robot_name + '/base_link',
                               odometry.pose.pose.position, odometry.pose.pose.orientation)
        # t = TransformStamped()
        # t.header.stamp = pointcloud.header.stamp
        # t.header.frame_id = robot_name + '/odom'
        # t.child_frame_id = robot_name + '/base_link'
        # t.transform.translation.x = odometry.pose.pose.position.x
        # t.transform.translation.y = odometry.pose.pose.position.y
        # t.transform.translation.z = odometry.pose.pose.position.z
        # t.transform.rotation.x = odometry.pose.pose.orientation.x
        # t.transform.rotation.y = odometry.pose.pose.orientation.y
        # t.transform.rotation.z = odometry.pose.pose.orientation.z
        # t.transform.rotation.w = odometry.pose.pose.orientation.w
        # self.tf_broadcaster.sendTransform(t)

        while any(self.data_dict[k]['slam_status'].in_optimization or
                  self.data_dict[k]['slam_status'].in_loop_closure or
                  self.data_dict[k]['slam_status'].in_graph_exchange for k in self.data_dict):
            if self.print_wait_info_once:
                print('Waiting for slam to finish optimizing or loop closing')
            self.timer.reset()
            self.print_wait_info_once = False
            return
        self.print_wait_info_once = True

        if self.enable_floor_detetction:
            self.data_dict[robot_name]['filtered_points_publisher'].publish(pointcloud)
            # sleep for some time to give the floor detection node time to process the pointcloud
            time.sleep(0.3)

        print('{} scan #{}/{} stamp {:.3f} odom stamp {:.3f}: delta t {:.3f}s, publishing scan, odom'.format(
            robot_name, self.data_dict[robot_name]['scan_counter'], len(self.data_dict[robot_name]['scans_stamps']) - 1,
            pointcloud_stamp / 1e9, odometry_stamp / 1e9, (pointcloud_stamp - odometry_stamp) / 1e9))

        # Publish the matching pointcloud and odometry message
        self.data_dict[robot_name]['point_cloud2_publisher'].publish(pointcloud)
        self.data_dict[robot_name]['odometry_publisher'].publish(odometry)

        self.data_dict[robot_name]['scan_counter'] += 1

        # Reset the timer so we can proceed processing the next message
        self.timer.reset()

        # Exit if all keyed scans have been processed
        if all(self.data_dict[k]['scan_counter'] == len(self.data_dict[k]['scans_stamps']) for k in self.data_dict):
            print('Finished processing all messages from the rosbag')
            self.timer.destroy()
            exit(0)

    def slam_status_callback(self, msg):
        self.data_dict[msg.robot_name]['slam_status'] = msg

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
            # Plot text at regular intervals along the trajectory
            accum_distances = np.cumsum(np.linalg.norm(odom_xyz[1:, :] - odom_xyz[:-1, :], axis=1))
            interval_percent = 0.05
            interval_distances = np.arange(0, accum_distances[-1], interval_percent * accum_distances[-1])
            # Find the indices of the odom_xyz array that are closest to the interval_distances
            closest_odom_indices = np.argmin(np.abs(accum_distances[:, np.newaxis] - interval_distances), axis=0)
            for i, odom_idx in enumerate(closest_odom_indices):
                if i == 0:
                    continue
                ax.text(odom_xyz[odom_idx, 0], odom_xyz[odom_idx, 1], odom_xyz[odom_idx, 2]+z_text_offset,
                        '{:.0f}%'.format(i * interval_percent*100), color=color, size=text_size)

        plt.legend()
        plt.show()

        exit(0)

    def print_info(self):
        self.setup_playback()
        for robot_name in self.robot_names:
            print('\nRobot {}'.format(robot_name))
            print('Number of pointclouds: {}'.format(len(self.data_dict[robot_name]['scans_stamps'])))
            print('Number of odometry messages: {}'.format(len(self.data_dict[robot_name]['odometry_stamps'])))

            # Print some point cloud statistics
            points_dict = {index: read_points(scans_msg[1].scan, field_names=['x', 'y', 'z'], reshape_organized_cloud=True)
                           for index, scans_msg in enumerate(self.data_dict[robot_name]['scans_msgs'])}
            # Reorganize the points into a numpy array
            for index, points_tuple in points_dict.items():
                points_dict[index] = np.array([np.array([point[0], point[1], point[2]]) for point in points_tuple])
            num_points_per_scan = np.array([points.shape[0] for index, points in points_dict.items()])
            print('Average number of points per pointcloud: {:.2f}'.format(np.mean(num_points_per_scan)))
            print('Std deviation of number of points per pointcloud: {:.2f}'.format(np.std(num_points_per_scan)))

            # Print height range statistics
            if len(self.robot_names) != len(self.sensor_heights):
                rclpy.logging.get_logger('rosbag_processor').warn(
                    'Number of robot names and sensor heights do not match, taking first sensor height for all robots')
                sensor_height = self.sensor_heights[0]
            else:
                sensor_height = self.sensor_heights[self.robot_names.index(robot_name)]
            z_min = sensor_height - self.sensor_clip_range
            z_max = sensor_height + self.sensor_clip_range
            num_points_in_range = [np.sum((points[:, 2] > z_min) & (points[:, 2] < z_max)) for index, points in points_dict.items()]
            print('Average nummber of points in height range {:.2f}m to {:.2f}m: {:.2f}'.format(z_min, z_max, np.mean(num_points_in_range)))

            # Print some keyframe statistics
            keyframe_odom_indices = [np.argmin(np.abs(self.data_dict[robot_name]['odometry_stamps'] - pcl_stamp))
                                     for pcl_stamp in self.data_dict[robot_name]['scans_stamps']]
            odom_xyz = np.array([[odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z]
                                for stamp, odom in np.take(self.data_dict[robot_name]['odometry_msgs'], keyframe_odom_indices, axis=0)])
            odom_xyz_norms = np.linalg.norm(odom_xyz[1:, :] - odom_xyz[:-1, :], axis=1)
            print('Average keyframe distance {:.2f}'.format(np.mean(odom_xyz_norms)))
            print('Max keyframe distance {:.2f}'.format(np.max(odom_xyz_norms)))
            print('Min keyframe distance {:.2f}'.format(np.min(odom_xyz_norms)))
            # calculate the traveled distance
            print('Total traveled distance {:.2f}'.format(np.sum(odom_xyz_norms)))

            # Need w, x, y, z for pyquaternion
            odom_orientations = np.array([
                [odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z]
                for stamp, odom in np.take(self.data_dict[robot_name]['odometry_msgs'],
                                           keyframe_odom_indices, axis=0)])
            delta_rots = np.array([Quaternion.absolute_distance(Quaternion(q1[0], q1[1], q1[2], q1[3]), Quaternion(
                q2[0], q2[1], q2[2], q2[3])) for q1, q2 in zip(odom_orientations[:-1, :], odom_orientations[1:, :])])
            print('Average keyframe rotation {:.2f}°'.format(np.rad2deg(np.mean(delta_rots))))
            print('Max keyframe rotation {:.2f}°'.format(np.rad2deg(np.max(delta_rots))))
            print('Min keyframe rotation {:.2f}°'.format(np.rad2deg(np.min(delta_rots))))

        exit(0)

    def write_odom_groundtruth(self):
        for robot_name in self.robot_names:
            nebula_dir = os.path.dirname(self.dataset_dir)
            if os.path.exists(os.path.join(nebula_dir, 'groundtruth')):
                dataset_name = os.path.basename(os.path.normpath(self.dataset_dir))
                gt_filepath = os.path.join(nebula_dir, 'groundtruth', dataset_name + '_' + robot_name + '.txt')
            else:
                gt_filepath = os.path.join(self.dataset_dir, 'groundtruth', robot_name + '_odom', 'stamped_groundtruth.txt')
            print('Writing ground truth odometry to {}'.format(gt_filepath))
            with open(gt_filepath, 'w') as f:
                for stamp, msg in self.data_dict[robot_name]['odometry_msgs']:
                    # we need to use the stamp from the header
                    original_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    f.write('{} {} {} {} {} {} {} {}\n'.format(
                        original_stamp, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

        exit(0)

    def results_to_pose_file(self):
        if self.result_folder == '':
            print('Please specify the result folder parameter <result_folder> like this: --ros-args -p result_folder:=/path/to/result_folder')
            exit(1)
        # get all filenames ending with .txt
        keyframes_folder = os.path.join(self.result_folder, 'keyframes')
        keyframe_files = [f for f in os.listdir(keyframes_folder) if f.endswith('.txt')]
        keyframe_files = sorted(keyframe_files)
        own_robot_name = None
        stamps_poses = []
        for keyframe_file in keyframe_files:
            with open(os.path.join(keyframes_folder, keyframe_file), mode='r') as f:
                lines = f.readlines()
                # Get the robot name from the first line
                robot_name_str = [line for line in lines if line.startswith('robot_name')]
                robot_name = robot_name_str[0].split(' ')[1]
                if own_robot_name is None:
                    own_robot_name = robot_name.rstrip()

                if own_robot_name not in robot_name:
                    continue

                stamp_str = next((line for line in lines if line.startswith('stamp')), None)
                stamp_secs = stamp_str.split(' ')[1]
                stamp_nanosecs = stamp_str.split(' ')[2]
                # remove new line character if present
                if stamp_nanosecs.endswith('\n'):
                    stamp_nanosecs = stamp_nanosecs[:-1]
                stamp_nanosecs = stamp_nanosecs.zfill(9)
                stamp = float(stamp_secs + '.' + stamp_nanosecs)

                # get the 4 lines after the one line containing 'estimate'
                estimate_str = next((line for line in lines if line.startswith('estimate')), None)
                estimate_lines = lines[lines.index(estimate_str)+1:lines.index(estimate_str)+5]
                T = np.genfromtxt(estimate_lines)
                translation = T[:3, 3]
                rotation = R.from_matrix(T[:3, :3]).as_quat()
                stamps_poses.append([stamp, translation, rotation])

        # Write the gathered data to a file
        short_robot_name = own_robot_name[0] + own_robot_name[-1]
        output_file = os.path.join(self.result_folder, short_robot_name + '.txt')
        print('Writing poses to file: {}'.format(os.path.abspath(output_file)))
        for stamp, translation, rotation in stamps_poses:
            with open(output_file, 'a') as f:
                f.write('{} {} {} {} {} {} {} {}\n'.format(
                    stamp, translation[0], translation[1], translation[2], rotation[0], rotation[1], rotation[2], rotation[3]))

        exit(0)

    def get_robot_name_from_first_edge(self, lines):
        # Get the line starting with from
        from_str = next((line for line in lines if line.startswith('from')), None)
        robot_name = from_str.split(' ')[1]
        if robot_name[-1] == '\n':
            robot_name = robot_name[:-1]
        # remove all characters after the first occurence of '-' including the '-'
        robot_name = robot_name.split('-')[0]
        return robot_name

    def get_edge_robot_names(self, lines):
        # Get the line start with 'from'
        from_str = next((line for line in lines if line.startswith('from')), None)
        from_robot_name = from_str.split(' ')[1]
        if from_robot_name[-1] == '\n':
            from_robot_name = from_robot_name[:-1]
        # remove all characters after the first occurence of '-' including the '-'
        from_robot_name = from_robot_name.split('-')[0]
        # Get the line start with 'to'
        to_str = next((line for line in lines if line.startswith('to')), None)
        to_robot_name = to_str.split(' ')[1]
        if to_robot_name[-1] == '\n':
            to_robot_name = to_robot_name[:-1]
        # remove all characters after the first occurence of '-' including the '-'
        to_robot_name = to_robot_name.split('-')[0]
        return from_robot_name, to_robot_name

    def print_results_info(self):
        if self.result_folder == '':
            print('Please specify the result folder parameter <result_folder> like this: --ros-args -p result_folder:=/path/to/result_folder')
            exit(1)
        # iterate over all the edges of a result directory and print the information
        edges_folder = os.path.join(self.result_folder, 'edges')
        edges_files = [f for f in os.listdir(edges_folder) if f.endswith('.txt')]
        edges_files = sorted(edges_files)
        edges_dict = {}
        edges_dict['robot_name'] = None
        edges_dict['odometry-edges'] = 0
        edges_dict['edges_names'] = []
        edges_dict['intra-robot-loops'] = 0
        edges_dict['inter-robot-loops'] = 0
        for edges_file in edges_files:
            with open(os.path.join(edges_folder, edges_file), mode='r') as f:
                lines = f.readlines()
                # Get the robot name from the first line
                if edges_dict['robot_name'] is None:
                    edges_dict['robot_name'] = self.get_robot_name_from_first_edge(lines)
                    print('Own robot name: {}'.format(edges_dict['robot_name']))

                # get the line starting with 'edge'
                edge_str = next((line for line in lines if line.startswith('edge')), None)
                edge_name = edge_str.split(' ')[1]
                if edge_name[-1] == '\n':
                    edge_name = edge_name[:-1]

                # get the line starting with 'type'
                type_str = next((line for line in lines if line.startswith('type')), None)
                type = type_str.split(' ')[1]
                if type[-1] == '\n':
                    type = type[:-1]
                if type == 'loop':
                    edges_dict['edges_names'].append(edge_name)
                    from_robot_name, to_robot_name = self.get_edge_robot_names(lines)
                    if from_robot_name == edges_dict['robot_name'] and to_robot_name == edges_dict['robot_name']:
                        edges_dict['intra-robot-loops'] += 1
                    if from_robot_name != edges_dict['robot_name'] or to_robot_name != edges_dict['robot_name']:
                        edges_dict['inter-robot-loops'] += 1
                elif type == 'odom':
                    from_robot_name, to_robot_name = self.get_edge_robot_names(lines)
                    if from_robot_name == edges_dict['robot_name'] and to_robot_name == edges_dict['robot_name']:
                        edges_dict['odometry-edges'] += 1

        print('Number of odometry edges: {}'.format(edges_dict['odometry-edges']))
        print('Number of intra-robot loops: {}'.format(edges_dict['intra-robot-loops']))
        print('Number of inter-robot loops: {}'.format(edges_dict['inter-robot-loops']))
        for edge_name in edges_dict['edges_names']:
            print('Edge name: {}'.format(edge_name))
        exit(0)


def play_rosbags(executor, ros_bag_processor):
    ros_bag_processor.start_playback()
    spin(executor, ros_bag_processor)


def print_initial_poses(executor, ros_bag_processor):
    ros_bag_processor.print_initial_poses()
    spin(executor, ros_bag_processor)


def plot_trajectories(executor, ros_bag_processor):
    ros_bag_processor.plot_trajectories()
    spin(executor, ros_bag_processor)


def print_info(executor, ros_bag_processor):
    ros_bag_processor.print_info()
    spin(executor, ros_bag_processor)


def write_odom_groundtruth(executor, ros_bag_processor):
    ros_bag_processor.write_odom_groundtruth()
    spin(executor, ros_bag_processor)


def results_to_pose_file(executor, ros_bag_processor):
    ros_bag_processor.results_to_pose_file()
    spin(executor, ros_bag_processor)


def print_results_info(executor, ros_bag_processor):
    ros_bag_processor.print_results_info()
    spin(executor, ros_bag_processor)


def spin(executor, node):
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)
    # We need a MultiThreadedExecutor to process certain callbacks while within another callback
    executor = rclpy.executors.MultiThreadedExecutor()
    rosbag_processor = RosbagProcessor()
    executor.add_node(rosbag_processor)

    fire.Fire({
        'play_rosbags': lambda: play_rosbags(executor, rosbag_processor),
        'print_initial_poses': lambda: print_initial_poses(executor, rosbag_processor),
        'plot_trajectories': lambda: plot_trajectories(executor, rosbag_processor),
        'print_info': lambda: print_info(executor, rosbag_processor),
        'write_odom_groundtruth': lambda: write_odom_groundtruth(executor, rosbag_processor),
        'results_to_pose_file': lambda: results_to_pose_file(executor, rosbag_processor),
        'print_results_info': lambda: print_results_info(executor, rosbag_processor),
    })


if __name__ == '__main__':
    main()
