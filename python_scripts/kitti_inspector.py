import os
import fire
import datetime
import time
import subprocess
from tqdm import tqdm
import signal
import sys
from enum import Enum
import re

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import Path
from vamex_slam_msgs.srv import DumpGraph, SaveMap
from visualization_msgs.msg import MarkerArray, Marker

from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import numpy as np
import pykitti
# Some information on the odometry dataset
# dataset.calib:      Calibration data are accessible as a named tuple
# dataset.timestamps: Timestamps are parsed into a list of timedelta objects
# dataset.poses:      List of ground truth poses T_w_cam0
# dataset.camN:       Generator to load individual images from camera N
# dataset.gray:       Generator to load monochrome stereo pairs (cam0, cam1)
# dataset.rgb:        Generator to load RGB stereo pairs (cam2, cam3)
# dataset.velo:       Generator to load velodyne scans as [x,y,z,reflectance]


def pykitti_ts_to_ros_ts(pykitti_ts: datetime.timedelta) -> Time:
    ros_ts = Time()
    ros_ts.sec = int(pykitti_ts.total_seconds())
    ros_ts.nanosec = int((pykitti_ts.total_seconds() - ros_ts.sec) * 1e9)
    return ros_ts


def float_ts_to_ros_ts(float_ts: float) -> Time:
    ros_ts = Time()
    ros_ts.sec = int(float_ts)
    ros_ts.nanosec = int((float_ts - ros_ts.sec) * 1e9)
    return ros_ts


class Loop(Enum):
    INTRA_ROBOT = 0
    INTER_ROBOT = 1


class KittiMultiRobotProcessor(Node):
    def __init__(self) -> None:
        super().__init__('kitti_multirobot_processor')

        self.robot_name = self.declare_parameter('robot_name', 'atlas').value
        self.base_path = self.declare_parameter('base_path', '/data/datasets/kitti/dataset/').value
        self.slam_config = self.declare_parameter('slam_config', 'hdl_multi_robot_graph_slam_kitti.yaml').value
        self.sequence = self.declare_parameter('sequence', 0).value
        self.sequence = str(self.sequence).zfill(2)
        self.result_dir = self.declare_parameter('result_dir', '/tmp/ground_truth').value
        self.tolerance = self.declare_parameter('tolerance', 4.0).value
        self.reversed_pose_file = self.declare_parameter('reversed_pose_file', '').value
        self.reversed_sequence = self.declare_parameter('reversed_sequence', -1).value

        self.reentrant_callback_group = ReentrantCallbackGroup()

        self.pose_file = self.declare_parameter('pose_file', '').value
        self.pose_file2 = self.declare_parameter('pose_file2', '').value
        self.pcl_filename = self.declare_parameter('pcl_filename', '').value
        self.edges_dir = self.declare_parameter('edges_dir', '').value
        self.keyframes_dir = self.declare_parameter('keyframes_dir', '').value
        self.pcl_pub = self.create_publisher(PointCloud2, self.robot_name + '/map_points', 10, callback_group=self.reentrant_callback_group)
        self.markers_pub_intra_loops_spheres = self.create_publisher(
            Marker, self.robot_name + '/intra_loop_markers_spheres', 10, callback_group=self.reentrant_callback_group)
        self.markers_pub_inter_loops_spheres = self.create_publisher(
            Marker, self.robot_name + '/inter_loop_markers_spheres', 10, callback_group=self.reentrant_callback_group)
        self.marker_pub_rob1 = self.create_publisher(Marker, self.robot_name + '/path_markers',
                                                     10, callback_group=self.reentrant_callback_group)
        self.markers_pub_inter_loops_lines = self.create_publisher(
            Marker, self.robot_name + '/inter_loop_markers_lines', 10, callback_group=self.reentrant_callback_group)
        self.markers_pub_intra_loops_lines = self.create_publisher(
            Marker, self.robot_name + '/intra_loop_markers_lines', 10, callback_group=self.reentrant_callback_group)
        self.marker_pub_rob2 = self.create_publisher(Marker, self.robot_name + '/path_markers2',
                                                     10, callback_group=self.reentrant_callback_group)

        point_cloud_topic = self.robot_name + '/velodyne_points'
        self.point_cloud_pub = self.create_publisher(PointCloud2, point_cloud_topic, 10, callback_group=self.reentrant_callback_group)
        self.clock_pub = self.create_publisher(Clock, 'clock', 10)
        self.point_cloud_counter = 0

        gt_path_topic = self.robot_name + '/gt_path'
        self.gt_path_pub = self.create_publisher(Path, gt_path_topic, 10, callback_group=self.reentrant_callback_group)

        self.dataset = pykitti.odometry(self.base_path, self.sequence)  # type: pykitti.odometry
        self.timestamps = self.dataset.timestamps  # type: list[datetime.timedelta]

    def plot_trajectories(self):
        cam_gt_poses = self.dataset.poses  # type: list[np.ndarray]
        self.velo_gt_poses = [np.linalg.inv(self.dataset.calib.T_cam0_velo) @ pose for pose in cam_gt_poses]
        traveled_distance = 0
        for i in range(len(self.velo_gt_poses) - 1):
            traveled_distance += np.linalg.norm(self.velo_gt_poses[i][0:3, 3] - self.velo_gt_poses[i + 1][0:3, 3])
        print(f'total traveled distance {traveled_distance}')
        print(f'total time {(self.timestamps[-1] - self.timestamps[0]).total_seconds()}')

        fig = plt.figure()
        ax = fig.add_subplot(111)
        # plot the ground truth poses with a color gradient from start (blue) to end (red)
        colors = np.arange(len(self.velo_gt_poses))
        colormap = plt.get_cmap('viridis')
        normalize = plt.Normalize(vmin=colors.min(), vmax=colors.max())
        scatter = ax.scatter([pose[0, 3] for pose in self.velo_gt_poses],
                             [pose[1, 3] for pose in self.velo_gt_poses],
                             c=colors, cmap='viridis', label='velo', s=0.1)
        # cbar = plt.colorbar(plt.cm.ScalarMappable(norm=normalize, cmap=colormap))
        cbar = plt.colorbar(scatter)
        cbar.set_label('time')

        # plot the ground truth line with a tolerance for picking points and printing their timestamps
        tolerance = self.tolerance
        ax.plot([pose[0, 3] for pose in self.velo_gt_poses],
                [pose[1, 3] for pose in self.velo_gt_poses],
                c='black', label='velo', picker=tolerance, linewidth=0.1)

        intervals = 50
        idxs = np.floor(np.linspace(0, len(self.velo_gt_poses) - 1, intervals)).astype(int)
        print(f'num poses {len(self.velo_gt_poses)} idxs {idxs}')
        for i, idx in enumerate(idxs):
            if i == 0:
                ax.text(self.velo_gt_poses[0][0, 3], self.velo_gt_poses[0][1, 3], 'start')
            if i == len(idxs) - 1:
                ax.text(self.velo_gt_poses[-1][0, 3], self.velo_gt_poses[-1][1, 3], 'end')
            else:
                ax.text(self.velo_gt_poses[idx][0, 3], self.velo_gt_poses[idx][1, 3],
                        f'{i*100/intervals}%')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_aspect('equal')
        # turn on grid and other options to make the plot look nice and analyze it better
        ax.grid(True)
        ax.legend()
        # tight layout
        plt.tight_layout()
        fig.canvas.mpl_connect('pick_event', self.on_pick)
        plt.show()

    def on_pick(self, event):
        artist = event.artist
        xmouse, ymouse = event.mouseevent.xdata, event.mouseevent.ydata
        # print(f'xmouse {xmouse:.2f}, ymouse {ymouse:.2f}')
        x, y = artist.get_xdata(), artist.get_ydata()
        indexes = event.ind
        print(f'indexes {indexes}')
        for index in indexes:
            print(
                f'   index {index} timestamp {self.timestamps[index].total_seconds()} x {self.velo_gt_poses[index][0, 3]} y {self.velo_gt_poses[index][1, 3]} z {self.velo_gt_poses[index][2, 3]}')

    # print info about all the chosen sequences
    def print_info(self):
        print(f'sequences {self.dataset.sequence}')
        print(f'number of point clouds {len(self.timestamps)}')
        cam_gt_poses = self.dataset.poses  # type: list[np.ndarray]
        velo_gt_poses = [np.linalg.inv(self.dataset.calib.T_cam0_velo) @ pose for pose in cam_gt_poses]
        traveled_distance = 0
        for i in range(len(velo_gt_poses) - 1):
            traveled_distance += np.linalg.norm(velo_gt_poses[i][0:3, 3] - velo_gt_poses[i + 1][0:3, 3])
        print(f'total traveled distance {traveled_distance}')
        print(f'number of timestamps {len(self.timestamps)}')
        self.timestamps = [time.total_seconds() for time in self.timestamps]
        print(f'total time {self.timestamps[-1] - self.timestamps[0]}')
        print(f'average time between timestamps {(self.timestamps[-1] - self.timestamps[0]) / len(self.timestamps)}')

        # prnt some speed statistics
        speeds = []
        prev_pose = None
        prev_ts = None
        for ts, pose in zip(self.timestamps, velo_gt_poses):
            if prev_pose is None:
                prev_pose = pose
                prev_ts = ts
                continue
            speeds.append(np.linalg.norm(pose[0:3, 3] - prev_pose[0:3, 3]) / (ts - prev_ts))
            prev_pose = pose
            prev_ts = ts
        print(f'mean speed {np.mean(speeds):.2f} m/s std {np.std(speeds):.2f} m/s')
        print(f'max speed {np.max(speeds):.2f} m/s, min {np.min(speeds):.2f} m/s')
        print(f'mean speed {np.mean(speeds) * 3.6:.2f} km/h std {np.std(speeds) * 3.6:.2f} km/h')
        print(f'max speed {np.max(speeds) * 3.6:.2f} km/h, min {np.min(speeds) * 3.6:.2f} km/h')
        # print(speeds)
        # print(self.timestamps)
        speeds_half_1 = speeds[0:len(speeds) // 2]
        speeds_half_2 = speeds[len(speeds) // 2:]
        print(f'mean speed half 1 {np.mean(speeds_half_1):.2f} m/s max {np.max(speeds_half_1):.2f} m/s min {np.min(speeds_half_1):.2f} m/s')
        print(f'mean speed half 2 {np.mean(speeds_half_2):.2f} m/s max {np.max(speeds_half_2):.2f} m/s min {np.min(speeds_half_2):.2f} m/s')
        print(
            f'mean speed half 1 {np.mean(speeds_half_1) * 3.6:.2f} km/h max {np.max(speeds_half_1) * 3.6:.2f} km/h min {np.min(speeds_half_1) * 3.6:.2f} km/h')
        print(
            f'mean speed half 2 {np.mean(speeds_half_2) * 3.6:.2f} km/h max {np.max(speeds_half_2) * 3.6:.2f} km/h min {np.min(speeds_half_2) * 3.6:.2f} km/h')

        step = 1000
        small_step = 10
        velo = self.dataset.get_velo(0)
        num_points = velo.shape[0]
        extra_num_points = num_points % step
        # remove the extra points so that the point cloud is divisible by 1000
        velo = velo[:-extra_num_points]
        print(f'number of points in first point cloud {velo.shape[0]}')
        points = []
        sizes = []
        while velo.shape[0] > 0:
            ros_pcl = self.kitti_to_ros_point_cloud(self.timestamps[0], velo)
            # print the number of bytes in the point cloud
            points.append(ros_pcl.width)
            sizes.append(sys.getsizeof(ros_pcl.data))
            print(f'ros pcl size {sys.getsizeof(ros_pcl.data)} ros pcl len {len(ros_pcl.data)} velo size {velo.shape[0]}')
            if velo.shape[0] <= step:
                velo = velo[:-small_step]
            else:
                velo = velo[:-step]

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(points, sizes, 'o')
        ax.set_xlabel('number of points')
        ax.set_ylabel('size in bytes')
        ax.grid(True)
        plt.tight_layout()
        plt.show()

        print('calculating velodyne point information, this may take some time')
        num_pcl_points = [velo.shape[0] for velo in self.dataset.velo]
        print(f'mean number of points pcl {np.mean(num_pcl_points)} std {np.std(num_pcl_points)}')

        exit(0)

    # Write all ground truth poses to a file with the following format:
    # timestamp x y z qx qy qz qw
    def write_ground_truth(self):
        for i in range(11):
            seq = str(i).zfill(2)
            dataset = pykitti.odometry(self.base_path, seq)
            times = dataset.timestamps
            times = [time.total_seconds() for time in times]
            cam_gt_poses = dataset.poses
            velo_gt_poses = [np.linalg.inv(dataset.calib.T_cam0_velo) @ pose for pose in cam_gt_poses]
            # transform the poses so that the first pose is the identity
            velo_gt_poses_tranformed = np.array([pose @ np.linalg.inv(velo_gt_poses[0]) for pose in velo_gt_poses])
            # cam ground truth
            cam_result_path = os.path.join(self.result_dir, 'ground_truth', seq + 'cam.txt')
            dir_result_path = os.path.dirname(cam_result_path)
            if not os.path.exists(dir_result_path):
                os.makedirs(dir_result_path)
            print(f'writing cam ground truth to {cam_result_path}')
            with open(cam_result_path, 'w') as f:
                for i in range(len(times)):
                    time = times[i]
                    pose = cam_gt_poses[i]
                    translation = pose[0:3, 3]
                    quat = R.from_matrix(pose[0:3, 0:3]).as_quat()
                    f.write(f'{time} {translation[0]} {translation[1]} {translation[2]} {quat[0]} {quat[1]} {quat[2]} {quat[3]}\n')
            # velo ground truth
            velo_result_path = os.path.join(self.result_dir, 'ground_truth', seq + 'velo.txt')
            velo_result_dir = os.path.dirname(velo_result_path)
            print(f'writing velo ground truth to {velo_result_path}')
            if not os.path.exists(velo_result_dir):
                os.makedirs(velo_result_dir)
            with open(velo_result_path, 'w') as f:
                for i in range(len(times)):
                    time = times[i]
                    pose = velo_gt_poses_tranformed[i]
                    translation = pose[0:3, 3]
                    quat = R.from_matrix(pose[0:3, 0:3]).as_quat()
                    f.write(f'{time} {translation[0]} {translation[1]} {translation[2]} {quat[0]} {quat[1]} {quat[2]} {quat[3]}\n')
            # velo transformed ground truth
            velo_transformed_result_path = os.path.join(self.result_dir, 'ground_truth', seq + 'velo_xy.txt')
            velo_transformed_result_dir = os.path.dirname(velo_transformed_result_path)
            print(f'writing velo transformed ground truth to {velo_transformed_result_path}')
            if not os.path.exists(velo_transformed_result_dir):
                os.makedirs(velo_transformed_result_dir)
            with open(velo_transformed_result_path, 'w') as f:
                for i in range(len(times)):
                    time = times[i]
                    pose = velo_gt_poses_tranformed[i]
                    translation = pose[0:3, 3]
                    quat = R.from_matrix(pose[0:3, 0:3]).as_quat()
                    f.write(f'{time} {translation[0]} {translation[1]} {translation[2]} {quat[0]} {quat[1]} {quat[2]} {quat[3]}\n')
        exit(0)

    # This function reverses the timestamps of a rover moving backwards through a sequence
    def correct_reversed_timestamps(self):
        if self.reversed_pose_file == '':
            print(f'Provide reversed_pose_file for reversing timestamps {self.reversed_pose_file}')
            exit(0)
        sequence = self.reversed_sequence
        sequence = str(sequence).zfill(2)
        if self.reversed_sequence < 0:
            print(f'Provide reversed_sequence for reversing timestamps for {self.reversed_pose_file}')
            exit(0)
        dataset = pykitti.odometry(self.base_path, sequence)
        timestamps = dataset.timestamps
        timestamps = np.array([time.total_seconds() for time in timestamps])
        reversed_timestamps = timestamps[::-1]
        if not os.path.exists(self.reversed_pose_file):
            print(f'No pose file found at {self.reversed_pose_file}')
            exit(0)

        corrected_filename = self.reversed_pose_file.replace('.txt', '_corrected.txt')
        with open(self.reversed_pose_file, mode='r') as f, open(corrected_filename, mode='w') as f_corrected:
            lines = f.readlines()
            corrected_lines = []
            for i in range(len(lines)):
                line = lines[i]
                if line.startswith('#'):
                    continue
                line = line.split(' ')
                forward_ts = line[0]
                # associate the index of the forward timestamp with the reversed timestamp
                idx = np.argmin(np.abs(timestamps - float(forward_ts)))
                # print(f'forward ts {forward_ts} idx {idx} reversed ts {reversed_timestamps[idx]}')
                line[0] = str(reversed_timestamps[idx])
                line = ' '.join(line)
                corrected_lines.append(line)
            f_corrected.writelines(corrected_lines)
        exit(0)

    def get_keyframes(self, keyframes_folder):
        # get all filenames ending with .txt
        keyframes_filenames = [os.path.join(keyframes_folder, filename)
                               for filename in os.listdir(keyframes_folder) if filename.endswith('.txt')]
        keyframes_filenames = sorted(keyframes_filenames)
        keyframes = {}
        for keyframe_file in keyframes_filenames:
            with open(keyframe_file, 'r') as f:
                lines = f.readlines()
                # get the line which starts with 'robot_name'
                robot_name_line = next((line for line in lines if line.startswith('robot_name')), None)
                robot_name = robot_name_line.split(' ')[1]
                if robot_name[-1] == '\n':
                    robot_name = robot_name[:-1]
                # if robot_name not in keyframes:
                #     keyframes[robot_name] = {}
                # get the line that starts with odom_counter and get the number
                odom_counter_line = next((line for line in lines if line.startswith('odom_counter')), None)
                odom_counter = int(odom_counter_line.split(' ')[1])
                # get the line that starts with estimate
                estimate_line = next((line for line in lines if line.startswith('estimate')), None)
                # the 4 lines after estimate contain the pose with the format of a 4x4 transformation matrix
                estimate_lines = lines[lines.index(estimate_line) + 1:lines.index(estimate_line) + 5]
                mat = np.genfromtxt(estimate_lines)
                position = mat[0:3, 3]
                keyframe_name = robot_name + '-' + str(odom_counter)
                keyframes[keyframe_name] = position

        return keyframes

    def get_edges(self, edges_folder):
        # get all filenames ending with .txt
        edges_filenames = [os.path.join(edges_folder, filename)
                           for filename in os.listdir(edges_folder) if filename.endswith('.txt')]
        edges_filenames = sorted(edges_filenames)
        edges = {}
        for edge_file in edges_filenames:
            with open(edge_file, 'r') as f:
                lines = f.readlines()
                # get the line which starts with 'type'
                type_line = next((line for line in lines if line.startswith('type')), None)
                if 'loop' not in type_line:
                    continue
                # get the line which starts with 'edge'
                edge_line = next((line for line in lines if line.startswith('edge')), None)
                edge = edge_line.split(' ')[1]
                if edge[-1] == '\n':
                    edge = edge[:-1]
                # get the line which starts with to
                to_line = next((line for line in lines if line.startswith('to')), None)
                to_keyframe = to_line.split(' ')[1]
                if to_keyframe[-1] == '\n':
                    to_keyframe = to_keyframe[:-1]
                to_robot_name = to_keyframe.split('-')[0]
                # get the line which starts with from
                from_line = next((line for line in lines if line.startswith('from')), None)
                from_keyframe = from_line.split(' ')[1]
                from_robot_name = from_keyframe.split('-')[0]
                if from_keyframe[-1] == '\n':
                    from_keyframe = from_keyframe[:-1]
                # get the line which starts with 'relative_pose'
                relative_pose_line = next((line for line in lines if line.startswith('relative_pose')), None)
                # the 4 lines after relative_pose contain the pose with the format of a 4x4 transformation matrix
                relative_pose_lines = lines[lines.index(relative_pose_line) + 1:lines.index(relative_pose_line) + 5]
                mat = np.genfromtxt(relative_pose_lines)
                if from_robot_name == to_robot_name:
                    loop_type = Loop.INTRA_ROBOT
                else:
                    loop_type = Loop.INTER_ROBOT
                edges[edge] = [from_keyframe, to_keyframe, mat, loop_type]

        return edges

    def publish_pcl_and_path(self):
        import pcl
        if self.pcl_filename == '':
            print(f'Provide pcl_filename for publishing {self.pcl_filename}')
            exit(0)
        if not os.path.exists(self.pcl_filename):
            print(f'No point cloud file found at {self.pcl_filename}')
            exit(0)
        if self.pose_file == '':
            print(f'Provide pose_file for publishing {self.pose_file}')
            exit(0)
        if not os.path.exists(self.pose_file):
            print(f'No pose file found at {self.pose_file}')
            exit(0)

        point_cloud = pcl.load_XYZI(self.pcl_filename)  # type: pcl.PointCloud
        print(f'point cloud size {point_cloud.size}')

        # print(point_cloud.to_array().tobytes())

        dummy_size = point_cloud.size
        point_cloud_step = 10
        print(f'array {point_cloud.to_array().shape}')
        array = point_cloud.to_array()
        array = array[0:dummy_size:point_cloud_step, :]
        print(f'array {array.shape}')
        array_size = array.shape[0]
        pc2_msg = PointCloud2()
        pc2_msg.header.frame_id = 'map'
        pc2_msg.header.stamp = self.get_clock().now().to_msg()
        pc2_msg.height = 1
        pc2_msg.width = array_size
        pc2_msg.is_dense = False
        pc2_msg.is_bigendian = False
        pc2_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        # Float 32 is 4 bytes and there are 4 fields
        pc2_msg.point_step = len(pc2_msg.fields) * 4
        pc2_msg.row_step = pc2_msg.point_step * array_size
        # pc2_msg.data = point_cloud.to_array().tobytes()
        pc2_msg.data = array.tobytes()

        print(f'publishing point cloud with {array_size} points on topic {self.robot_name + "/map_points"}')
        self.pcl_pub.publish(pc2_msg)

        # timestamp x y z qx qy qz qw
        stamps_poses = []
        with open(self.pose_file, 'r') as f:
            for line in f.readlines():
                if line.startswith('#'):
                    continue
                line = line.split(' ')
                stamp_pose = [float(val) for val in line]
                stamps_poses.append(stamp_pose)

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        self.path_pub = self.create_publisher(Path, self.robot_name + '/path', 10, callback_group=self.reentrant_callback_group)
        for stamp_pose in stamps_poses:
            # pose = stamp_pose[1:]
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = float_ts_to_ros_ts(stamp_pose[0])
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = stamp_pose[1]
            pose_stamped.pose.position.y = stamp_pose[2]
            pose_stamped.pose.position.z = stamp_pose[3]
            pose_stamped.pose.orientation.x = stamp_pose[4]
            pose_stamped.pose.orientation.y = stamp_pose[5]
            pose_stamped.pose.orientation.z = stamp_pose[6]
            pose_stamped.pose.orientation.w = stamp_pose[7]

            path.poses.append(pose_stamped)

        print(f'publishing path with {len(path.poses)} poses on topic {self.robot_name + "/path"}')
        self.path_pub.publish(path)

        # Publish the loop edges as lines between from and to keyframes
        if self.edges_dir != '':
            if self.keyframes_dir == '':
                print(f'Provide keyframes_dir for publishing {self.keyframes_dir}')
                exit(0)
            if not os.path.exists(self.keyframes_dir):
                print(f'No keyframes folder found at {self.keyframes_dir}')
                exit(0)

            keyframes = self.get_keyframes(self.keyframes_dir)
            edges = self.get_edges(self.edges_dir)

            # Sphere list
            sphere_scale = 9.0
            line_scale = 1.5
            line_threshold = 5.0  # whether to draw a line between two keyframes or a sphere
            intra_robot_color = [25/255.0, 255/255.0, 14/255.0]
            inter_robot_color = [255/255.0, 105/255.0, 180/255.0]
            intra_robot_loop_sphere_list = Marker()
            inter_robot_loop_sphere_list = Marker()
            intra_robot_loop_line_list = Marker()
            inter_robot_loop_line_list = Marker()
            intra_robot_loop_sphere_list.header.frame_id = inter_robot_loop_sphere_list.header.frame_id = intra_robot_loop_line_list.header.frame_id = inter_robot_loop_line_list.header.frame_id = 'map'
            intra_robot_loop_sphere_list.header.stamp = inter_robot_loop_sphere_list.header.stamp = intra_robot_loop_line_list.header.stamp = inter_robot_loop_line_list.header.stamp = self.get_clock().now().to_msg()
            intra_robot_loop_sphere_list.ns = inter_robot_loop_sphere_list.ns = intra_robot_loop_line_list.ns = inter_robot_loop_line_list.ns = 'loop'
            intra_robot_loop_sphere_list.action = inter_robot_loop_sphere_list.action = intra_robot_loop_line_list.action = inter_robot_loop_line_list.action = Marker.ADD
            intra_robot_loop_sphere_list.pose.orientation.w = inter_robot_loop_sphere_list.pose.orientation.w = intra_robot_loop_line_list.pose.orientation.w = inter_robot_loop_line_list.pose.orientation.w = 1.0
            intra_robot_loop_sphere_list.id = inter_robot_loop_sphere_list.id = intra_robot_loop_line_list.id = inter_robot_loop_line_list.id = 0
            intra_robot_loop_sphere_list.type = inter_robot_loop_sphere_list.type = Marker.SPHERE_LIST
            intra_robot_loop_line_list.type = inter_robot_loop_line_list.type = Marker.LINE_LIST
            intra_robot_loop_sphere_list.scale.x = inter_robot_loop_sphere_list.scale.x = sphere_scale
            intra_robot_loop_sphere_list.scale.y = inter_robot_loop_sphere_list.scale.y = sphere_scale
            intra_robot_loop_sphere_list.scale.z = inter_robot_loop_sphere_list.scale.z = sphere_scale
            intra_robot_loop_line_list.scale.x = inter_robot_loop_line_list.scale.x = line_scale
            intra_robot_loop_line_list.scale.y = inter_robot_loop_line_list.scale.y = line_scale
            intra_robot_loop_line_list.scale.z = inter_robot_loop_line_list.scale.z = line_scale
            intra_robot_loop_sphere_list.color.a = inter_robot_loop_sphere_list.color.a = 1.0
            intra_robot_loop_sphere_list.color.r = intra_robot_loop_line_list.color.r = intra_robot_color[0]
            intra_robot_loop_sphere_list.color.g = intra_robot_loop_line_list.color.g = intra_robot_color[1]
            intra_robot_loop_sphere_list.color.b = intra_robot_loop_line_list.color.b = intra_robot_color[2]
            intra_robot_loop_line_list.color.a = inter_robot_loop_line_list.color.a = 1.0
            inter_robot_loop_sphere_list.color.r = inter_robot_loop_line_list.color.r = inter_robot_color[0]
            inter_robot_loop_sphere_list.color.g = inter_robot_loop_line_list.color.g = inter_robot_color[1]
            inter_robot_loop_sphere_list.color.b = inter_robot_loop_line_list.color.b = inter_robot_color[2]

            for from_keyframe, to_keyframe, relative_pose, loop_type in edges.values():
                from_keyframe_position = keyframes[from_keyframe]
                to_keyframe_position = keyframes[to_keyframe]
                distance = np.linalg.norm(to_keyframe_position - from_keyframe_position)
                if distance < line_threshold:
                    # draw a sphere
                    unit_vector_to_minus_from = (
                        to_keyframe_position - from_keyframe_position) / np.linalg.norm(to_keyframe_position - from_keyframe_position)
                    middle_position = from_keyframe_position + distance / 2.0 * unit_vector_to_minus_from
                    middle_point = Point()
                    middle_point.x = middle_position[0]
                    middle_point.y = middle_position[1]
                    middle_point.z = middle_position[2] + 1.0
                    if loop_type == Loop.INTER_ROBOT:
                        inter_robot_loop_sphere_list.points.append(middle_point)
                    elif loop_type == Loop.INTRA_ROBOT:
                        intra_robot_loop_sphere_list.points.append(middle_point)
                else:
                    from_point = Point()
                    from_point.x = from_keyframe_position[0]
                    from_point.y = from_keyframe_position[1]
                    from_point.z = from_keyframe_position[2] + 1.0
                    to_point = Point()
                    to_point.x = to_keyframe_position[0]
                    to_point.y = to_keyframe_position[1]
                    to_point.z = to_keyframe_position[2] + 1.0
                    if loop_type == Loop.INTER_ROBOT:
                        inter_robot_loop_line_list.points.append(from_point)
                        inter_robot_loop_line_list.points.append(to_point)
                    elif loop_type == Loop.INTRA_ROBOT:
                        intra_robot_loop_line_list.points.append(from_point)
                        intra_robot_loop_line_list.points.append(to_point)

            print(
                f'publishing loop edges as lines with {len(intra_robot_loop_line_list.points)} points on topic {self.robot_name + "/loop_markers"}')
            self.markers_pub_intra_loops_lines.publish(intra_robot_loop_line_list)
            print(
                f'publishing loop edges as lines with {len(inter_robot_loop_line_list.points)} points on topic {self.robot_name + "/loop_markers"}')
            self.markers_pub_inter_loops_lines.publish(inter_robot_loop_line_list)
            print(
                f'publishing loop edges as spheres with {len(intra_robot_loop_sphere_list.points)} points on topic {self.robot_name + "/loop_markers"}')
            self.markers_pub_intra_loops_spheres.publish(intra_robot_loop_sphere_list)
            print(
                f'publishing loop edges as spheres with {len(inter_robot_loop_sphere_list.points)} points on topic {self.robot_name + "/loop_markers"}')
            self.markers_pub_inter_loops_spheres.publish(inter_robot_loop_sphere_list)

        # publish the path as markers but as a line
        line_alpha = 0.6
        line_thickness = 2.0
        line_list = Marker()
        color = [1.0, 0.0, 0.0]  # red
        line_list.header.frame_id = 'map'
        line_list.header.stamp = self.get_clock().now().to_msg()
        line_list.ns = 'path'
        line_list.action = Marker.ADD
        line_list.pose.orientation.w = 1.0
        line_list.id = 0
        line_list.type = Marker.LINE_STRIP
        line_list.scale.x = line_thickness
        line_list.color.a = line_alpha
        line_list.color.r = color[0]
        line_list.color.g = color[1]
        line_list.color.b = color[2]
        for pose_stamped in path.poses:
            line_list.points.append(pose_stamped.pose.position)
        print(f'publishing path as markers with {len(line_list.points)} points on topic {self.robot_name + "/path_markers"}')
        self.marker_pub_rob1.publish(line_list)

        if self.pose_file2 != '':
            print(f'publishing path as markers with {len(line_list.points)} points on topic {self.robot_name + "/path_markers2"}')

        stamps_poses2 = []
        if self.pose_file2 != '':
            with open(self.pose_file2, 'r') as f:
                for line in f.readlines():
                    if line.startswith('#'):
                        continue
                    line = line.split(' ')
                    stamp_pose = [float(val) for val in line]
                    stamps_poses2.append(stamp_pose)

        line_list2 = Marker()
        color2 = [255 / 255.0, 165 / 255.0, 0.0]  # orange
        line_list2.header.frame_id = 'map'
        line_list2.header.stamp = self.get_clock().now().to_msg()
        line_list2.ns = 'path'
        line_list2.action = Marker.ADD
        line_list2.pose.orientation.w = 1.0
        line_list2.id = 0
        line_list2.type = Marker.LINE_STRIP
        line_list2.scale.x = line_thickness + + 0.25
        line_list2.color.a = line_alpha + 0.2
        line_list2.color.r = color2[0]
        line_list2.color.g = color2[1]
        line_list2.color.b = color2[2]
        for stamp_pose in stamps_poses2:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = float_ts_to_ros_ts(stamp_pose[0])
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = stamp_pose[1]
            pose_stamped.pose.position.y = stamp_pose[2]
            pose_stamped.pose.position.z = stamp_pose[3]
            pose_stamped.pose.orientation.x = stamp_pose[4]
            pose_stamped.pose.orientation.y = stamp_pose[5]
            pose_stamped.pose.orientation.z = stamp_pose[6]
            pose_stamped.pose.orientation.w = stamp_pose[7]
            line_list2.points.append(pose_stamped.pose.position)

        print(f'publishing path as markers with {len(line_list2.points)} points on topic {self.robot_name + "/path_markers2"}')
        self.marker_pub_rob2.publish(line_list2)

        exit(0)

    def kitti_to_ros_point_cloud(self, ts: datetime.timedelta, velo: np.ndarray) -> PointCloud2:
        # create a ROS2 PointCloud2 message
        ros_pcl = PointCloud2()
        if isinstance(ts, datetime.timedelta):
            ros_pcl.header.stamp = pykitti_ts_to_ros_ts(ts)
        else:
            ros_pcl.header.stamp = float_ts_to_ros_ts(ts)
        # ros_pcl.header.frame_id = self.robot_name + '/velodyne'
        num_points = velo.shape[0]
        ros_pcl.height = 1
        ros_pcl.width = num_points
        ros_pcl.is_dense = False
        ros_pcl.is_bigendian = False
        ros_pcl.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        # Float 32 is 4 bytes and there are 4 fields
        ros_pcl.point_step = len(ros_pcl.fields) * 4
        ros_pcl.row_step = ros_pcl.point_step * num_points
        ros_pcl.data = velo.tobytes()

        return ros_pcl


def plot_trajectories(executor, kitti_processor: KittiMultiRobotProcessor):
    kitti_processor.plot_trajectories()
    spin(executor, kitti_processor)


def print_info(executor, kitti_processor: KittiMultiRobotProcessor):
    kitti_processor.print_info()
    spin(executor, kitti_processor)


def write_ground_truth(executor, kitti_processor: KittiMultiRobotProcessor):
    kitti_processor.write_ground_truth()
    spin(executor, kitti_processor)


def correct_reversed_timestamps(executor, kitti_processor: KittiMultiRobotProcessor):
    kitti_processor.correct_reversed_timestamps()
    spin(executor, kitti_processor)


def publish_pcl_and_path(executor, kitti_processor: KittiMultiRobotProcessor):
    kitti_processor.publish_pcl_and_path()
    spin(executor, kitti_processor)


def spin(executor: MultiThreadedExecutor, node: KittiMultiRobotProcessor):
    # make sure all SIGINT are captured
    signal.signal(signal.SIGINT, signal.default_int_handler)
    try:
        rclpy.spin(node=node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        print('finally')
        node.destroy_node()
        executor.shutdown()


def main(args=None):

    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    kitti_processor = KittiMultiRobotProcessor()
    executor.add_node(kitti_processor)

    fire.Fire({
        'plot_trajectories': lambda: plot_trajectories(executor, kitti_processor),
        'print_info': lambda: print_info(executor, kitti_processor),
        'write_ground_truth': lambda: write_ground_truth(executor, kitti_processor),
        'correct_reversed_timestamps': lambda: correct_reversed_timestamps(executor, kitti_processor),
        'publish_pcl_and_path': lambda: publish_pcl_and_path(executor, kitti_processor),
    })


if __name__ == '__main__':
    main()
