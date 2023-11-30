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
from nav_msgs.msg import Path
from vamex_slam_msgs.srv import DumpGraph, SaveMap

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
                print(f'forward ts {forward_ts} idx {idx} reversed ts {reversed_timestamps[idx]}')
                line[0] = str(reversed_timestamps[idx])
                line = ' '.join(line)
                corrected_lines.append(line)
            f_corrected.writelines(corrected_lines)
        exit(0)


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
    })


if __name__ == '__main__':
    main()
