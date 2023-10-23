import os
import fire
import datetime
import time
import subprocess
from tqdm import tqdm
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from pprint import pprint

from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Path
from vamex_slam_msgs.msg import SlamStatus

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


class KittiMultiRobotProcessor(Node):
    def __init__(self) -> None:
        super().__init__('kitti_multirobot_processor')

        self.robot_name = self.declare_parameter('robot_name', 'atlas').value
        self.base_path = self.declare_parameter('base_path', '/data/datasets/kitti/dataset/').value
        self.slam_config = self.declare_parameter('slam_config', 'hdl_multi_robot_graph_slam_kitti.yaml').value
        self.sequence = self.declare_parameter('sequence', '00').value
        self.rate = self.declare_parameter('rate', 10.0).value
        self.result_dir = self.declare_parameter('result_dir', '/tmp/').value
        self.reentrant_callback_group = ReentrantCallbackGroup()

        point_cloud_topic = self.robot_name + '/velodyne_points'
        self.point_cloud_pub = self.create_publisher(PointCloud2, point_cloud_topic, 10, callback_group=self.reentrant_callback_group)
        self.clock_pub = self.create_publisher(Clock, 'clock', 10)
        self.point_cloud_counter = 0

        gt_path_topic = self.robot_name + '/gt_path'
        self.gt_path_pub = self.create_publisher(Path, gt_path_topic, 10, callback_group=self.reentrant_callback_group)

        slam_status_topic = '/' + self.robot_name + '/hdl_graph_slam/slam_status'
        self.slam_status_sub = self.create_subscription(
            SlamStatus, slam_status_topic, self.slam_status_callback, 10, callback_group=self.reentrant_callback_group)
        self.slam_status = SlamStatus()  # type: SlamStatus

        self.dataset = pykitti.odometry(self.base_path, self.sequence)  # type: pykitti.odometry
        self.timestamps = self.dataset.timestamps  # type: list[datetime.timedelta]
        # assert (len(self.timestamps) == len(list(self.dataset.velo)))

    def slam_status_callback(self, msg: SlamStatus):
        self.slam_status = msg

    def start_playback(self):
        # Start the slam node in a subprocess
        slam_cmd = ['ros2', 'launch', 'hdl_graph_slam', 'hdl_multi_robot_graph_slam.launch.py',
                    'mode_namespace:=' + self.robot_name, 'config:=' + self.slam_config]
        with open(os.path.join(self.result_dir, 'slam.log'), mode='w') as slam_log:
            self.slam_process = subprocess.Popen(slam_cmd, stdout=slam_log, stderr=slam_log)
            print(f'Started slam node with cmd {slam_cmd}')
            print(f'Started slam node with pid {self.slam_process.pid}')
        time.sleep(2)
        self.progress_bar = tqdm(total=len(self.timestamps), desc='Playback progress', unit='point clouds')
        print(f'Starting playback with rate {self.rate:.2f}x')
        self.timer = self.create_timer(1.0 / self.rate, self.playback_timer,
                                       callback_group=self.reentrant_callback_group)

    def kitti_to_ros_point_cloud(self, ts: datetime.timedelta, velo: np.ndarray) -> PointCloud2:
        # create a ROS2 PointCloud2 message
        ros_pcl = PointCloud2()
        ros_pcl.header.stamp = pykitti_ts_to_ros_ts(ts)
        ros_pcl.header.frame_id = self.robot_name + '/velodyne'
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

    def publish_clock_msg(self, stamp: Time):
        clock_msg = Clock()
        clock_msg.clock = stamp
        self.clock_pub.publish(clock_msg)

    def playback_timer(self):
        self.timer.cancel()
        if self.slam_status.in_optimization or self.slam_status.in_loop_closure:
            self.timer.reset()
            return

        velo = self.dataset.get_velo(self.point_cloud_counter)  # type: np.ndarray
        ts = self.timestamps[self.point_cloud_counter]
        ros_pcl = self.kitti_to_ros_point_cloud(ts, velo)
        self.progress_bar.update(1)

        self.publish_clock_msg(ros_pcl.header.stamp)
        self.point_cloud_pub.publish(ros_pcl)

        self.point_cloud_counter += 1

        if self.point_cloud_counter >= len(self.timestamps):
            self.timer.cancel()
            print('Finished playback')
            self.finalize_playback()
        else:
            self.timer.reset()

    def finalize_playback(self):
        # call the dumb and save graph service on hdl graph slam
        self.slam_process.send_signal(subprocess.signal.SIGINT)
        self.slam_process.wait(timeout=10)
        self.progress_bar.close()

    def plot_trajectories(self):
        cam_gt_poses = self.dataset.poses  # type: list[np.ndarray]
        self.velo_gt_poses = [np.linalg.inv(self.dataset.calib.T_cam0_velo) @ pose for pose in cam_gt_poses]
        fig = plt.figure()
        ax = fig.add_subplot(111)
        tolerance = 0.5
        ax.plot([pose[0, 3] for pose in self.velo_gt_poses], [pose[1, 3]
                for pose in self.velo_gt_poses], 'r-', label='velo', picker=tolerance)
        intervals = 20
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
        ax.legend()

        def on_pick(self, event):
            artist = event.artist
            xmouse, ymouse = event.mouseevent.xdata, event.mouseevent.ydata
            print(f'xmouse {xmouse:.2f}, ymouse {ymouse:.2f}')
            x, y = artist.get_xdata(), artist.get_ydata()
            index = event.ind
            print(f'indexes {index}')
            print(f'timestamp {self.timestamps[index].total_seconds()} pose {self.velo_gt_poses[index]}')
        fig.canvas.mpl_connect('pick_event', on_pick)
        plt.show()

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
        num_pcl_points = [velo.shape[0] for velo in self.dataset.velo]
        print(f'mean number of points pcl {np.mean(num_pcl_points)} std {np.std(num_pcl_points)}')


def plot_trajectories(executor, kitti_processor: KittiMultiRobotProcessor):
    kitti_processor.plot_trajectories()
    spin(executor, kitti_processor)


def start_playback(executor, kitti_processor: KittiMultiRobotProcessor):
    kitti_processor.start_playback()
    spin(executor, kitti_processor)


def print_info(executor, kitti_processor: KittiMultiRobotProcessor):
    kitti_processor.print_info()
    spin(executor, kitti_processor)


def spin(executor: MultiThreadedExecutor, node: KittiMultiRobotProcessor):
    # make sure all SIGINT are captured
    signal.signal(signal.SIGINT, signal.default_int_handler)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Trying to gracefully shutdown')
        node.finalize_playback()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    kitti_processor = KittiMultiRobotProcessor()
    executor.add_node(kitti_processor)

    fire.Fire({
        'plot_trajectories': lambda: plot_trajectories(executor, kitti_processor),
        'start_playback': lambda: start_playback(executor, kitti_processor),
        'print_info': lambda: print_info(executor, kitti_processor),
    })


if __name__ == '__main__':
    main()
