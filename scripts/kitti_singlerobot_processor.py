import os
import fire
import datetime
import time
import subprocess
from tqdm import tqdm
import signal
import sys
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from vamex_slam_msgs.msg import SlamStatus
from vamex_slam_msgs.srv import DumpGraph, SaveMap
from scipy.spatial.transform import Rotation as R

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


class Task(Enum):
    PLAYBACK = 0
    DUMP_GRAPH = 1
    SAVE_MAP = 2
    SHUTDOWN_SLAM = 3
    SHUTDOWN_NODE = 4


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


class KittiMultiRobotProcessor(Node):
    def __init__(self) -> None:
        super().__init__('kitti_multirobot_processor')

        self.robot_name = self.declare_parameter('robot_name', 'atlas').value
        self.base_path = self.declare_parameter('base_path', '/data/datasets/kitti/dataset/').value
        self.slam_config = self.declare_parameter('slam_config', 'hdl_multi_robot_graph_slam_kitti.yaml').value
        self.sequence = self.declare_parameter('sequence', '00').value
        self.rate = self.declare_parameter('rate', 25.0).value
        self.result_dir = self.declare_parameter('result_dir', '/tmp/').value
        self.playback_length = self.declare_parameter('playback_length', -1).value
        self.map_resolution = self.declare_parameter('map_resolution', -1.0).value

        self.task = None

        self.reentrant_callback_group = ReentrantCallbackGroup()

        point_cloud_topic = self.robot_name + '/velodyne_points'
        self.point_cloud_pub = self.create_publisher(PointCloud2, point_cloud_topic, 10, callback_group=self.reentrant_callback_group)
        self.clock_pub = self.create_publisher(Clock, 'clock', 10)
        self.point_cloud_counter = 0

        gt_path_topic = self.robot_name + '/gt_path'
        self.gt_path_pub = self.create_publisher(Path, gt_path_topic, 10, callback_group=self.reentrant_callback_group)
        self.gt_path = Path()
        self.gt_path.header.frame_id = 'map'

        slam_status_topic = '/' + self.robot_name + '/hdl_graph_slam/slam_status'
        self.slam_status_sub = self.create_subscription(
            SlamStatus, slam_status_topic, self.slam_status_callback, 10, callback_group=self.reentrant_callback_group)
        self.slam_status = SlamStatus()  # type: SlamStatus

        self.dataset = pykitti.odometry(self.base_path, self.sequence)  # type: pykitti.odometry
        self.timestamps = self.dataset.timestamps  # type: list[datetime.timedelta]
        self.timestamps = [ts.total_seconds() for ts in self.timestamps]

        cam_gt_poses = self.dataset.poses  # type: list[np.ndarray]
        self.velo_gt_poses = [np.linalg.inv(self.dataset.calib.T_cam0_velo) @ pose for pose in cam_gt_poses]
        # transform the poses so that the first pose is the identity
        self.velo_gt_poses = np.array([pose @ np.linalg.inv(self.velo_gt_poses[0]) for pose in self.velo_gt_poses])

        self.dump_service_client = self.create_client(
            DumpGraph, '/' + self.robot_name + '/hdl_graph_slam/dump', callback_group=self.reentrant_callback_group)
        self.save_map_client = self.create_client(
            SaveMap, '/' + self.robot_name + '/hdl_graph_slam/save_map', callback_group=self.reentrant_callback_group)

        self.slam_process = None  # type: subprocess.Popen
        self.progress_bar = None  # type: tqdm

    def start_playback(self):
        self.dump_graph_requested = False
        self.save_map_requested = False
        # create the results directory
        if not os.path.exists(os.path.join(self.result_dir, self.sequence)):
            os.makedirs(os.path.join(self.result_dir, self.sequence))
        # Start the slam node in a subprocess
        slam_cmd = ['ros2', 'launch', 'hdl_graph_slam', 'hdl_multi_robot_graph_slam.launch.py',
                    'mode_namespace:=' + self.robot_name, 'config:=' + self.slam_config]
        with open(os.path.join(self.result_dir, self.sequence, 'slam.log'), mode='w') as slam_log:
            self.slam_process = subprocess.Popen(slam_cmd, stdout=slam_log, stderr=slam_log)
            print(f'Started slam node with pid {self.slam_process.pid} and cmd:')
            print(' '.join(slam_cmd))
        time.sleep(2)
        if self.playback_length > 0 and self.playback_length < len(self.timestamps):
            self.timestamps = self.timestamps[:self.playback_length]
        print(f'Starting playback with rate {self.rate:.2f}x')
        self.progress_bar = tqdm(total=len(self.timestamps), desc='Playback progress', unit='point clouds')
        self.task = Task.PLAYBACK
        self.task_timer = self.create_timer(1.0 / self.rate, self.task_timer_callback, callback_group=self.reentrant_callback_group)

    def task_timer_callback(self):
        self.task_timer.cancel()
        if self.task == Task.PLAYBACK:
            self.playback()
        elif self.task == Task.DUMP_GRAPH:
            self.dump_graph()
        elif self.task == Task.SAVE_MAP:
            self.save_map()
        elif self.task == Task.SHUTDOWN_SLAM:
            self.shutdown_slam()
        elif self.task == Task.SHUTDOWN_NODE:
            self.shutdown_node()
        else:
            print('Unknown task')
        self.task_timer.reset()

    def slam_status_callback(self, msg: SlamStatus):
        self.slam_status = msg

    def publish_ground_truth_path(self, ts, robot_name, kitti_pose):
        self.gt_path.header.stamp = float_ts_to_ros_ts(ts)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = float_ts_to_ros_ts(ts)
        pose_msg.header.frame_id = robot_name + '/velodyne'
        pose_msg.pose.position.x = kitti_pose[0, 3]
        pose_msg.pose.position.y = kitti_pose[1, 3]
        pose_msg.pose.position.z = kitti_pose[2, 3]
        rotation = R.from_matrix(kitti_pose[0:3, 0:3]).as_quat()
        pose_msg.pose.orientation.x = rotation[0]
        pose_msg.pose.orientation.y = rotation[1]
        pose_msg.pose.orientation.z = rotation[2]
        pose_msg.pose.orientation.w = rotation[3]
        self.gt_path.poses.append(pose_msg)
        self.gt_path_pub.publish(self.gt_path)

    def kitti_to_ros_point_cloud(self, ts: float, velo: np.ndarray) -> PointCloud2:
        # create a ROS2 PointCloud2 message
        ros_pcl = PointCloud2()
        ros_pcl.header.stamp = float_ts_to_ros_ts(ts)
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

    def playback(self):
        if self.slam_status.in_optimization or self.slam_status.in_loop_closure:
            return

        velo = self.dataset.get_velo(self.point_cloud_counter)  # type: np.ndarray
        ts = self.timestamps[self.point_cloud_counter]
        ros_pcl = self.kitti_to_ros_point_cloud(ts, velo)
        self.progress_bar.update(1)

        self.publish_clock_msg(ros_pcl.header.stamp)
        self.point_cloud_pub.publish(ros_pcl)

        self.publish_ground_truth_path(ts, self.robot_name, self.velo_gt_poses[self.point_cloud_counter])

        self.point_cloud_counter += 1

        if self.point_cloud_counter >= len(self.timestamps):
            self.progress_bar.close()
            print('Finished playback and closed progress bar')
            self.task = Task.DUMP_GRAPH
            # self.finalize_playback()

    def perform_async_service_call(self, client, request):
        while client.wait_for_service(timeout_sec=1.0) is False:
            print('service', client.srv_name, 'not available, waiting again...')

        print('calling async service', client.srv_name)
        future = client.call_async(request)
        if isinstance(request, DumpGraph.Request):
            future.add_done_callback(self.done_dump_graph_callback)
        if isinstance(request, SaveMap.Request):
            future.add_done_callback(self.done_save_map_callback)

    def done_dump_graph_callback(self, future):
        print(future.result())
        self.result = future.result()
        print(f'Dump graph service call success? {self.result.success}')
        self.task = Task.SAVE_MAP

    def done_save_map_callback(self, future):
        print(future.result())
        self.result = future.result()
        print(f'Save map service call success? {self.result.success}')
        self.task = Task.SHUTDOWN_SLAM

    def dump_graph(self):
        if self.dump_graph_requested:
            return
        self.dump_graph_requested = True
        # call the dumb and save graph service on hdl graph slam
        dump_request = DumpGraph.Request()
        dump_request.destination = os.path.join(self.result_dir, self.sequence, 'g2o')
        self.perform_async_service_call(self.dump_service_client, dump_request)

    def save_map(self):
        if self.save_map_requested:
            return
        # call the dumb and save graph service on hdl graph slam
        save_map_request = SaveMap.Request()
        save_map_request.destination = os.path.join(self.result_dir, self.sequence, 'map')
        save_map_request.resolution = self.map_resolution
        self.save_map_requested = True
        self.perform_async_service_call(self.save_map_client, save_map_request)

    def shutdown_slam(self):
        if self.slam_process is not None:
            print('Sending SIGINT to slam process')
            self.slam_process.send_signal(subprocess.signal.SIGINT)
            self.slam_process.wait(timeout=10)
            print('Slam process terminated')
        self.task = Task.SHUTDOWN_NODE

    def shutdown_node(self):
        print('Shutting down node, rclpy, and sys')
        self.destroy_node()
        rclpy.shutdown()
        print('Shutting down system')
        sys.exit()


def main(args=None):

    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    kitti_processor = KittiMultiRobotProcessor()
    executor.add_node(kitti_processor)

    kitti_processor.start_playback()
    signal.signal(signal.SIGINT, signal.default_int_handler)
    try:
        rclpy.spin(node=kitti_processor, executor=executor)
    except KeyboardInterrupt:
        print('Trying to gracefully shutdown')
        kitti_processor.shutdown_slam()
        kitti_processor.shutdown_node()
    finally:
        print('Shutting down')
        kitti_processor.destroy_node()
        executor.shutdown()


if __name__ == '__main__':
    main()
