import os
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
from vamex_slam_msgs.msg import SlamStatus
from vamex_slam_msgs.srv import DumpGraph, SaveMap

import numpy as np
from scipy.spatial.transform import Rotation as R
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
    WAIT_SLAM_DONE = 1
    DUMP_GRAPH = 2
    SAVE_MAP = 3
    SHUTDOWN_SLAM = 4
    SHUTDOWN_NODE = 5


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

        self.robot_names = self.declare_parameter('robot_names', ['atlas', 'bestla']).value
        print(self.robot_names)
        self.min_times = self.declare_parameter('min_times', [0.0, 0.0]).value
        self.max_times = self.declare_parameter('max_times', [60.0, 60.0]).value
        self.base_path = self.declare_parameter('base_path', '/data/datasets/kitti/dataset/').value
        self.slam_config = self.declare_parameter('slam_config', 'hdl_multi_robot_graph_slam_kitti.yaml').value
        self.sequence = self.declare_parameter('sequence', 0).value
        self.sequence = str(self.sequence).zfill(2)
        self.rate = self.declare_parameter('rate', 1.0).value
        self.result_dir = self.declare_parameter('result_dir', '/home/andi/Seafile/data/slam_results/kitti/sequences/').value
        self.playback_length = self.declare_parameter('playback_length', -1).value
        self.eval_name = self.declare_parameter('eval_name', 'reversed').value
        # -1 means all points of the pointcloud, otherwise voxel size
        self.map_resolution = self.declare_parameter('map_resolution', -1.0).value
        self.start_time = self.declare_parameter('start_time', 0.0).value
        self.end_time = self.declare_parameter('end_time', float('inf')).value
        self.proceed = self.declare_parameter('proceed', True).value

        self.task = None

        self.reentrant_callback_group = ReentrantCallbackGroup()

        self.robots = dict()

        # Setup all robots with dictionaries for their publishers, subscribers, and services, flags
        for robot_name in self.robot_names:
            self.robots[robot_name] = dict()
            point_cloud_topic = robot_name + '/velodyne_points'
            self.robots[robot_name]['point_cloud_pub'] = self.create_publisher(
                PointCloud2, point_cloud_topic, 10, callback_group=self.reentrant_callback_group)
            slam_status_topic = '/' + robot_name + '/hdl_graph_slam/slam_status'
            self.robots[robot_name]['slam_status_sub'] = self.create_subscription(
                SlamStatus, slam_status_topic, self.slam_status_callback, 10, callback_group=self.reentrant_callback_group)
            self.robots[robot_name]['slam_status'] = SlamStatus()  # type: SlamStatus
            self.robots[robot_name]['min_timestamp'] = self.min_times[self.robot_names.index(robot_name)]
            self.robots[robot_name]['max_timestamp'] = self.max_times[self.robot_names.index(robot_name)]
            if self.robots[robot_name]['min_timestamp'] < self.start_time:
                self.robots[robot_name]['min_timestamp'] = self.start_time
            if self.robots[robot_name]['max_timestamp'] > self.end_time:
                self.robots[robot_name]['max_timestamp'] = self.end_time
            print(
                f'Robot {robot_name} max timestamp: {self.robots[robot_name]["max_timestamp"]}')
            self.robots[robot_name]['dump_service_client'] = self.create_client(
                DumpGraph, '/' + robot_name + '/hdl_graph_slam/dump', callback_group=self.reentrant_callback_group)
            self.robots[robot_name]['save_map_client'] = self.create_client(
                SaveMap, '/' + robot_name + '/hdl_graph_slam/save_map', callback_group=self.reentrant_callback_group)
            self.robots[robot_name]['slam_process'] = None  # type: subprocess.Popen
            self.robots[robot_name]['dump_graph_requested'] = False
            self.robots[robot_name]['dump_graph_done'] = False
            self.robots[robot_name]['save_map_requested'] = False
            self.robots[robot_name]['save_map_done'] = False
            self.robots[robot_name]['result_dir'] = os.path.join(self.result_dir, self.sequence, robot_name + '_' + self.eval_name)

        self.clock_pub = self.create_publisher(Clock, 'clock', 10)

        # gt_path_topic = self.robot_name + '/gt_path'
        # self.gt_path_pub = self.create_publisher(Path, gt_path_topic, 10, callback_group=self.reentrant_callback_group)
        self.dataset = pykitti.odometry(self.base_path, self.sequence)  # type: pykitti.odometry

        self.timestamps = self.dataset.timestamps  # type: list[datetime.timedelta]
        self.timestamps = np.array([ts.total_seconds() for ts in self.timestamps])
        print(f'Min dataset time {self.timestamps[0]} max dataset time {self.timestamps[-1]}')

        cam_gt_poses = self.dataset.poses  # type: list[np.ndarray]
        self.velo_gt_poses = [np.linalg.inv(self.dataset.calib.T_cam0_velo) @ pose for pose in cam_gt_poses]
        # transform the poses so that the first pose is the identity
        self.velo_gt_poses = np.array([pose @ np.linalg.inv(self.velo_gt_poses[0]) for pose in self.velo_gt_poses])

        self.slam_process = None  # type: subprocess.Popen
        self.progress_bar = None  # type: tqdm

    def start_playback(self):
        # create the results directory
        if not os.path.exists(os.path.join(self.result_dir, self.sequence)):
            os.makedirs(os.path.join(self.result_dir, self.sequence))

        # set the point cloud counter to the index of the first timestamp that is greater than the min timestamp
        self.point_cloud_counter = 0
        # self.point_cloud_idx_max = -1
        self.point_cloud_counter_reversed = len(self.timestamps) - 1
        for robot_name in self.robot_names:
            print(f'\n\n{robot_name}')
            if not os.path.exists(self.robots[robot_name]['result_dir']):
                print(f'Creating result dir for robot {robot_name} at {self.robots[robot_name]["result_dir"]}')
                os.makedirs(self.robots[robot_name]['result_dir'])
            # Start the slam node in a subprocess, set the start position as well
            idx_min = np.abs(self.timestamps - self.robots[robot_name]['min_timestamp']).argmin()
            print(f'Found min timestamp {self.timestamps[idx_min]} at index {idx_min}')
            print(f'Pose for min timestamp\n{self.velo_gt_poses[idx_min]}')
            if robot_name == 'atlas':
                x, y, z = self.velo_gt_poses[idx_min][0:3, 3]
                print(f'rotmat {self.velo_gt_poses[idx_min][0:3, 0:3].reshape(3, 3)}')
                rotation = R.from_matrix(self.velo_gt_poses[idx_min][0:3, 0:3])  # type: R
                yaw, pitch, roll = rotation.as_euler('ZYX', degrees=False)
                print(f'x: {x}, y: {y}, z: {z}, roll: {np.rad2deg(roll)}, pitch: {np.rad2deg(pitch)}, yaw: {np.rad2deg(yaw)}')
                slam_cmd = [
                    'ros2', 'launch', 'hdl_graph_slam', 'hdl_multi_robot_graph_slam.launch.py', 'model_namespace:=' + robot_name, 'config:=' +
                    self.slam_config,
                    'x:=' + str(x), 'y:=' + str(y), 'z:=' + str(z),
                    'roll:=' + str(roll), 'pitch:=' + str(pitch), 'yaw:=' + str(yaw)]
            elif robot_name == 'bestla':
                # take the last pose
                x, y, z = self.velo_gt_poses[-1][0:3, 3]
                print(f'rotmat {self.velo_gt_poses[-1][0:3, 0:3].reshape(3, 3)}')
                rotation = R.from_matrix(self.velo_gt_poses[-1][0:3, 0:3])  # type: R
                yaw, pitch, roll = rotation.as_euler('ZYX', degrees=False)
                print(f'x: {x}, y: {y}, z: {z}, roll: {np.rad2deg(roll)}, pitch: {np.rad2deg(pitch)}, yaw: {np.rad2deg(yaw)}')
                slam_cmd = [
                    'ros2', 'launch', 'hdl_graph_slam', 'hdl_multi_robot_graph_slam.launch.py', 'model_namespace:=' + robot_name, 'config:=' +
                    self.slam_config,
                    'x:=' + str(x), 'y:=' + str(y), 'z:=' + str(z),
                    'roll:=' + str(roll), 'pitch:=' + str(pitch), 'yaw:=' + str(yaw)]
            with open(os.path.join(self.robots[robot_name]['result_dir'], 'slam.log'), mode='w') as std_log, \
                    open(os.path.join(self.robots[robot_name]['result_dir'], 'slam.err'), mode='w') as err_log:
                self.robots[robot_name]['slam_process'] = subprocess.Popen(
                    slam_cmd, stdout=std_log, stderr=err_log)  # type: subprocess.Popen
                print(f'Started slam node with pid {self.robots[robot_name]["slam_process"].pid} and cmd:')
                print(' '.join(slam_cmd))
            time.sleep(2)
        print(f'Starting playback with rate {self.rate:.2f}x')
        idx_max = np.abs(self.timestamps - max(self.max_times)).argmin()
        print(f'Found max timestamp {self.timestamps[idx_max]} at index {idx_max}')
        # get the index of the last timestamp that is less than the max timestamp
        self.progress_bar = tqdm(total=idx_max - self.point_cloud_counter, unit='point clouds', desc='Playback')
        self.task = Task.PLAYBACK
        self.task_timer = self.create_timer(1.0 / self.rate, self.task_timer_callback, callback_group=self.reentrant_callback_group)

    def task_timer_callback(self):
        self.task_timer.cancel()
        self.proceed = self.get_parameter('proceed').value
        if self.proceed:
            if self.task == Task.PLAYBACK:
                self.playback()
            elif self.task == Task.WAIT_SLAM_DONE:
                self.wait_slam_done()
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
        else:
            print('Paused')
        self.task_timer.reset()

    def slam_status_callback(self, msg: SlamStatus):
        self.robots[msg.robot_name]['slam_status'] = msg  # type: SlamStatus

    def wait_slam_done(self):
        if any([self.robots[robot_name]['slam_status'].in_optimization for robot_name in self.robot_names]) or \
                any([self.robots[robot_name]['slam_status'].in_loop_closure for robot_name in self.robot_names]):
            time.sleep(1)
            print('Slam is optimizing or in loop closure, waiting')
            return
        print('Slam is done, starting dump graph')
        self.task = Task.DUMP_GRAPH

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

    def publish_clock_msg(self, stamp: Time):
        clock_msg = Clock()
        clock_msg.clock = stamp
        self.clock_pub.publish(clock_msg)

    def playback(self):
        if any([self.robots[robot_name]['slam_status'].in_optimization for robot_name in self.robot_names]) or \
           any([self.robots[robot_name]['slam_status'].in_loop_closure for robot_name in self.robot_names]):
            return

        velo = self.dataset.get_velo(self.point_cloud_counter)  # type: np.ndarray
        velo_reversed = self.dataset.get_velo(self.point_cloud_counter_reversed)  # type: np.ndarray
        ts = self.timestamps[self.point_cloud_counter]
        ros_pcl = self.kitti_to_ros_point_cloud(ts, velo)
        ros_pcl_reversed = self.kitti_to_ros_point_cloud(ts, velo_reversed)

        self.publish_clock_msg(ros_pcl.header.stamp)
        # Publish the forward moving pointcloud for atlas
        if self.robots['atlas']['min_timestamp'] <= ts <= self.robots['atlas']['max_timestamp']:
            ros_pcl.header.frame_id = 'atlas/velodyne'
            self.robots['atlas']['point_cloud_pub'].publish(ros_pcl)
            print(f'Publishing point cloud for robot atlas with ts {ts}')
        if self.robots['bestla']['min_timestamp'] <= ts <= self.robots['bestla']['max_timestamp']:
            ros_pcl_reversed.header.frame_id = 'bestla/velodyne'
            ros_pcl_reversed.header.stamp = float_ts_to_ros_ts(ts)
            print(f'Publishing point cloud for robot bestla with ts {ts}')
            self.robots['bestla']['point_cloud_pub'].publish(ros_pcl_reversed)

        self.point_cloud_counter += 1
        self.point_cloud_counter_reversed -= 1

        self.progress_bar.update(1)

        if self.point_cloud_counter >= len(self.timestamps) or self.timestamps[self.point_cloud_counter] > self.end_time or \
                self.point_cloud_counter_reversed < 0 or self.timestamps[self.point_cloud_counter_reversed] < self.start_time:
            self.progress_bar.close()
            print('Finished playback and closed progress bar')
            # publish a clock message with an offset to trigger optimization once more
            self.publish_clock_msg(Time(sec=ros_pcl.header.stamp.sec + 2, nanosec=ros_pcl.header.stamp.nanosec))
            self.task = Task.WAIT_SLAM_DONE

    def perform_async_service_call(self, client, request, robot_name):
        while client.wait_for_service(timeout_sec=1.0) is False:
            print('service', client.srv_name, 'not available, waiting again...')

        print('calling async service', client.srv_name)
        future = client.call_async(request)
        if isinstance(request, DumpGraph.Request):
            future.add_done_callback(self.get_done_dump_graph_callback(robot_name))
        if isinstance(request, SaveMap.Request):
            future.add_done_callback(self.get_done_save_map_callback(robot_name))

    def get_done_dump_graph_callback(self, robot_name):
        def done_dump_graph_callback(future):
            result = future.result()
            print(f'Dump graph service call for robot {robot_name} success? {result.success}')
            self.robots[robot_name]['dump_graph_done'] = True
            if all([self.robots[robot_name]['dump_graph_requested'] for robot_name in self.robot_names]):
                print('All dump graph requests done, starting save map')
                self.task = Task.SAVE_MAP
        return done_dump_graph_callback

    def get_done_save_map_callback(self, robot_name):
        def done_save_map_callback(future):
            result = future.result()
            print(f'Save map service call for robot {robot_name} success? {result.success}')
            self.robots[robot_name]['save_map_done'] = True
            if all([self.robots[robot_name]['save_map_requested'] for robot_name in self.robot_names]):
                self.task = Task.SHUTDOWN_SLAM
        return done_save_map_callback

    def dump_graph(self):
        robot_to_dump = None
        for robot_name in self.robot_names:
            if self.robots[robot_name]['dump_graph_requested']:
                if self.robots[robot_name]['dump_graph_done']:
                    continue
                else:
                    break
            else:
                robot_to_dump = robot_name
                break
        if robot_to_dump is None:
            return
        # call the dumb and save graph service on hdl graph slam
        dump_request = DumpGraph.Request()
        dump_request.destination = os.path.join(self.robots[robot_name]['result_dir'], 'g2o')
        self.robots[robot_name]['dump_graph_requested'] = True
        print(f'Dumping graph at: {dump_request.destination}')
        self.perform_async_service_call(self.robots[robot_name]['dump_service_client'], dump_request, robot_name)

    def save_map(self):
        robot_to_save = None
        for robot_name in self.robot_names:
            if self.robots[robot_name]['save_map_requested']:
                if self.robots[robot_name]['save_map_done']:
                    continue
                else:
                    break
            else:
                robot_to_save = robot_name
                break
        if robot_to_save is None:
            return
        save_map_request = SaveMap.Request()
        save_map_request.destination = os.path.join(self.robots[robot_name]['result_dir'], 'map.pcd')
        save_map_request.resolution = self.map_resolution
        self.robots[robot_name]['save_map_requested'] = True
        print(f'Saving map with resolution {save_map_request.resolution} at: {save_map_request.destination}')
        self.perform_async_service_call(self.robots[robot_name]['save_map_client'], save_map_request, robot_name)

    def shutdown_slam(self):
        for robot_name in self.robot_names:
            if self.robots[robot_name]['slam_process'] is not None:
                print(f'Sending SIGINT to slam process for robot {robot_name}')
                try:
                    self.robots[robot_name]['slam_process'].send_signal(subprocess.signal.SIGINT)
                    self.robots[robot_name]['slam_process'].wait(timeout=10)
                except subprocess.TimeoutExpired:
                    print(f'Timeout expired for robot {robot_name} slam process, trying SIGTERM')
                    self.robots[robot_name]['slam_process'].terminate()
                    self.robots[robot_name]['slam_process'].wait(timeout=10)
                finally:
                    print(f'Robot {robot_name} slam process terminated')

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
    except Exception as e:
        print(e)
        kitti_processor.shutdown_slam()
        kitti_processor.shutdown_node()
    finally:
        print('Shutting down')
        kitti_processor.destroy_node()
        executor.shutdown()


if __name__ == '__main__':
    main()
