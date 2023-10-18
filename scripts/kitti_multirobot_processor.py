import os
import fire
import datetime

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField
from vamex_slam_msgs.msg import SlamStatus

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
        self.base_path = self.declare_parameter('base_path', '/data/datasets/kitti_dataset/data_odometry_velodyne/dataset/').value
        self.sequence = self.declare_parameter('sequence', '00').value
        self.rate = self.declare_parameter('rate', 10.0).value
        self.reentrant_callback_group = ReentrantCallbackGroup()

        self.point_cloud_pub = self.create_publisher(PointCloud2, 'velodyne_points', 10, callback_group=self.reentrant_callback_group)
        self.clock_pub = self.create_publisher(Clock, 'clock', 10)
        self.point_cloud_counter = 0

        slam_status_topic = '/' + self.robot_name + '/hdl_graph_slam/slam_status'
        self.slam_status_sub = self.create_subscription(
            SlamStatus, slam_status_topic, self.slam_status_callback, 10, callback_group=self.reentrant_callback_group)

        self.dataset = pykitti.odometry(self.base_path, self.sequence)

    def slam_status_callback(self, msg: SlamStatus):
        self.slam_status = msg

    def start_playback(self):
        print(f'Starting playback with rate {self.rate:.2f}x')
        self.timestamps = self.dataset.timestamps  # type: list[datetime.timedelta]
        assert (len(self.timestamps) == len(self.dataset.velo))
        self.timer = self.create_timer(1.0 / self.rate, self.playback_timer,
                                       callback_group=self.reentrant_callback_group)

    def playback_timer(self):
        self.timer.cancel()

        if self.slam_status.in_optimization or self.slam_status.in_loop_closure:
            if self.print_info_once:
                print('Waiting for optimization to finish')
            self.timer.reset()
            self.print_info_once = False
            return
        self.print_info_once = True

        velo = self.dataset.get_velo(self.point_cloud_counter)  # type: np.ndarray
        ts = self.timestamps[self.point_cloud_counter]
        # create a ROS2 PointCloud2 message
        ros_pcl = PointCloud2()
        ros_pcl.header.stamp = pykitti_ts_to_ros_ts(ts)
        print(ros_pcl.header.stamp.sec, ros_pcl.header.stamp.nanosec)
        ros_pcl.header.frame_id = 'velodyne'
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

        # publish the clock msg
        clock_msg = Clock()
        clock_msg.clock = ros_pcl.header.stamp
        self.clock_pub.publish(clock_msg)
        # publish the point cloud
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
        pass

    def plot_trajectories(self):
        pass


def plot_trajectories(executor, kitti_processor):
    kitti_processor.plot_trajectories()
    spin(executor, kitti_processor)


def start_playback(executor, kitti_processor):
    kitti_processor.start_playback()
    spin(executor, kitti_processor)


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
    executor = MultiThreadedExecutor()
    kitti_processor = KittiMultiRobotProcessor()
    executor.add_node(kitti_processor)

    fire.Fire({
        'plot_trajectories': lambda: plot_trajectories(executor, kitti_processor),
        'start_playback': lambda: start_playback(executor, kitti_processor)
    })


if __name__ == '__main__':
    main()
