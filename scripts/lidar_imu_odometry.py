#!/usr/bin/env python3

import os

import numpy as np
from scipy.spatial.transform import Rotation as R
from enum import Enum

import ros2_numpy as rnp

import pygicp
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import TransformBroadcaster, TransformStamped
from enum import Enum

from collections import deque
from copy import deepcopy
# reference: Quaternion kinematics for the error-state Kalman filter https://arxiv.org/pdf/1711.02508v1.pdf

NORM_G = 9.81
ACC_START_STD = 0.15  # only needed when acceleration is part of the state
TURN_RATE_START_STD = 0.125  # degrees, only needed when turn rate is part of the state
# matrix positions error state
P = 0
V = 3
Q = 6
A_B = 9
W_B = 12
G = 15
# matrix positions random impulses
A_N_I = 0
W_N_I = 3
A_W_I = 6
W_W_I = 9

I = np.identity(3)


def float_ts_to_ros_stamp(ts: float):
    ros_ts = Time()
    ros_ts.sec = int(ts)
    ros_ts.nanosec = int((ts - ros_ts.sec) * 1e9)
    return ros_ts


def ros_stamp_to_float_ts(ros_ts: Time):
    return float(ros_ts.sec + ros_ts.nanosec * 1e-9)


def skew(v: np.ndarray):
    vflat = v.flatten()
    return np.array([
        [0.0, -vflat[2], vflat[1]],
        [vflat[2], 0.0, -vflat[0]],
        [-vflat[1], vflat[0], 0.0]
    ])


def pointcloud2_to_array(cloud_msg: PointCloud2, only_xyz=True):
    points = rnp.numpify(cloud_msg)
    # points = np.array([list(point) for point in points]).reshape(-1, 5)
    if only_xyz:
        points = points['xyz']
    return points


def error_quaternion_derivative(q: np.ndarray):  # Equation 281
    result = np.ndarray((4, 3))
    qx = q[0]
    qy = q[1]
    qz = q[2]
    qw = q[3]
    result = 0.5 * np.array([
        [-qx, -qy, -qz],
        [qw, -qz, qy],
        [qz, qw, -qx],
        [-qy, qx, qw],
    ])
    return result


class NominalState:
    def __init__(self):
        self.p = np.zeros((3, 1))  # position
        self.v = np.zeros((3, 1))  # velocity
        self.q = R.identity()  # quaternion
        self.a_b = np.zeros((3, 1))
        self.w_b = np.zeros((3, 1))
        self.g = np.array([0.0, 0.0, -NORM_G])
        self.g = self.g[:, np.newaxis]  # make it a column vector
        self.DOF = 18
        self.cov = np.zeros((self.DOF, self.DOF))

    # a_m: measured acceleration
    # dt: time step
    def predict(self, a_m: np.ndarray, w_m: np.ndarray, dt: float):
        self.p += self.v * dt + 0.5 * (self.q.as_matrix() @ (a_m - self.a_b) + self.g) * dt * dt
        self.v += (self.q.as_matrix() @ (a_m - self.a_b) + self.g) * dt
        self.q = self.q * R.from_rotvec(((w_m - self.w_b).flatten() * dt))
        # self.a_b += 0 # for completeness
        # self.w_b += 0 # for completeness
        # self.g += 0 # for completeness


class ErrorState:
    def __init__(self) -> None:
        self.pe = np.zeros((3, 1))
        self.ve = np.zeros((3, 1))
        self.theta = np.zeros((3, 1))  # error rotation in R^3
        self.a_b_e = np.zeros((3, 1))
        self.w_b_e = np.zeros((3, 1))
        self.ge = np.zeros((3, 1))
        self.DOF = 18
        self.Pe = np.zeros((self.DOF, self.DOF))  # error covariance

        # measured accel and turn rate
        # self.a_m = np.zeros((3, 1))
        # self.w_m = np.zeros((3, 1))
        # self.time = None

        self.a_n = 0.04  # noise applied to velocity error
        self.w_n = 0.04  # noise applied to rotation error
        self.a_w = 0.00433  # noise applied to accelerometer bias
        self.w_w = 0.00255  # noise applied to gyroscope bias

        # this is the noise covariance matrix of the noise applied to the error state
        self.Q = np.zeros((12, 12))
        self.Q[A_N_I:A_N_I+3, A_N_I:A_N_I+3] = np.diag([self.a_n, self.a_n, self.a_n])  # equation 262
        self.Q[W_N_I:W_N_I+3, W_N_I:W_N_I+3] = np.diag([self.w_n, self.w_n, self.w_n])  # equation 263
        self.Q[A_W_I:A_W_I+3, A_W_I:A_W_I+3] = np.diag([self.a_w, self.a_w, self.a_w])  # equation 264
        self.Q[W_W_I:W_W_I+3, W_W_I:W_W_I+3] = np.diag([self.w_w, self.w_w, self.w_w])  # equation 265

        self.Fi = self.perturbation_vector_jacobian()  # type: np.ndarray # equation 271

    def perturbation_vector_jacobian(self):
        # equation 271
        Fi = np.zeros((18, 12))
        Fi[V:V+3, A_N_I:A_N_I+3] = np.identity(3)
        Fi[Q:Q+3, W_N_I:W_N_I+3] = np.identity(3)
        Fi[A_B:A_B+3, A_W_I:A_W_I+3] = np.identity(3)
        Fi[W_B:W_B+3, W_W_I:W_W_I+3] = np.identity(3)
        return Fi

    def update_error_state_jacobian(self):
        X_error_state = np.identity((self.DOF, self.DOF))
        X_error_state[Q:Q+3, Q:Q+3] = error_quaternion_derivative(self.x.q)  # make sure nominal q is up to date

    def predict(self, x: NominalState, a_m: np.ndarray, w_m: np.ndarray, dt: float):
        # The error state mean is initialized to zero. In the prediction step the error state mean stays zero: equation 268
        # x_e = Fx * x_e + Fi * w_i # equation 268

        # self.pe += self.ve * self.dt
        # self.ve += (- x.q.as_matrix() @ skew(self.a_m - x.a_b) @ self.theta - x.q.as_matrix() @ self.a_b_e + self.ge) * self.dt
        # self.theta += R.from_rotvec((self.w_m - x.w_b) * self.dt) * self.theta - self.w_b_e * self.dt
        # for completeness
        # self.a_b_e += a_i # a_i random impulse
        # self.w_b_e += w_i  # w_i random impulse
        # self.ge += 0

        # Construct the state transition matrix Fx in order to propagate the error state covariance matrix Pe: equation 270
        Fx = np.identity(self.DOF, dtype=float)  # type: np.ndarray
        # position
        # Fx[P:P+3, P:P+3] = I
        Fx[P:P+3, V:V+3] = I * dt
        # velocity
        # Fx[V:V+3, V:V+3] = I
        Fx[V:V+3, Q:Q+3] = - x.q.as_matrix() @ skew(a_m - x.a_b) @ self.theta * dt
        Fx[V:V+3, A_B:A_B+3] = - x.q.as_matrix() * dt
        Fx[G:G+3, G:G+3] = I * dt
        # rot part
        Fx[Q:Q+3, Q:Q+3] = R.from_rotvec(((w_m - x.w_b) * dt).flatten()).as_matrix().transpose()
        Fx[W_B:W_B+3, W_B:W_B+3] = - I * dt
        # accel bias part
        # Fx[A_B:A_B+3, A_B:A_B+3] = I
        # gyro bias part
        # Fx[W_B:W_B+3, W_B:W_B+3] = I
        # gravity part
        # Fx[G:G+3, G:G+3] = I

        # Fx = self.predict_error_state_jacobian(x)  # type: np.ndarray
        # Fi = self.predict_noise_jacobian()  # type: np.ndarray
        Qi = self.Q  # type: np.ndarray
        Qi[A_N_I:A_N_I+3, A_N_I:A_N_I+3] *= dt * dt
        Qi[W_N_I:W_N_I+3, W_N_I:W_N_I+3] *= dt * dt
        Qi[A_W_I:A_W_I+3, A_W_I:A_W_I+3] *= dt
        Qi[W_W_I:W_W_I+3, W_W_I:W_W_I+3] *= dt

        # Progate the error state covariance matrix Pe: equation 269
        self.Pe = Fx @ self.Pe @ Fx.transpose() + self.Fi @ Qi @ self.Fi.transpose()


class ErrorStateKalmanFilter:
    def __init__(self) -> None:
        self.nominal_state = NominalState()  # type: NominalState
        self.error_state = ErrorState()  # type: ErrorState
        self.a_m = np.zeros((3, 1))
        self.w_m = np.zeros((3, 1))
        self.time = None
        self.imu_calibrated = False
        self.imu_calibration_num_samples = 0
        self.imu_calibration_max_samples = 800
        self.imu_calibration_acc_samples = []
        self.imu_calibration_turn_rate_samples = []
        self.imu_calibration_time_samples = []
        self.acc_samples = deque()
        self.turn_rate_samples = deque()
        self.time_samples = deque()

    def predict(self, dt: float):
        # print(f'position before prediction: {self.nominal_state.p.flatten()}')
        # print(f'velocity before prediction: {self.nominal_state.v.flatten()}')
        # print(f'orientation before prediction: {self.nominal_state.q.as_quat()}')
        self.nominal_state.predict(self.a_m, self.w_m, dt)
        # print(f'position after prediction: {self.nominal_state.p.flatten()}')
        # print(f'velocity after prediction: {self.nominal_state.v.flatten()}')
        # print(f'orientation after prediction: {self.nominal_state.q.as_quat()}')
        self.error_state.predict(self.nominal_state, self.a_m, self.w_m, dt)

    def predict_upto(self, time_upto: float):
        if not self.imu_calibrated:
            return
        if self.time is None:
            # get the imu sample before the first time_upto
            for time in deepcopy(self.time_samples):
                if time <= time_upto:
                    self.time = self.time_samples.popleft()
                    self.a_m = self.acc_samples.popleft()
                    self.w_m = self.turn_rate_samples.popleft()
                    # print(f'Setting first prediction sample at time {self.time} for time_upto {time_upto}')
                elif time > time_upto:
                    break

        print(f'\n\nStarting prediction upto time {time_upto} - self.time {self.time} = {time_upto - self.time} ')
        if time_upto < self.time:
            raise ValueError(f'Time to predict {time_upto} < filter time {self.time}')
        while self.time < time_upto:
            if time_upto > self.time_samples[0]:
                dt = self.time_samples[0] - self.time
                self.time = self.time_samples[0]
            elif time_upto <= self.time_samples[0]:
                dt = time_upto - self.time
                self.time = time_upto
            if dt < 0:
                raise ValueError(f'dt {dt} < 0 in predict_upto')
            # print(f'\nPredicting one step with self.time {self.time} and dt {dt}')
            self.predict(dt)
            # pop the last time, acc, and turn rate samples, note that self.time is set above
            self.time_samples.popleft()
            self.a_m = self.acc_samples.popleft()
            self.w_m = self.turn_rate_samples.popleft()

        print(f'Finished predict_upto: time_upto {time_upto} - self.time = {self.time} = {time_upto - self.time}')

    # calibrate imu assuming horizontal starting pose
    def calibrate_imu(self):
        # calculate the mean and covariance of the acc and turn rate samples
        self.nominal_state.a_b = np.array(self.imu_calibration_acc_samples).mean(axis=0).reshape((3, 1))
        self.nominal_state.a_b[2] -= NORM_G  # correct the gravity (assuming horizontal)
        self.nominal_state.w_b = np.array(self.imu_calibration_turn_rate_samples).mean(axis=0).reshape((3, 1))
        print(f'{len(self.imu_calibration_acc_samples)} imu samples used for bias calibration')
        print(f'acc bias: {self.nominal_state.a_b.flatten()}')
        print(f'turn rate bias {self.nominal_state.w_b.flatten()}')

        self.acc_samples.extend(self.imu_calibration_acc_samples)
        self.turn_rate_samples.extend(self.imu_calibration_turn_rate_samples)
        self.time_samples.extend(self.imu_calibration_time_samples)

        self.imu_calibrated = True

    def insert_imu_sample(self, acc: np.ndarray, turn_rate: np.ndarray, time: float):
        if not self.imu_calibrated:
            self.imu_calibration_acc_samples.append(acc)
            self.imu_calibration_turn_rate_samples.append(turn_rate)
            self.imu_calibration_time_samples.append(time)
            if len(self.imu_calibration_acc_samples) == self.imu_calibration_max_samples:
                self.calibrate_imu()
            return
        self.acc_samples.append(acc)
        self.turn_rate_samples.append(turn_rate)
        if len(self.time_samples) > 0 and time < self.time_samples[-1]:
            raise ValueError(f'Inserting IMU sample, Current time {time} < last time {self.time_samples[-1]}, IMU out of order.')
        self.time_samples.append(time)

    def update(self, z: np.ndarray):
        pass


class LidarImuOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_imu_odometry')
        imu_topic = self.declare_parameter('imu_topic', 'imu/data').value
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)

        lidar_topic = self.declare_parameter('lidar_topic', 'velodyne_points').value
        self.lidar_sub = self.create_subscription(PointCloud2, lidar_topic, self.lidar_callback, 10)

        self.error_state_kalman_filter = ErrorStateKalmanFilter()

        self.prediction_counter = 0

        self.pose_topic = self.declare_parameter('pose_topic', 'pose').value
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)

        self.odom_topic = self.declare_parameter('odom_topic', 'odom').value
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        self.path = Path()
        self.path.header.frame_id = 'map'
        self.path_topic = self.declare_parameter('path_topic', 'path').value
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)

        # kimera multi [ 0.5, -0.5, 0.5, -0.5 ] quaternion
        self.imu_extrinsic_quat = self.declare_parameter('imu_extrinsic_rotation', [0.0, 0.0, 0.0, 1.0]).value
        self.imu_extrinsic_quat = R.from_quat(self.imu_extrinsic_quat)

        self.downsample_resolution = self.declare_parameter('downsample_resolution', 0.1).value

        self.scan_matching_path_pub = self.create_publisher(Path, 'scan_matching_path', 10)
        self.init_guess_path_pub = self.create_publisher(Path, 'init_guess_path', 10)
        self.scan_matching_path = Path()
        self.init_guess_path = Path()
        self.pure_scan_matching_pose = np.identity(4)
        self.init_guess_pose = np.identity(4)

        # Setup scan matching registration
        # Consult README.md of fast_gicp for more information
        self.registration_method = pygicp.FastGICP()
        self.registration_method.set_num_threads(8)
        self.registration_method.set_max_correspondence_distance(2.0)
        self.registration_method.set_correspondence_randomness(20)

        self.gt_path_file = self.declare_parameter('gt_path_file', '').value
        if os.path.exists(self.gt_path_file):
            self.gt_delimiter = self.declare_parameter('gt_delimiter', ' ').value
            self.gt_quat_format = self.declare_parameter('gt_format', 'qxqyqzqw').value
            self.publish_ground_truth = True
            self.gt_path_topic = self.declare_parameter('gt_path_topic', 'gt_path').value
            self.gt_path_pub = self.create_publisher(Path, self.gt_path_topic, 10)
            self.gt_path = Path()
            self.gt_time_format = self.declare_parameter('gt_time_format', 'seconds').value
            with open(self.gt_path_file, 'r') as f:
                # usual format is timestamp x y z qx qy qz qw
                # in kimera multi dataset the format is timestamp x y z qw qx qy qz
                print(f'loading gt_path_file {self.gt_path_file}')
                if self.gt_delimiter == 'comma':
                    self.gt_delimiter = ','
                gt_mat = np.genfromtxt(self.gt_path_file, delimiter=self.gt_delimiter)
                if self.gt_quat_format == 'qwqxqyqz':
                    print(f'fixing quaternion format for gt_path_file {self.gt_path_file}')
                    gt_mat = gt_mat[:, [0, 1, 2, 3, 7, 4, 5, 6]]
                if self.gt_time_format != 'seconds':
                    print(f'fixing timestamp format for gt_path_file {self.gt_path_file}')
                    if self.gt_time_format == 'milliseconds':
                        gt_mat[:, 0] = gt_mat[:, 0] * 1e-3
                    elif self.gt_time_format == 'microseconds':
                        gt_mat[:, 0] = gt_mat[:, 0] * 1e-6
                    elif self.gt_time_format == 'nanoseconds':
                        gt_mat[:, 0] = gt_mat[:, 0] * 1e-9
                    else:
                        raise ValueError(f'unknown gt_time_format {self.gt_time_format}')

                # transform into origin at first pose
                print('Transforming gt path to origin at first pose')
                first_pos = np.copy(gt_mat[0, 1:4])
                first_quat = R.from_quat(np.copy(gt_mat[0, 4:8]))
                first_rot_mat_inv = first_quat.as_matrix().transpose()
                for i in range(gt_mat.shape[0]):
                    old_pos = gt_mat[i][1:4]
                    old_rot_mat = R.from_quat(gt_mat[i][4:8]).as_matrix()
                    transformed_pos = first_rot_mat_inv @ (old_pos - first_pos)
                    transformed_rot_mat = first_rot_mat_inv @ old_rot_mat
                    gt_mat[i][1:4] = transformed_pos.flatten()
                    gt_mat[i][4:8] = R.from_matrix(transformed_rot_mat).as_quat()
                self.gt_mat = gt_mat

        self.last_points = None

    def initialized(self):
        return self.error_state_kalman_filter.imu_calibrated

    def imu_callback(self, msg: Imu):
        # the imu data needs to be rotated to the correct frame, I believe it is the following transformation
        # np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]]), but I am not sure TODO
        # [ 0.5, -0.5, 0.5, -0.5 b] quaternion
        a_m = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        a_m = a_m[:, np.newaxis]
        a_m = self.imu_extrinsic_quat.as_matrix() @ a_m
        w_m = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        w_m = w_m[:, np.newaxis]
        w_m = self.imu_extrinsic_quat.as_matrix() @ w_m
        float_ts = ros_stamp_to_float_ts(msg.header.stamp)

        self.error_state_kalman_filter.insert_imu_sample(a_m, w_m, float_ts)
        # self.error_state_kalman_filter.set_acc(a_m)
        # self.error_state_kalman_filter.set_turn_rate(w_m)
        # self.error_state_kalman_filter.set_time(float_ts)
        # self.error_state_kalman_filter.predict()
        # self.publish_all()

    # def voxel_grid_filter_point_cloud(self, cloud: pcl.PointCloud):
    #     vg = cloud.make_voxel_grid_filter()
    #     vg.set_leaf_size(0.2, 0.2, 0.2)
    #     cloud = vg.filter()
    #     return cloud

    def init_guess_from_states(self):
        init_guess = np.identity(4)
        last_pos = self.last_nominal_state.p
        last_rot = self.last_nominal_state.q.as_matrix()
        current_pos = self.error_state_kalman_filter.nominal_state.p
        current_rot = self.error_state_kalman_filter.nominal_state.q.as_matrix()
        init_guess[0:3, 0:3] = last_rot.transpose() @ current_rot
        init_guess[0:3, 3] = last_rot.transpose() @ (current_pos - last_pos).flatten()
        # init_guess[0:3, 0:3] = current_rot.transpose() @ last_rot
        # init_guess[0:3, 3] = current_rot.transpose() @ (last_pos - current_pos).flatten()

        init_guess = np.array(init_guess, dtype=np.float32)
        return init_guess

    def lidar_callback(self, msg: PointCloud2):
        if not self.initialized():
            return
        # get the current nominal state as an input for point cloud matching, assume XYZI point cloud
        if self.last_points is None:
            self.last_points = pointcloud2_to_array(msg)
            self.last_points = pygicp.downsample(self.last_points, self.downsample_resolution)
            self.registration_method.set_input_target(self.last_points)

            # save the current estimated pose as the initial pose for the point cloud matching, prediction will be done
            float_ts = ros_stamp_to_float_ts(msg.header.stamp)
            self.last_time_upto = float_ts
            self.error_state_kalman_filter.predict_upto(float_ts)
            self.last_nominal_state = deepcopy(self.error_state_kalman_filter.nominal_state)
            return

        # predict the error state kalmann filter
        float_ts = ros_stamp_to_float_ts(msg.header.stamp)
        if float_ts < self.last_time_upto:
            raise ValueError(f'new time upto {float_ts} < last time upto {self.last_time_upto}')
        self.error_state_kalman_filter.predict_upto(float_ts)

        # calculate the alignment between last and current point cloud
        self.current_points = pointcloud2_to_array(msg)
        self.current_points = pygicp.downsample(self.current_points, self.downsample_resolution)
        self.registration_method.set_input_source(self.current_points)

        # GICP

        init_guess = self.init_guess_from_states()

        delta_transformation = self.registration_method.align(initial_guess=init_guess)

        self.pure_scan_matching_pose = self.pure_scan_matching_pose @ delta_transformation
        self.init_guess_pose = self.init_guess_pose @ init_guess

        R_init_guess = R.from_matrix(init_guess[:3, :3])
        R_registration = R.from_matrix(delta_transformation[:3, :3])
        delta_angle = np.rad2deg((R_init_guess.inv() * R_registration).magnitude())
        delta_tranlsation = np.linalg.norm((init_guess[0:3, 3] - delta_transformation[0:3, 3]))
        print(f'initial guess from states:\n{init_guess}')
        print(f'GICP delta transformation:\n{delta_transformation}')
        print(f'delta translation {delta_tranlsation} delta angle (deg) {delta_angle}')

        # update the nominal state with the estimated transformation TODO
        self.registration_method.swap_source_and_target()
        self.last_nominal_state = deepcopy(self.error_state_kalman_filter.nominal_state)

        self.publish_all()

    def publish_all(self):
        if not self.initialized():
            return
        self.publish_pose()
        self.publish_scan_matching_pose()
        self.publish_init_guess_pose()
        self.publish_odometry()
        self.publish_tf()
        self.publish_estimate_path()
        self.publish_gt_path()

    def pose_from_matrix(self, mat: np.ndarray):
        pose = Pose()
        pose.position.x = mat[0, 3]
        pose.position.y = mat[1, 3]
        pose.position.z = mat[2, 3]
        qx, qy, qz, qw = R.from_matrix(mat[:3, :3]).as_quat()
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def publish_scan_matching_pose(self):
        pose = self.pose_from_matrix(self.pure_scan_matching_pose)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = float_ts_to_ros_stamp(self.error_state_kalman_filter.time)
        pose_stamped.pose = pose
        self.scan_matching_path.header.frame_id = 'map'
        self.scan_matching_path.header.stamp = float_ts_to_ros_stamp(self.error_state_kalman_filter.time)
        self.scan_matching_path.poses.append(pose_stamped)
        self.scan_matching_path_pub.publish(self.scan_matching_path)

    def publish_init_guess_pose(self):
        pose = self.pose_from_matrix(self.init_guess_pose)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = float_ts_to_ros_stamp(self.error_state_kalman_filter.time)
        pose_stamped.pose = pose
        self.init_guess_path.header.frame_id = 'map'
        self.init_guess_path.header.stamp = float_ts_to_ros_stamp(self.error_state_kalman_filter.time)
        self.init_guess_path.poses.append(pose_stamped)
        self.init_guess_path_pub.publish(self.init_guess_path)

    def pose_stamped_from_state(self):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = float_ts_to_ros_stamp(self.error_state_kalman_filter.time)
        x, y, z = self.error_state_kalman_filter.nominal_state.p.flatten()
        qx, qy, qz, qw = self.error_state_kalman_filter.nominal_state.q.as_quat()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw
        return pose_stamped

    def pose_from_state(self):
        pose = Pose()
        x, y, z = self.error_state_kalman_filter.nominal_state.p.flatten()
        qx, qy, qz, qw = self.error_state_kalman_filter.nominal_state.q.as_quat()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def publish_pose(self):
        self.pose_pub.publish(self.pose_stamped_from_state())

    def publish_odometry(self):
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = float_ts_to_ros_stamp(self.error_state_kalman_filter.time)
        vx, vy, vz = self.error_state_kalman_filter.nominal_state.v.flatten()
        odom.pose.pose = self.pose_from_state()
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        self.odom_pub.publish(odom)

    def publish_gt_path(self):
        if not self.publish_ground_truth:
            return
        # get the closest gt pose to the current time
        time = self.error_state_kalman_filter.time
        idx = (np.abs(self.gt_mat[:, 0] - time)).argmin()
        gt_time = self.gt_mat[idx, 0]
        gt_pos = self.gt_mat[idx, 1:4]
        gt_quat = self.gt_mat[idx, 4:8]
        self.gt_path.header.frame_id = 'map'
        self.gt_path.header.stamp = float_ts_to_ros_stamp(gt_time)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = float_ts_to_ros_stamp(gt_time)
        pose.pose.position.x = gt_pos[0]
        pose.pose.position.y = gt_pos[1]
        pose.pose.position.z = gt_pos[2]
        pose.pose.orientation.x = gt_quat[0]
        pose.pose.orientation.y = gt_quat[1]
        pose.pose.orientation.z = gt_quat[2]
        pose.pose.orientation.w = gt_quat[3]

        self.gt_path.poses.append(pose)

        self.gt_path_pub.publish(self.gt_path)

    def publish_estimate_path(self):
        self.path.poses.append(self.pose_stamped_from_state())
        self.path_pub.publish(self.path)

    def publish_tf(self):
        pass


if __name__ == '__main__':
    rclpy.init()
    node = LidarImuOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
