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
# import colored function
from termcolor import colored, cprint


# reference: Quaternion kinematics for the error-state Kalman filter https://arxiv.org/pdf/1711.02508v1.pdf

G_EARTH = 9.81
ACC_START_STD = 0.15  # only needed when acceleration is part of the state
TURN_RATE_START_STD = 0.125  # degrees, only needed when turn rate is part of the state
# matrix positions error state
P = 0  # position
V = 3  # velocity
Q = 6  # orientation
A_B = 9  # acc bias
W_B = 12  # gyro bias
G = 15  # gravity
# matrix positions random impulses
A_N_I = 0
W_N_I = 3
A_W_I = 6
W_W_I = 9

I_3 = np.identity(3)
I_18 = np.eye(18)


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


# for some reason on different systems the point cloud message is numpified differently: array of tuples or dict
def pointcloud2_to_array(cloud_msg: PointCloud2, only_xyz=True, remove_nans_and_inf=True):
    points = rnp.numpify(cloud_msg)
    # numpify returns a numpy array of tuples which needs to be converted to a pure numpy array. a tuple is a single point
    if isinstance(points, np.ndarray):
        points = np.array([list(point) for row in points for point in row])
    if only_xyz:
        if isinstance(points, dict):
            points = points['xyz']
        elif isinstance(points, np.ndarray):
            points = points[:, 0:3]
    # remove all rows containing nans or inf
    if remove_nans_and_inf:
        points = points[~np.isnan(points).any(axis=1)]
        points = points[~np.isinf(points).any(axis=1)]
    return points


# eq. (281)
def error_quaternion_derivative(q: np.ndarray):
    if isinstance(q, R):
        q = q.as_quat()
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
        self.g = np.array([0.0, 0.0, -G_EARTH])
        self.g = self.g[:, np.newaxis]  # make it a column vector
        self.DOF = 18


class ErrorStateKalmanFilter:
    def __init__(self) -> None:
        # nominal state
        self.nominal_state = NominalState()
        self.last_nominal_state = None
        self.DOF = self.nominal_state.DOF

        # error state is created and injected in a measurement update, no need to initialize it
        self.error_cov = np.zeros((self.DOF, self.DOF))  # error covariance

        # current filter time
        self.time = None

        # noise parameters
        self.a_n = 0.04  # noise applied to velocity error
        self.w_n = 0.04  # noise applied to rotation error
        self.a_w = 0.00433  # noise applied to accelerometer bias
        self.w_w = 0.00255  # noise applied to gyroscope bias

        # this is the noise covariance matrix of the noise applied to the error state, eq. (271)
        self.Q = np.zeros((12, 12))
        self.Q[A_N_I:A_N_I+3, A_N_I:A_N_I+3] = np.diag([self.a_n, self.a_n, self.a_n])  # eq. (262)
        self.Q[W_N_I:W_N_I+3, W_N_I:W_N_I+3] = np.diag([self.w_n, self.w_n, self.w_n])  # eq. (263)
        self.Q[A_W_I:A_W_I+3, A_W_I:A_W_I+3] = np.diag([self.a_w, self.a_w, self.a_w])  # eq. (264)
        self.Q[W_W_I:W_W_I+3, W_W_I:W_W_I+3] = np.diag([self.w_w, self.w_w, self.w_w])  # eq. (265)

        # eq. (271)
        self.Fi = np.zeros((18, 12))
        self.Fi[V:V+3, A_N_I:A_N_I+3] = np.identity(3)
        self.Fi[Q:Q+3, W_N_I:W_N_I+3] = np.identity(3)
        self.Fi[A_B:A_B+3, A_W_I:A_W_I+3] = np.identity(3)
        self.Fi[W_B:W_B+3, W_W_I:W_W_I+3] = np.identity(3)

        # imu calibration
        self.imu_calibrated = False
        self.imu_calibration_max_samples = 5
        self.acc_samples = deque()
        self.turn_rate_samples = deque()
        self.time_samples = deque()

        self.scan_matching_update_translation_std = 0.01  # meters
        self.scan_matching_update_rotation_std = 1.0  # degree
        self.scan_matching_update_rotation_std = np.deg2rad(self.scan_matching_update_rotation_std)
        self.scan_matching_update_measurement_covariance = np.diag([
            self.scan_matching_update_translation_std ** 2, self.scan_matching_update_translation_std ** 2, self.scan_matching_update_translation_std ** 2,
            self.scan_matching_update_rotation_std ** 2, self.scan_matching_update_rotation_std ** 2, self.scan_matching_update_rotation_std ** 2
        ])
        self.V = self.scan_matching_update_measurement_covariance  # eq. (273) shorter name for convenience
        print(f'scan_matching_update_measurement_covariance\n{self.scan_matching_update_measurement_covariance}')

    def predict(self, dt: float, a_m: np.ndarray, w_m: np.ndarray):
        # for readability
        p, v, q, a_b, w_b, g = self.nominal_state.p, self.nominal_state.v, self.nominal_state.q, self.nominal_state.a_b, self.nominal_state.w_b, self.nominal_state.g

        # predict the nominal state
        self.nominal_state.p += v * dt + 0.5 * (q.as_matrix() @ (a_m - a_b) + g) * dt * dt
        self.nominal_state.v += (q.as_matrix() @ (a_m - a_b) + g) * dt
        self.nominal_state.q = q * R.from_rotvec(((w_m - w_b).flatten() * dt))
        # self.a_b += 0 # for completeness
        # self.w_b += 0 # for completeness
        # self.g += 0 # for completeness

        # The error state mean is initialized to zero. In the prediction step the error state mean stays zero: eq. (268)
        # x_e = F_nominal_state(nominal_state, move_input) * error_state

        # Construct the state transition matrix Fx in order to propagate the error state covariance matrix Pe: eq. (270)
        Fx = np.identity(self.DOF, dtype=float)  # type: np.ndarray
        # position
        # Fx[P:P+3, P:P+3] = I_3
        Fx[P:P+3, V:V+3] = I_3 * dt
        # velocity
        # Fx[V:V+3, V:V+3] = I_3
        Fx[V:V+3, Q:Q+3] = - q.as_matrix() @ skew(a_m - a_b) * dt
        Fx[V:V+3, A_B:A_B+3] = - q.as_matrix() * dt
        Fx[G:G+3, G:G+3] = I_3 * dt
        # rot part
        Fx[Q:Q+3, Q:Q+3] = R.from_rotvec(((w_m - w_b) * dt).flatten()).as_matrix().transpose()
        Fx[W_B:W_B+3, W_B:W_B+3] = - I_3 * dt
        # accel bias part
        # Fx[A_B:A_B+3, A_B:A_B+3] = I_3
        # gyro bias part
        # Fx[W_B:W_B+3, W_B:W_B+3] = I_3
        # gravity part
        # Fx[G:G+3, G:G+3] = I_3

        # Fx = self.predict_error_state_jacobian(x)  # type: np.ndarray
        # Fi = self.predict_noise_jacobian()  # type: np.ndarray
        Qi = self.Q  # type: np.ndarray
        Qi[A_N_I:A_N_I+3, A_N_I:A_N_I+3] *= dt * dt
        Qi[W_N_I:W_N_I+3, W_N_I:W_N_I+3] *= dt * dt
        Qi[A_W_I:A_W_I+3, A_W_I:A_W_I+3] *= dt
        Qi[W_W_I:W_W_I+3, W_W_I:W_W_I+3] *= dt

        # Progate the error state covariance matrix Pe: eq. (269)
        self.error_cov = Fx @ self.error_cov @ Fx.transpose() + self.Fi @ Qi @ self.Fi.transpose()

    def predict_upto(self, time_upto: float) -> bool:
        if not self.imu_calibrated:
            return False
        if self.time is None:
            # get the imu sample before the first time_upto, once after imu is calibrated
            for time in deepcopy(self.time_samples):
                if time <= time_upto:
                    self.time = self.time_samples.popleft()
                    a_m = self.acc_samples.popleft()
                    w_m = self.turn_rate_samples.popleft()
                    # print(f'Setting first prediction sample at time {self.time} for time_upto {time_upto}')
                elif time > time_upto:
                    break

        print(f'\nStarting prediction upto time {time_upto} - self.time {self.time} = {time_upto-self.time}')
        if time_upto < self.time:
            raise ValueError(f'Time to predict {time_upto} < filter time {self.time}')

        if len(self.time_samples) == 0:
            print(f'No more IMU samples to predict upto time {time_upto}')
            return False

        while self.time < time_upto:
            print(f'len time samples {len(self.time_samples)} time_samples {self.time_samples}')
            if time_upto > self.time_samples[0]:
                print(
                    f'Predicting one step with self.time {self.time} and dt {self.time_samples[0] - self.time} and time_samples[0] {self.time_samples[0]}')
                dt = self.time_samples[0] - self.time
                self.time = self.time_samples[0]
            elif time_upto <= self.time_samples[0]:
                print(f'Predicting one step with self.time {self.time} and dt {self.time_samples[0] - self.time} and time_upto {time_upto}')
                dt = time_upto - self.time
                self.time = time_upto
            if dt < 0:
                raise ValueError(f'dt {dt} < 0 in predict_upto, filter time {self.time}')
            # print(f'\nPredicting one step with self.time {self.time} and dt {dt}')
            a_m = self.acc_samples[0]
            w_m = self.turn_rate_samples[0]
            self.predict(dt, a_m, w_m)
            # pop the last time, acc, and turn rate samples, note that self.time is set above
            self.time_samples.popleft()
            a_m = self.acc_samples.popleft()
            w_m = self.turn_rate_samples.popleft()

        print(f'Finished predict_upto: time_upto {time_upto} - self.time = {self.time} = {time_upto - self.time}')

        return True

    # calibrate imu assuming horizontal starting pose
    def calibrate_imu(self):
        # calculate the mean and covariance of the acc and turn rate samples
        self.a_b = np.array(self.acc_samples).mean(axis=0).reshape((3, 1))
        self.a_b[2] -= G_EARTH  # correct the gravity (assuming horizontal)
        self.w_b = np.array(self.turn_rate_samples).mean(axis=0).reshape((3, 1))
        print(f'{len(self.acc_samples)} imu samples used for bias calibration')
        print(f'acc bias: {self.a_b.flatten()}')
        print(f'turn rate bias {self.w_b.flatten()}')

        self.imu_calibrated = True

    def insert_imu_sample(self, acc: np.ndarray, turn_rate: np.ndarray, time: float):
        self.acc_samples.append(acc)
        self.turn_rate_samples.append(turn_rate)
        self.time_samples.append(time)
        if not self.imu_calibrated:
            if len(self.acc_samples) == self.imu_calibration_max_samples:
                self.calibrate_imu()
            return
        if len(self.time_samples) > 0 and time < self.time_samples[-1]:
            raise ValueError(f'Inserting IMU sample, Current time {time} < last time {self.time_samples[-1]}, IMU out of order.')

    # Perform a scan matching update, where z is the relative transformation in R^(4x4) between the last and current point cloud
    def scan_matching_update(self, z: np.ndarray, R_LI: np.ndarray, t_LI: np.ndarray):
        # if self.last_nominal_state is None:
        #     self.last_nominal_state = deepcopy(self.nominal_state)
        #     print(colored('First scan matching update, saving nominal state', 'green'))
        #     return
        R_WL_k_1 = self.last_nominal_state.q.as_matrix()  # R from LIDAR to WORLD at k-1
        R_WL_k = self.nominal_state.q.as_matrix()  # R from LIDAR to WORLD at k
        t_WL_k_1 = self.last_nominal_state.p  # translation from LIDAR to WORLD at k-1
        t_WL_k = self.nominal_state.p  # translation from LIDAR to WORLD at k

        # construct the measurement function
        h = np.zeros((6, 1))
        h[0:3] = (R_LI.transpose() @ (R_WL_k_1.transpose() @ R_WL_k @ t_LI + R_WL_k_1 @ (t_WL_k - t_WL_k_1) - t_LI)).reshape((3, 1))
        h_rot_mat = R_LI.transpose() @  R_WL_k_1.transpose() @ R_WL_k @ R_LI
        h[3:6] = R.from_matrix(h_rot_mat).as_rotvec().reshape((3, 1))

        print(f'measurement function {np.array2string(h.flatten(), precision=2, max_line_width=np.inf)}\nmeasurement          {np.array2string(z.flatten(), precision=2, max_line_width=np.inf)}')

        # calculate the Kalman gain
        # construct the Jacobian H_nominal_state in R^(6x18) of h_trans and h_rot wrt to last nominal state eq. (278) H_x
        # 6 DOF measurement = 3 DOF (translation) + 3 DOF (rotation)
        # 18 DOF for the nominal state
        # Note that we use hard coded values for slicing H_nominal state below concerning the rows of H_nominal state
        # Otherwise use the state entries for the columns which correspond to the actual state matrix entries
        H_nominal_state = np.zeros((6, self.last_nominal_state.DOF))

        # h_trans wrt state translation
        H_nominal_state[0:3, P:P+3] = -R_LI.transpose() @ R_WL_k_1
        # h_trans wrt state rotation
        H_nominal_state[0:3, Q:Q+3] = R_LI.transpose() @ R_WL_k_1.transpose() @ skew(R_WL_k @ t_LI + t_WL_k - t_WL_k_1) @ R_WL_k_1
        # h_rot wrt state translation
        # H_nominal_state[3:6, P:P+3] = np.zeros((3, 3))
        # h_rot wrt state rotation
        H_nominal_state[3:6, Q:Q+3] = - (R_WL_k @ R_LI).transpose() @ R_WL_k_1
        print(f'scan matching update H_nominal_state\n{np.array2string( H_nominal_state[0:9, 0:9], precision=2, max_line_width=np.inf)}')
        H_error_state = np.identity(self.DOF)
        H_error_state[Q:Q+4, Q:Q+3] = error_quaternion_derivative(self.nominal_state.q)  # make sure nominal q is up to date

        # Final correction jacobian eq. (278)
        H_full = H_nominal_state @ H_error_state
        # TODO find out whether it is possible to retrieve the covariance of the GICP alignment

        # Kalman gain eq. (274)
        kalman_gain = self.error_cov @ H_full.transpose() @ np.linalg.inv((H_full @ self.error_cov @ H_full.transpose() + self.V))
        print(f'Kalman gain {np.array2string(kalman_gain, precision=2, max_line_width=np.inf)}')

        error_state = kalman_gain @ (z-h)
        print(f'error state {np.array2string( error_state.flatten(), precision=2, max_line_width=np.inf)}')

        self.print_state('before update')
        self.inject_error_state(error_state)
        self.print_state('after update')
        self.reset_error_state(error_state)
        # Joseph form of covariance propagation eq. (276)
        self.error_cov = (np.identity(self.DOF) - kalman_gain @ H_full) @ self.error_cov @ (np.identity(self.DOF) - kalman_gain @
                                                                                            H_full).transpose() + kalman_gain @ self.V @ kalman_gain.transpose()

        self.last_nominal_state = deepcopy(self.nominal_state)

    def print_state(self, header=None):
        if header is not None:
            print(header)
        print(f'position          {np.array2string(self.nominal_state.p.flatten(), precision=6)}')
        print(f'velocity          {np.array2string(self.nominal_state.v.flatten(), precision=6)}')
        print(f'quaternion        {np.array2string(self.nominal_state.q.as_quat(), precision=6)}')
        print(f'acceleration bias {np.array2string(self.nominal_state.a_b.flatten(), precision=6)}')
        print(f'gyro bias         {np.array2string(self.nominal_state.w_b.flatten(), precision=6)}')
        print(f'gravity           {np.array2string(self.nominal_state.g.flatten(), precision=6)}')

    def inject_error_state(self, error_state: np.ndarray):
        self.nominal_state.p += error_state[P:P+3]
        self.nominal_state.v += error_state[V:V+3]
        self.nominal_state.q = R.from_matrix(self.last_nominal_state.q.as_matrix() @
                                             R.from_rotvec(error_state[Q:Q+3].flatten()).as_matrix())
        self.nominal_state.a_b += error_state[A_B:A_B+3]
        self.nominal_state.w_b += error_state[W_B:W_B+3]
        self.nominal_state.g += error_state[G:G+3]

    def reset_error_state(self, error_state):
        rot_error = error_state[Q:Q+3]
        # eq. (285) setting the error_state to 0 which is not necessary
        # self.error_state <- 0
        G = np.identity(self.DOF)
        G[Q:Q+3, Q:Q+3] = np.identity(3) - skew(0.5 * rot_error)  # eq. 287, can be neglected
        # Reset the error state covariance eq. (286)
        self.error_cov = G @ self.error_cov @ G.transpose()
        print(f'error covariance after reset:\n{np.array2string( self.error_cov, precision=2, max_line_width=np.inf)}')


class LidarImuOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_imu_odometry')
        imu_topic = self.declare_parameter('imu_topic', 'imu/data').value
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)

        lidar_topic = self.declare_parameter('lidar_topic', 'velodyne_points').value
        self.lidar_sub = self.create_subscription(PointCloud2, lidar_topic, self.lidar_callback, 10)

        self.error_state_kalman_filter = ErrorStateKalmanFilter()

        # Pose publisher
        self.pose_topic = self.declare_parameter('pose_topic', 'pose').value
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)

        # Odometry publisher
        self.odom_topic = self.declare_parameter('odom_topic', 'odom').value
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # Path publisher
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.path_topic = self.declare_parameter('path_topic', 'path').value
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)

        # extrinsic calibration of IMU to LIDAR, this is IMU to Body/LIDAR -> R_LI, t_LI
        # kimera multi [ 0.5, -0.5, 0.5, -0.5 ] quaternion
        self.imu_extrinsic_quat = self.declare_parameter('imu_extrinsic_rotation', [0.0, 0.0, 0.0, 1.0]).value
        self.imu_extrinsic_quat = R.from_quat(self.imu_extrinsic_quat)
        self.R_LI = self.imu_extrinsic_quat.as_matrix()
        self.t_LI = self.declare_parameter('imu_extrinsic_translation', [0.0, 0.0, 0.0]).value
        self.t_LI = np.array(self.t_LI)[:, np.newaxis]  # make it a column vector

        self.downsample_resolution = self.declare_parameter('downsample_resolution', 0.05).value

        # Scan matching Path publisher
        self.scan_matching_path_pub = self.create_publisher(Path, 'scan_matching_path', 10)
        self.scan_matching_path = Path()
        self.pure_scan_matching_pose = np.identity(4)
        # Initial guess Path publisher
        self.init_guess_path_pub = self.create_publisher(Path, 'init_guess_path', 10)
        self.init_guess_path = Path()
        self.init_guess_pose = np.identity(4)

        # Setup scan matching registration
        # Consult README.md of fast_gicp for more information
        self.registration_method = pygicp.FastGICP()
        self.registration_method.set_num_threads(8)
        self.registration_method.set_max_correspondence_distance(2.0)
        self.registration_method.set_correspondence_randomness(20)

        # Setup ground truth path publisher if available
        self.gt_path_file = self.declare_parameter('gt_path_file', '').value
        self.publish_ground_truth = False
        if os.path.exists(self.gt_path_file):
            self.publish_ground_truth = True
            self.gt_delimiter = self.declare_parameter('gt_delimiter', ' ').value
            self.gt_quat_format = self.declare_parameter('gt_format', 'qxqyqzqw').value
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

    def init_guess_from_states(self):
        init_guess = np.identity(4)
        last_pos = self.error_state_kalman_filter.last_nominal_state.p
        last_rot = self.error_state_kalman_filter.last_nominal_state.q.as_matrix()
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
            # Predict the state
            float_ts = ros_stamp_to_float_ts(msg.header.stamp)
            self.last_time_upto = float_ts
            prediction_valid = self.error_state_kalman_filter.predict_upto(float_ts)
            if not prediction_valid:
                print(colored(f'prediction not valid at time {float_ts}', 'yellow'))

            self.last_points = pointcloud2_to_array(msg)
            self.last_points = pygicp.downsample(self.last_points, self.downsample_resolution)
            self.registration_method.set_input_target(self.last_points)

            # # save the current estimated pose as the initial pose for the point cloud matching, prediction will be done
            # self.last_nominal_state = deepcopy(self.error_state_kalman_filter.nominal_state)
            self.error_state_kalman_filter.last_nominal_state = deepcopy(self.error_state_kalman_filter.nominal_state)
            return

        # predict the error state kalmann filter
        float_ts = ros_stamp_to_float_ts(msg.header.stamp)
        if float_ts < self.last_time_upto:
            raise ValueError(f'new time upto {float_ts} < last time upto {self.last_time_upto}')
        if float_ts > self.error_state_kalman_filter.time_samples[-1]:
            print(f'\nCannot perform prediction upto time {float_ts} > latest imu time \
                    {self.error_state_kalman_filter.time_samples[-1]}, returning')
            return
        prediction_valid = self.error_state_kalman_filter.predict_upto(float_ts)
        if not prediction_valid:
            print(colored(f'prediction not valid at time {float_ts}', 'yellow'))

        # calculate the alignment between last and current point cloud
        self.current_points = pointcloud2_to_array(msg)
        self.current_points = pygicp.downsample(self.current_points, self.downsample_resolution)
        self.registration_method.set_input_source(self.current_points)

        # GICP
        init_guess = self.init_guess_from_states()
        delta_transformation = self.registration_method.align(initial_guess=init_guess)

        print(f'GICP transformation:\n{delta_transformation}')
        print(f'init guess:\n{init_guess}')

        self.pure_scan_matching_pose = self.pure_scan_matching_pose @ delta_transformation
        self.init_guess_pose = self.init_guess_pose @ init_guess

        R_init_guess = R.from_matrix(init_guess[:3, :3])
        R_registration = R.from_matrix(delta_transformation[:3, :3])
        delta_angle = np.rad2deg((R_init_guess.inv() * R_registration).magnitude())
        delta_translation = np.linalg.norm((init_guess[0:3, 3] - delta_transformation[0:3, 3]))

        if delta_angle > 20 or delta_translation > 0.5 or (delta_translation == 0 and delta_angle == 0):
            print(f'recalculating gicp with init guess = identity delta_pos {delta_translation} delta_angle (deg) {delta_angle}')
            delta_transformation = self.registration_method.align(initial_guess=np.identity(4))

        # print(f'initial guess from states:\n{init_guess}')
        # print(f'GICP delta transformation:\n{delta_transformation}')
        # print(f'delta translation {delta_translation} delta angle (deg) {delta_angle}')

        # update the nominal state with the estimated transformation TODO
        z_vec = np.zeros((6, 1))
        z_vec[0:3] = delta_transformation[0:3, 3].reshape((3, 1))
        z_vec[3:6] = R.from_matrix(delta_transformation[:3, :3]).as_rotvec().reshape((3, 1))
        self.error_state_kalman_filter.scan_matching_update(z_vec, self.R_LI, self.t_LI)

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
