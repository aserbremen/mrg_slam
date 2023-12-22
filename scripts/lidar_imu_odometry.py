import numpy as np
from scipy.spatial.transform import Rotation as R
from enum import Enum

# import pcl python
import pcl

import rclpy
from rclpy import Node
from sensor_msgs.msg import Imu, PointCloud2

NORM_G = 9.81
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


def skew(v: np.ndarray):
    return np.ndarray(
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    )


def error_quaternion_derivative(q: np.ndarray):
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


class State(object):
    def __init__(self):
        self.p = np.zeros((3, 1))  # position
        self.v = np.zeros((3, 1))  # velocity
        self.q = R.identity()  # quaternion
        self.a_b = np.zeros((3, 1))
        self.w_b = np.zeros((3, 1))
        self.g = np.array([0.0, 0.0, NORM_G])
        self.DOF = 18
        self.cov = np.zeros((self.DOF, self.DOF))


class ErrorState(object):
    def __init__(self) -> None:
        self.x = State()  # nominal state
        self.pe = np.zeros((3, 1))
        self.ve = np.zeros((3, 1))
        self.theta = np.zeros((3, 1))  # error rotation in R^3
        self.a_b_e = np.zeros((3, 1))
        self.w_b_e = np.zeros((3, 1))
        self.ge = np.zeros((3, 1))
        self.DOF = 18
        self.Pe = np.zeros((self.DOF, self.DOF))  # error covariance
        self.time = 0.0

        # measured accel and turn rate
        self.a_m = np.zeros((3, 1))
        self.w_m = np.zeros((3, 1))
        self.time = None

        self.a_n = 1e-3  # noise applied to velocity error
        self.w_n = 1e-4  # noise applied to rotation error
        self.a_w = 1e-4  # noise applied to accelerometer bias
        self.w_w = 1e-5  # noise applied to gyroscope bias

        self.Q = np.zeros((12, 12))
        self.Q[A_N_I:A_N_I+3, A_N_I:A_N_I+3] = np.diag([self.a_n, self.a_n, self.a_n])
        self.Q[W_N_I:W_N_I+3, W_N_I:W_N_I+3] = np.diag([self.w_n, self.w_n, self.w_n])
        self.Q[A_W_I:A_W_I+3, A_W_I:A_W_I+3] = np.diag([self.a_w, self.a_w, self.a_w])
        self.Q[W_W_I:W_W_I+3, W_W_I:W_W_I+3] = np.diag([self.w_w, self.w_w, self.w_w])
        print(f'Q\n{self.Q}')

    def predict_error_state_jacobian(self):
        Fx = np.identity((self.DOF, self.DOF), dtype=float)  # type: np.ndarray
        # position
        I = np.identity(3)
        # Fx[P:P+3, P:P+3] = I
        Fx[P:P+3, V:V+3] = I * self.dt
        # velocity
        # Fx[V:V+3, V:V+3] = I
        Fx[V:V+3, Q:Q+3] = - self.x.q * skew(self.a_m - self.x.a_b) @ self.theta * self.dt
        Fx[V:V+3, A_B:A_B+3] = - self.x.q * self.dt
        Fx[G:G+3, G:G+3] = I * self.dt
        # rot part
        Fx[Q:Q+3, Q:Q+3] = R.from_rotvec((self.w_m - self.x.w_b) * self.dt).as_matrix()
        Fx[W_B:W_B+3, W_B:W_B+3] = - I * self.dt
        # accel bias part
        # Fx[A_B:A_B+3, A_B:A_B+3] = I
        # gyro bias part
        # Fx[W_B:W_B+3, W_B:W_B+3] = I
        # gravity part
        # Fx[G:G+3, G:G+3] = I

        print(f'Fx\n{Fx}')
        return Fx

    def predict_noise_jacobian(self):
        Fi = np.zeros((18, 12))
        Fi[V:V+3, A_N_I:A_N_I+3] = np.identity(3)
        Fi[Q:Q+3, W_N_I:W_N_I+3] = np.identity(3)
        Fi[A_B:A_B+3, A_W_I:A_W_I+3] = np.identity(3)
        Fi[W_B:W_B+3, W_W_I:W_W_I+3] = np.identity(3)
        print(f'Fi\n{Fi}')
        return Fi

    def update_error_state_jacobian(self):
        X_error_state = np.identity((self.DOF, self.DOF))
        X_error_state[Q:Q+3, Q:Q+3] = error_quaternion_derivative(self.x.q)  # make sure nominal q is up to date
        print(f'X_error_state\n{X_error_state}')

    def set_acc(self, acc):
        self.a_m = acc

    def set_turn_rate(self, turn_rate):
        self.w_m = turn_rate

    def set_time(self, t):
        if self.time is not None:
            self.dt = t - self.time
            self.predict(self.dt)
        self.time = t

    # a_m measured acceleration
    # w_m measured turn rate

    def predict(self):
        # dt = self.time - t
        self.pe += self.ve * self.dt
        self.ve += (- self.x.q * skew(self.a_m - self.x.a_b) @ self.theta - self.x.q * self.a_b_e + self.ge) * self.dt
        self.theta += R.from_rotvec((self.w_m - self.x.w_b)*self.dt) * self.theta - self.w_b_e * self.dt
        # for completeness
        # self.a_b_e += a_i # a_i random impulse
        # self.w_b_e += w_i  # w_i random impulse
        # self.ge += 0

        Fx = self.predict_error_state_jacobian(self.a_m, self.w_m, self.dt)  # type: np.ndarray
        Fi = self.predict_noise_jacobian()  # type: np.ndarray
        self.Pe = Fx @ self.Pe @ Fx.transpose() + Fi @ self.Q @ Fi.transpose()


class LidarImuOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_imu_odometry')
        imu_topic = self.declare_parameter('imu_topic', 'imu')
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)

        lidar_topic = self.declare_parameter('lidar_topic', 'velodyne_points')
        self.lidar_sub = self.create_subscription(PointCloud2, lidar_topic, self.lidar_callback, 10)

        self.error_state = ErrorState()

    def imu_callback(self, msg: Imu):
        self.error_state.set_time(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.error_state.set_acc(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]))
        self.error_state.set_turn_rate(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]))

    def lidar_callback(self, msg: PointCloud2):
        # get the current nominal state as an input for point cloud matching
        x = self.error_state.x
