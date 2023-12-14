#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomToFile(Node):
    def __init__(self):
        super().__init__('odom_to_file')
        self.result_file = self.declare_parameter('result_file', '/tmp/odom.txt').value
        # Get the directory without the file name
        dir_name = os.path.dirname(self.result_file)
        if not os.path.exists(dir_name):
            print(f'Creating directory {dir_name}')
            os.makedirs(dir_name)
        with open(self.result_file, 'w') as f:
            print(f'Writing to odom {self.result_file}')
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscription  # prevent unused variable warning

        self.counter = 0
        self.every_nth = self.declare_parameter('every_n', 10).value

        print(f'Writing every {self.every_nth}th odom message to {self.result_file}')

    def odom_callback(self, msg):
        if self.counter % self.every_nth == 0:
            tx = msg.pose.pose.position.x
            ty = msg.pose.pose.position.y
            tz = msg.pose.pose.position.z
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            with open(self.result_file, 'a') as f:
                # fill nanosec up to 9 digits
                nano_sec_str = str(msg.header.stamp.nanosec).zfill(9)
                f.write(f'{msg.header.stamp.sec}.{nano_sec_str} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n')
        self.counter += 1


def main():
    rclpy.init()
    odom_to_file = OdomToFile()

    try:
        rclpy.spin(odom_to_file)
    except KeyboardInterrupt:
        pass
    finally:
        odom_to_file.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
