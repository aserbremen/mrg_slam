#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
import sys
import rclpy
from rclpy.node import Node
import rclpy.logging
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.clock import Clock
from rclpy.duration import Duration

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Transform2OdomForwarder(Node):
    def __init__(self):
        super().__init__('transform2odom_forwarder')
        self.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.source_frame = self.declare_parameter('source_frame', 'world').value
        self.robot_names = self.declare_parameter('robot_names', ['atlas']).value

        self.odom_publishers = {}
        for robot_name in self.robot_names:
            self.odom_publishers[robot_name] = self.create_publisher(Odometry, f'{robot_name}/vtb_odom', 10)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.01, self.timer_callback)

        # Variable to check if sim time is active
        self.sim_time_active = False

    def timer_callback(self):
        # Check if the clock is using simulation time
        if not self.sim_time_active:
            if self.get_clock().ros_time_is_active:
                self.get_logger().info("Simulation time is now active.")
                self.sim_time_active = True
            else:
                # If sim time isn't active yet, wait
                self.get_logger().warn("Waiting for simulation time to become active...")
                return
        self.timer.cancel()

        for robot_name in self.robot_names:
            try:
                duration = Duration(seconds=3, nanoseconds=0)
                transform = self.tf_buffer.lookup_transform(
                    robot_name, self.source_frame, self.get_clock().now().to_msg(),
                    timeout=duration)
            except TransformException as e:
                self.get_logger().error(f'Failed to lookup transform from {self.source_frame} to {robot_name}: {e}')
                continue
            odom_msg = Odometry()
            odom_msg.header.stamp = transform.header.stamp
            odom_msg.header.frame_id = robot_name + '/odom'
            odom_msg.child_frame_id = robot_name + '/base_link'
            odom_msg.pose.pose.position.x = transform.transform.translation.x
            odom_msg.pose.pose.position.y = transform.transform.translation.y
            odom_msg.pose.pose.position.z = transform.transform.translation.z
            odom_msg.pose.pose.orientation.x = transform.transform.rotation.x
            odom_msg.pose.pose.orientation.y = transform.transform.rotation.y
            odom_msg.pose.pose.orientation.z = transform.transform.rotation.z
            odom_msg.pose.pose.orientation.w = transform.transform.rotation.w
            self.odom_publishers[robot_name].publish(odom_msg)
        self.timer.reset()


def main(args=None):
    rclpy.init(args=args)

    transform2odom_forwarder_node = Transform2OdomForwarder()
    try:
        rclpy.spin(transform2odom_forwarder_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        transform2odom_forwarder_node.destroy_node()


if __name__ == '__main__':
    main()
