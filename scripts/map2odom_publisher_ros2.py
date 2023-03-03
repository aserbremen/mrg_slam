#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import rclpy.logging
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3


class Map2OdomPublisher(Node):
    def __init__(self):
        super().__init__('map2odom_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(TransformStamped, '/hdl_graph_slam/odom2pub', self.odom_callback, 10)
        self.subscription  # prevent unused variable warning
        # create a timer for simplicity instead of creating a ROS2 spin inside its own thread, see
        # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def odom_callback(self, odom_msg: TransformStamped):
        self.odom_msg = odom_msg

    def timer_callback(self):
        if not hasattr(self, 'odom_msg'):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.translation = Vector3(x=0.0, y=0.0, z=0.0)
            t.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            self.tf_broadcaster.sendTransform(t)
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_msg.header.frame_id
        t.child_frame_id = self.odom_msg.child_frame_id
        t.transform.translation = self.odom_msg.transform.translation
        t.transform.rotation = self.odom_msg.transform.rotation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    map2odom_publisher_node = Map2OdomPublisher()
    try:
        rclpy.spin(map2odom_publisher_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        map2odom_publisher_node.destroy_node()


if __name__ == '__main__':
    main()
