#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
import sys
import rclpy
from rclpy.node import Node
import rclpy.logging
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock

class ClockPublisherRos2(Node):
    # Use this node to publish the clock from the hdl_graph_slam_400 rosbag (containing imu messages) to the /clock topic.
    def __init__(self):
        super().__init__('clock_publisher_ros2')
        self.declare_parameter("imu_topic", "/imu/data")
        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        rclpy.logging.get_logger("clock_publisher_ros2").info(
            "Going to publish time on /clock whenever receiving imu messages on topic " + imu_topic)
        self.imu_subscription = self.create_subscription(
            msg_type=Imu,
            topic=imu_topic,
            callback=self.imu_callback,
            qos_profile=10,
        )
        self.imu_subscription  # prevent unused variable warning

        self.clock_publisher = self.create_publisher(
            msg_type=Clock,
            topic="/clock",
            qos_profile=10
        )

    def imu_callback(self, imu_msg: Imu):
        clock_msg = Clock()
        clock_msg.clock = imu_msg.header.stamp
        self.clock_publisher.publish(clock_msg)


def main(args=None):
    rclpy.init(args=args)

    clock_publisher_ros2_node = ClockPublisherRos2()
    try:
        rclpy.spin(clock_publisher_ros2_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        clock_publisher_ros2_node.destroy_node()


if __name__ == '__main__':
    main()
