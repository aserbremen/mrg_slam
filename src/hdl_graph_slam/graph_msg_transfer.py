#!/usr/bin/python

import os
import rospy
import rosbag
from hdl_graph_slam.msg import GraphRos


class GraphMsgTransfer():
    def __init__(self):
        self.subscriber = rospy.Subscriber('/hdl_graph_slam/graph_broadcast', GraphRos, self.graph_ros_callback, queue_size=1)
        self.publisher = rospy.Publisher('/hdl_graph_slam/graph_broadcast', GraphRos, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.graph_receive_timer)
        self.bag_counter = 0
        self.remote_name = rospy.get_param('remote_name')
        self.received_file_size = -1

    def graph_ros_callback(self, graph_ros_msg: GraphRos):
        bag_name_record = '/tmp/graph_record_%d.bag' % self.bag_counter
        bag_name_receive = '/tmp/graph_receive_%d.bag' % self.bag_counter
        self.bag_counter += 1

        bag = rosbag.Bag(bag_name_record, 'w')
        try:
            bag.write('/hdl_graph_slam/graph_broadcast', graph_ros_msg)
        finally:
            bag.close()
            os.system('rsync %s %s:%s' % (bag_name_record, self.remote_name, bag_name_receive))

    def graph_receive_timer(self, event):
        # Check received bag file size is not changing
        # If it is not changing, then it is done transferring
        bag_stats = os.stat(self.bag_name_receive)
        self.received_file_size = bag_stats.st_size
        if self.received_file_size > 0 and self.received_file_size == self.received_file_size:
            bag = rosbag.Bag(self.bag_name_receive, 'r')
            for topic, msg, t in bag.read_messages(topics=['/hdl_graph_slam/graph_broadcast']):
                self.publisher.publish(msg)
                break
            bag.close()


if __name__ == "__main__":
    node = GraphMsgTransfer()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()
