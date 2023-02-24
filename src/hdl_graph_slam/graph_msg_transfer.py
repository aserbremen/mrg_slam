#!/usr/bin/python

import os
import rospy
import rosbag
import glob
from hdl_graph_slam.msg import GraphRos


class GraphMsgTransfer():
    def __init__(self):
        self.subscriber = rospy.Subscriber('/hdl_graph_slam/graph_broadcast', GraphRos, self.graph_ros_callback, queue_size=1)
        self.publisher = rospy.Publisher('/hdl_graph_slam/graph_broadcast', GraphRos, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.graph_receive_timer)
        self.last_bag_received = None
        self.cur_bag_received = None
        self.bag_counter = 0
        self.robot_name = 'atlas'
        self.received_file_size = -1

    def graph_ros_callback(self, graph_ros_msg: GraphRos):
        bag_name_record = '/tmp/graph_record_%05d.bag' % self.bag_counter
        bag_name_receive = '/tmp/graph_receive_%05d.bag' % self.bag_counter
        self.bag_counter += 1

        bag = rosbag.Bag(bag_name_record, 'w')
        try:
            bag.write('/hdl_graph_slam/graph_broadcast', graph_ros_msg)
        finally:
            bag.close()
            os.system('rsync %s %s:%s' % (bag_name_record, self.robot_name, bag_name_receive))
            print("Transfered graph bag file to other rover: %s -> " % (bag_name_record. bag_name_receive))

    def graph_receive_timer(self, event):
        # Check received bag file size is not changing
        # If it is not changing, then it is done transferring
        if not self.cur_bag_received:
            file_list = glob.glob("/tmp/graph_receive_*.bag")
            if len(file_list) == 0:
                return
            file_list.sort()
            if file_list[-1] != self.last_bag_received:
                self.cur_bag_received = file_list[-1]
        bag_stats = os.stat(self.cur_bag_received)
        size = bag_stats.st_size
        if size > 0 and size == self.received_file_size:
            print("Received new graph bag file from other rover:  %s (%d bytes)" % (self.cur_bag_received, size))
            bag = rosbag.Bag(self.cur_bag_received, 'r')
            for topic, msg, t in bag.read_messages(topics=['/hdl_graph_slam/graph_broadcast']):
                self.publisher.publish(msg)
                break
            bag.close()
            os.remove(self.cur_bag_received)
            self.received_file_size = -1
            self.last_bag_received = self.cur_bag_received
            self.cur_bag_received = None
        else:
            self.received_file_size = size


if __name__ == "__main__":
    node = GraphMsgTransfer()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()
