// SPDX-License-Identifier: BSD-2-Clause

#ifndef IMU_PROCESSOR_HPP
#define IMU_PROCESSOR_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>

#include <deque>
#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <mutex>


namespace hdl_graph_slam {

class ImuProcessor {
public:
    ImuProcessor() : private_nh( nullptr ) {}

    void onInit( ros::NodeHandle &nh, ros::NodeHandle &mt_nh, ros::NodeHandle &private_nh );

    void imu_callback( const sensor_msgs::ImuPtr &imu_msg );

    bool flush( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes, const std::string &base_frame_id );

private:
    ros::NodeHandle *private_nh;
    ros::Subscriber  imu_sub;

    tf::TransformListener tf_listener;

    double                               imu_time_offset;
    bool                                 enable_imu_orientation;
    double                               imu_orientation_edge_stddev;
    bool                                 enable_imu_acceleration;
    double                               imu_acceleration_edge_stddev;
    std::mutex                           imu_queue_mutex;
    std::deque<sensor_msgs::ImuConstPtr> imu_queue;
};

}  // namespace hdl_graph_slam

#endif  // IMU_PROCESSOR_HPP