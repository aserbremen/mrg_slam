// SPDX-License-Identifier: BSD-2-Clause

#ifndef MARKER_PUBLISHER_HPP
#define MARKER_PUBLISHER_HPP

#include <ros/ros.h>

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>


namespace hdl_graph_slam {

class MarkersPublisher {
public:
    void onInit( ros::NodeHandle &nh, ros::NodeHandle &mt_nh, ros::NodeHandle &private_nh );

    void publish( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes,
                  double loop_detector_distance_thresh );

    uint32_t getNumSubscribers() const { return markers_pub.getNumSubscribers(); }

private:
    ros::Publisher markers_pub;
    std::string    map_frame_id;
};

}  // namespace hdl_graph_slam


#endif  // MARKER_PUBLISHER_HPP