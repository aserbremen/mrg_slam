// SPDX-License-Identifier: BSD-2-Clause

#ifndef MARKER_PUBLISHER_HPP
#define MARKER_PUBLISHER_HPP

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>

#include <hdl_graph_slam/edge.hpp>
#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>


namespace hdl_graph_slam {

class MarkersPublisher {
public:
    void onInit( ros::NodeHandle &nh, ros::NodeHandle &mt_nh, ros::NodeHandle &private_nh );

    void publish( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes, const std::vector<Edge::Ptr> &edges,
                  const KeyFrame::ConstPtr                                               &last_keyframe,
                  const std::vector<std::pair<KeyFrame::ConstPtr, geometry_msgs::Pose> > &others_last_kf_and_pose,
                  double loop_detector_distance_thresh, const GlobalIdGenerator &gid_gen );

    uint32_t getNumSubscribers() const { return markers_pub.getNumSubscribers(); }

private:
    ros::Publisher markers_pub;
    std::string    map_frame_id;
    std::string    own_name;

    std_msgs::ColorRGBA color_blue, color_orange, color_green, color_red, color_purple, color_brown, color_pink, color_olive, color_cyan,
        color_black, color_white, color_gray;
};

}  // namespace hdl_graph_slam


#endif  // MARKER_PUBLISHER_HPP