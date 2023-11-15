// SPDX-License-Identifier: BSD-2-Clause

#ifndef MARKER_PUBLISHER_HPP
#define MARKER_PUBLISHER_HPP

// #include <ros/ros.h>
// #include <std_msgs/ColorRGBA.h>
#include <hdl_graph_slam/edge.hpp>
#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hdl_graph_slam {

class MarkersPublisher {
public:
    // void onInit( ros::NodeHandle &nh, ros::NodeHandle &mt_nh, ros::NodeHandle &private_nh );
    void onInit( rclcpp::Node::SharedPtr _node );

    void publish( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes, const std::vector<Edge::Ptr> &edges,
                  const KeyFrame::ConstPtr &last_keyframe, const std::vector<KeyFrame::ConstPtr> &others_last_kf,
                  double loop_detector_distance_thresh );

    void publishMarginals( const std::vector<KeyFrame::Ptr> &keyframes, const std::shared_ptr<g2o::SparseBlockMatrixX> &marginals );

    // uint32_t getNumSubscribers() const { return markers_pub.getNumSubscribers(); }
    // uint32_t getNumMarginalsSubscribers() const { return markers_marginals_pub.getNumSubscribers(); }
    size_t getNumSubscribers() const { return markers_pub->get_subscription_count(); }
    size_t getNumMarginalsSubscribers() const { return markers_marginals_pub->get_subscription_count(); }


private:
    // ros::Publisher markers_pub;
    // ros::Publisher markers_marginals_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_node_names_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_marginals_pub;

    rclcpp::Node::SharedPtr node;

    std::string map_frame_id;
    std::string own_name;

    std_msgs::msg::ColorRGBA color_blue, color_orange, color_green, color_red, color_purple, color_brown, color_pink, color_olive,
        color_cyan, color_black, color_white, color_gray;
};

}  // namespace hdl_graph_slam


#endif  // MARKER_PUBLISHER_HPP