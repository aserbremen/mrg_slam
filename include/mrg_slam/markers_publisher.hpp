// SPDX-License-Identifier: BSD-2-Clause

#ifndef MARKER_PUBLISHER_HPP
#define MARKER_PUBLISHER_HPP

#include <mrg_slam/edge.hpp>
#include <mrg_slam/graph_slam.hpp>
#include <mrg_slam/keyframe.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace mrg_slam {

class MarkersPublisher {
public:
    void onInit( rclcpp::Node::SharedPtr node );

    void publish( std::shared_ptr<GraphSLAM> &graph_slam, const boost::uuids::uuid &own_slam_uuid,
                  const std::vector<KeyFrame::Ptr> &keyframes, const std::vector<Edge::Ptr> &edges, const KeyFrame::ConstPtr &last_keyframe,
                  const std::vector<KeyFrame::ConstPtr> &others_last_kf );

    void publishMarginals( const boost::uuids::uuid &own_slam_uuid, const std::vector<KeyFrame::Ptr> &keyframes,
                           const std::shared_ptr<g2o::SparseBlockMatrixX> &marginals );

    size_t getNumSubscribers() const { return markers_pub->get_subscription_count(); }
    size_t getNumMarginalsSubscribers() const { return markers_marginals_pub->get_subscription_count(); }


private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_marginals_pub;

    std_msgs::msg::ColorRGBA color_blue, color_orange, color_green, color_red, color_purple, color_brown, color_pink, color_olive,
        color_cyan, color_black, color_white, color_dark_gray, color_light_gray;
};

}  // namespace mrg_slam


#endif  // MARKER_PUBLISHER_HPP