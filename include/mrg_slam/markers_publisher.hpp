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

    size_t getNumSubscribers() const { return markers_pub_->get_subscription_count(); }
    size_t getNumMarginalsSubscribers() const { return markers_marginals_pub_->get_subscription_count(); }


private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_marginals_pub_;

    std_msgs::msg::ColorRGBA color_blue_, color_orange_, color_green_, color_red_, color_purple_, color_brown_, color_pink_, color_olive_,
        color_cyan_, color_black_, color_white_, color_dark_gray_, color_light_gray_;
};

}  // namespace mrg_slam


#endif  // MARKER_PUBLISHER_HPP