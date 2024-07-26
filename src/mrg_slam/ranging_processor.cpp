#include <mrg_slam/ranging_processor.hpp>

namespace mrg_slam {

RangingProcessor::RangingProcessor() {}

void
RangingProcessor::onInit( rclcpp::Node::SharedPtr _node )
{
    node = _node;

    own_name = node->get_parameter( "own_name" ).as_string();

    ranging_sub_topic       = node->declare_parameter( "ranging_sub_topic", "/ranging_measurement" );
    ranging_robot_sub_topic = node->declare_parameter( "ranging_robot_sub_topic", "/odom_ground_truth" );
    multi_robot_names       = node->get_parameter( "multi_robot_names" ).as_string_array();
    ranging_names           = node->declare_parameter( "ranging_names", std::vector<std::string>() );

    std::string prefix = own_name.empty() ? "" : "/" + own_name;
    ranging_sub_topic  = prefix + ranging_sub_topic;

    ranging_sub = node->create_subscription<ros2_radio_ranging_interfaces::msg::Range>( ranging_sub_topic, rclcpp::QoS( 10 ),
                                                                                        std::bind( &RangingProcessor::ranging_callback,
                                                                                                   this, std::placeholders::_1 ) );

    for( const auto &name : multi_robot_names ) {
        std::string position_prefix = name.empty() ? "" : "/" + name;
        std::string position_topic  = position_prefix + ranging_robot_sub_topic;
        position_subs.push_back( node->create_subscription<nav_msgs::msg::Odometry>( position_topic, rclcpp::QoS( 10 ),
                                                                                     std::bind( &RangingProcessor::position_callback, this,
                                                                                                std::placeholders::_1 ) ) );
    }

    for( const auto &name : ranging_names ) {
        std::string position_prefix = name.empty() ? "" : "/" + name;
        std::string position_topic  = position_prefix + "/gnss_rtk_rel_nav";
        position_subs.push_back( node->create_subscription<nav_msgs::msg::Odometry>( position_topic, rclcpp::QoS( 10 ),
                                                                                     std::bind( &RangingProcessor::position_callback, this,
                                                                                                std::placeholders::_1 ) ) );
    }
}

void
RangingProcessor::position_callback( const nav_msgs::msg::Odometry::SharedPtr position_msg )
{
    for( const auto &name : ranging_names ) {
        if( position_msg->child_frame_id.find( name ) != std::string::npos ) {
            others_position_map[name].push_back( position_msg );
        }
    }
}

void
RangingProcessor::ranging_callback( ros2_radio_ranging_interfaces::msg::Range::ConstSharedPtr ranging_msg )
{
    std::lock_guard<std::mutex> lock( ranging_queue_mutex );
    ranging_queue.push_back( ranging_msg );
}

bool
RangingProcessor::flush( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes )
{
    std::lock_guard<std::mutex> lock( ranging_queue_mutex );
    if( ranging_queue.empty() || keyframes.empty() ) {
        return false;
    }

    init_positions( graph_slam );

    for( const auto &ranging_msg : ranging_queue ) {
        for( const auto &keyframe : keyframes ) {
            if( std::abs( ( rclcpp::Time( ranging_msg->header.stamp ) - rclcpp::Time( keyframe->stamp ) ).seconds() ) < 0.1 ) {
                RCLCPP_INFO_STREAM( node->get_logger(), "Ranging message from " << ranging_msg->self_name << " to "
                                                                                << ranging_msg->neighbor_name << " close to "
                                                                                << keyframe->readable_id );
            }
        }
    }


    return false;  // TODO
}


void
RangingProcessor::init_positions( std::shared_ptr<GraphSLAM> &graph_slam )
{
    for( const auto &name : ranging_names ) {
        if( others_position_map.find( name ) == others_position_map.end() ) {
            RCLCPP_WARN_STREAM( node->get_logger(), "No position information for " << name );
            continue;
        }

        auto &position_deque = others_position_map[name];
        if( position_deque.empty() ) {
            RCLCPP_WARN_STREAM( node->get_logger(), "No position information for " << name );
            continue;
        }

        // TODO: for now, only use the latest position, for stationary boxes this is fine, for other moving robots we need to choose the
        // position corresponding to the ranging message that will be added to the graph
        auto position_msg = position_deque.back();
        position_deque.clear();

        Eigen::Isometry3d  pose = Eigen::Isometry3d::Identity();
        Eigen::Vector3d    translation( position_msg->pose.pose.position.x, position_msg->pose.pose.position.y,
                                        position_msg->pose.pose.position.z );
        Eigen::Quaterniond quat( position_msg->pose.pose.orientation.w, position_msg->pose.pose.orientation.x,
                                 position_msg->pose.pose.orientation.y, position_msg->pose.pose.orientation.z );

        g2o::VertexSE3 *g2o_vertex = graph_slam->add_se3_node( pose );
        // TODO set stddev from parameter or from the message
        graph_slam->add_se3_prior_xyz_edge( g2o_vertex, translation, Eigen::Matrix3d::Identity() * 0.05 );
        graph_slam->add_se3_prior_quat_edge( g2o_vertex, quat, Eigen::Matrix3d::Identity() * 0.05 );
    }
}

}  // namespace mrg_slam
