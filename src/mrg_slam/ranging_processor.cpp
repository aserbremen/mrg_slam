#include <mrg_slam/ranging_processor.hpp>

namespace mrg_slam {

RangingProcessor::RangingProcessor() {}

void
RangingProcessor::onInit( rclcpp::Node::SharedPtr _node )
{
    node = _node;

    own_name = node->get_parameter( "own_name" ).as_string();

    enable_ranging                  = node->declare_parameter( "enable_ranging", false );
    ranging_topic                   = node->declare_parameter( "ranging_topic", "/ranging_measurement" );
    ranging_position_topic          = node->declare_parameter( "ranging_position_topic", "/gnss_rtk_rel_nav" );
    ranging_names                   = node->declare_parameter( "ranging_names", std::vector<std::string>() );
    ranging_position_stddev         = node->declare_parameter( "ranging_position_stddev", 0.05 );
    ranging_orientation_stddev      = node->declare_parameter( "ranging_orientation_stddev", 0.1 );
    ranging_max_time_diff           = node->declare_parameter( "ranging_max_time_diff", 0.05 );
    ranging_edge_robust_kernel      = node->declare_parameter( "ranging_edge_robust_kernel", "NONE" );
    ranging_edge_robust_kernel_size = node->declare_parameter( "ranging_edge_robust_kernel_size", 1.0 );


    if( enable_ranging ) {
        std::string prefix = own_name.empty() ? "" : "/" + own_name;
        ranging_topic      = prefix + ranging_topic;
        RCLCPP_INFO_STREAM( node->get_logger(), "Subscribing to ranging topic: " << ranging_topic );
        ranging_sub = node->create_subscription<ros2_radio_ranging_interfaces::msg::Range>( ranging_topic, rclcpp::QoS( 10 ),
                                                                                            std::bind( &RangingProcessor::ranging_callback,
                                                                                                       this, std::placeholders::_1 ) );

        for( const auto &name : ranging_names ) {
            std::string prefix = name.empty() ? "" : "/" + name;
            std::string topic  = prefix + ranging_position_topic;
            ranging_data_map.insert( { name, RangingData() } );
            ranging_data_map[name].position_sub = node->create_subscription<nav_msgs::msg::Odometry>(
                topic, rclcpp::QoS( 10 ), std::bind( &RangingProcessor::position_callback, this, std::placeholders::_1 ) );
        }
    }
}

void
RangingProcessor::position_callback( nav_msgs::msg::Odometry::ConstSharedPtr position_msg )
{
    for( const auto &name : ranging_names ) {
        if( position_msg->child_frame_id.find( name ) != std::string::npos && !ranging_data_map[name].initialized ) {
            ranging_data_map[name].position_deque.push_back( position_msg );
        }
    }
}

void
RangingProcessor::ranging_callback( ros2_radio_ranging_interfaces::msg::Range::ConstSharedPtr range_msg )
{
    std::lock_guard<std::mutex> lock( range_queue_mutex );
    range_queue.push_back( range_msg );
}

bool
RangingProcessor::flush( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes )
{
    if( !enable_ranging ) {
        return false;
    }
    std::lock_guard<std::mutex> lock( range_queue_mutex );
    if( range_queue.empty() || keyframes.empty() ) {
        RCLCPP_INFO_STREAM( node->get_logger(), "No ranging messages or keyframes" );
        return false;
    }

    init_positions( graph_slam );

    bool added_edge = false;

    for( const auto &range_msg : range_queue ) {
        if( ranging_data_map.find( range_msg->neighbor_name ) == ranging_data_map.end()
            || !ranging_data_map[range_msg->neighbor_name].initialized ) {
            RCLCPP_WARN_STREAM( node->get_logger(), "No information for " << range_msg->neighbor_name << ", cannot add ranging edge" );
            continue;
        }
        KeyFrame::Ptr closest_keyframe = find_closest_keyframe( range_msg, keyframes );
        if( closest_keyframe != nullptr ) {
            add_ranging_edge( graph_slam, range_msg, closest_keyframe );
            added_edge = true;
        }
    }

    if( added_edge ) {
        range_queue.clear();
    }

    return added_edge;
}


void
RangingProcessor::init_positions( std::shared_ptr<GraphSLAM> &graph_slam )
{
    for( const auto &name : ranging_names ) {
        if( ranging_data_map[name].initialized ) {
            RCLCPP_INFO_STREAM_THROTTLE( node->get_logger(), *node->get_clock(), 5000, "Position already initialized for " << name );
            continue;
        }

        auto &position_deque = ranging_data_map[name].position_deque;
        if( position_deque.empty() ) {
            RCLCPP_WARN_STREAM( node->get_logger(), "No position msgs for " << name );
            continue;
        }

        // TODO: for now, only use the latest position, for stationary boxes this is fine, for other moving robots we need to choose the
        // position corresponding to the ranging message that will be added to the graph
        // Update the position of the ranging device and prior edge
        auto position_msg = position_deque.back();

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation()     = Eigen::Vector3d( position_msg->pose.pose.position.x, position_msg->pose.pose.position.y,
                                                  position_msg->pose.pose.position.z );
        Eigen::Quaterniond quat( position_msg->pose.pose.orientation.w, position_msg->pose.pose.orientation.x,
                                 position_msg->pose.pose.orientation.y, position_msg->pose.pose.orientation.z );
        pose.linear() = quat.toRotationMatrix();

        RCLCPP_INFO_STREAM( node->get_logger(), "Adding se3 node for " << name << " at " << pose.translation().transpose() );
        g2o::VertexSE3 *g2o_vertex        = graph_slam->add_se3_node( pose );
        ranging_data_map[name].se3_vertex = g2o_vertex;
        // TODO set stddev from parameter or from the message
        double position_information = ( 1 / ranging_position_stddev ) * ( 1 / ranging_position_stddev );
        graph_slam->add_se3_prior_xyz_edge( g2o_vertex, pose.translation(), position_information * Eigen::Matrix3d::Identity() );
        double orientation_information = ( 1 / ranging_orientation_stddev ) * ( 1 / ranging_orientation_stddev );
        graph_slam->add_se3_prior_quat_edge( g2o_vertex, quat, orientation_information * Eigen::Matrix3d::Identity() );
        ranging_data_map[name].initialized = true;

        position_deque.clear();
    }
}

KeyFrame::Ptr
RangingProcessor::find_closest_keyframe( ros2_radio_ranging_interfaces::msg::Range::ConstSharedPtr range_msg,
                                         const std::vector<KeyFrame::Ptr>                         &keyframes )
{
    KeyFrame::Ptr closest_keyframe = nullptr;
    double        min_time_diff    = std::numeric_limits<double>::max();

    for( const auto &keyframe : keyframes ) {
        double time_diff = std::abs( ( rclcpp::Time( range_msg->header.stamp ) - rclcpp::Time( keyframe->stamp ) ).seconds() );
        if( time_diff < min_time_diff && time_diff < ranging_max_time_diff ) {
            min_time_diff    = time_diff;
            closest_keyframe = keyframe;
        }
    }

    return closest_keyframe;
}

void
RangingProcessor::add_ranging_edge( std::shared_ptr<GraphSLAM>                               &graph_slam,
                                    ros2_radio_ranging_interfaces::msg::Range::ConstSharedPtr range_msg, KeyFrame::Ptr keyframe )
{
    RCLCPP_INFO_STREAM( node->get_logger(), "Ranging message from " << range_msg->self_name << " to " << range_msg->neighbor_name
                                                                    << " close to " << keyframe->readable_id );
    double          range      = range_msg->bias_compensation_valid ? range_msg->range_compensated : range_msg->range_raw;
    Eigen::MatrixXd inf_matrix = Eigen::MatrixXd::Identity( 1, 1 );
    inf_matrix( 0, 0 ) = range_msg->bias_compensation_valid ? ( 1 / range_msg->var_range_compensated ) : ( 1 / range_msg->var_range_raw );
    RCLCPP_INFO_STREAM( node->get_logger(), "Adding ranging edge from " << range_msg->self_name << " to " << range_msg->neighbor_name
                                                                        << " with range " << range << " and inf " << inf_matrix.value() );
    g2o::EdgeSE3Ranging *g2o_ranging_edge = graph_slam->add_se3_ranging_edge( keyframe->node,
                                                                              ranging_data_map[range_msg->neighbor_name].se3_vertex, range,
                                                                              inf_matrix );
    ranging_data_map[range_msg->neighbor_name].edges_se3_ranging.push_back( g2o_ranging_edge );

    graph_slam->add_robust_kernel( static_cast<g2o::HyperGraph::Edge *>( g2o_ranging_edge ), ranging_edge_robust_kernel,
                                   ranging_edge_robust_kernel_size );
}

}  // namespace mrg_slam
