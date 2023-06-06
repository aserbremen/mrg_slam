// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/edge_se3_plane.hpp>
#include <hdl_graph_slam/floor_coeffs_processor.hpp>


namespace hdl_graph_slam {

// void
// FloorCoeffsProcessor::onInit( ros::NodeHandle &nh, ros::NodeHandle &mt_nh, ros::NodeHandle &private_nh_ )
// {
//     private_nh           = &private_nh_;
//     floor_plane_node_ptr = nullptr;

//     floor_edge_stddev = private_nh->param<double>( "floor_edge_stddev", 10.0 );
//     floor_sub         = nh.subscribe( "/floor_coeffs", 1024, &FloorCoeffsProcessor::floor_coeffs_callback, this );
// }

void
FloorCoeffsProcessor::onInit( rclcpp::Node::SharedPtr _node )
{
    node                 = _node;
    floor_plane_node_ptr = nullptr;

    // TODO: ROS2 parameter handling, verify
    floor_edge_stddev             = node->declare_parameter<double>( "floor_edge_stddev", 10.0 );
    floor_edge_robust_kernel      = node->declare_parameter<std::string>( "floor_edge_robust_kernel", "NONE" );
    floor_edge_robust_kernel_size = node->declare_parameter<double>( "floor_edge_robust_kernel_size", 1.0 );

    floor_sub = node->create_subscription<vamex_slam_msgs::msg::FloorCoeffs>( "/floor_coeffs", rclcpp::QoS( 1024 ),
                                                                             std::bind( &FloorCoeffsProcessor::floor_coeffs_callback, this,
                                                                                        std::placeholders::_1 ) );
}

/**
 * @brief received floor coefficients are added to #floor_coeffs_queue
 * @param floor_coeffs_msg
 */
void
FloorCoeffsProcessor::floor_coeffs_callback( vamex_slam_msgs::msg::FloorCoeffs::ConstSharedPtr floor_coeffs_msg )
{
    if( floor_coeffs_msg->coeffs.empty() ) {
        return;
    }

    std::lock_guard<std::mutex> lock( floor_coeffs_queue_mutex );
    floor_coeffs_queue.push_back( floor_coeffs_msg );
}


/**
 * @brief this methods associates floor coefficients messages with registered keyframes, and then adds the associated coeffs to the pose
 * graph
 * @return if true, at least one floor plane edge is added to the pose graph
 */
bool
// FloorCoeffsProcessor::flush( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes,
//                              const std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> &keyframe_hash,
//                              const ros::Time                                                 &latest_keyframe_stamp )
FloorCoeffsProcessor::flush( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes,
                             const std::unordered_map<builtin_interfaces::msg::Time, KeyFrame::Ptr, RosTimeHash> &keyframe_hash,
                             const builtin_interfaces::msg::Time                                                 &latest_keyframe_stamp )
{
    std::lock_guard<std::mutex> lock( floor_coeffs_queue_mutex );

    if( keyframes.empty() ) {
        return false;
    }

    bool updated = false;
    for( const auto &floor_coeffs : floor_coeffs_queue ) {
        if( rclcpp::Time( floor_coeffs->header.stamp ) > rclcpp::Time( latest_keyframe_stamp ) ) {
            break;
        }

        auto found = keyframe_hash.find( floor_coeffs->header.stamp );
        if( found == keyframe_hash.end() ) {
            continue;
        }

        if( !floor_plane_node_ptr ) {
            floor_plane_node_ptr = graph_slam->add_plane_node( Eigen::Vector4d( 0.0, 0.0, 1.0, 0.0 ) );
            floor_plane_node_ptr->setFixed( true );
        }

        const auto &keyframe = found->second;

        Eigen::Vector4d coeffs( floor_coeffs->coeffs[0], floor_coeffs->coeffs[1], floor_coeffs->coeffs[2], floor_coeffs->coeffs[3] );
        Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * ( 1.0 / floor_edge_stddev );
        auto            edge        = graph_slam->add_se3_plane_edge( keyframe->node, floor_plane_node_ptr, coeffs, information );
        // graph_slam->add_robust_kernel( edge, private_nh->param<std::string>("floor_edge_robust_kernel", "NONE" ),
        //                                private_nh->param<double>( "floor_edge_robust_kernel_size", 1.0 ) );
        graph_slam->add_robust_kernel( edge, floor_edge_robust_kernel, floor_edge_robust_kernel_size );

        keyframe->floor_coeffs = coeffs;

        updated = true;
    }

    // TODO: verify
    // auto remove_loc = std::upper_bound( floor_coeffs_queue.begin(), floor_coeffs_queue.end(), latest_keyframe_stamp,
    //                                     [=]( const ros::Time &stamp, const hdl_graph_slam::FloorCoeffsConstPtr &coeffs ) {
    //                                         return stamp < coeffs->header.stamp;
    //                                     } );
    auto remove_loc = std::upper_bound( floor_coeffs_queue.begin(), floor_coeffs_queue.end(), rclcpp::Time( latest_keyframe_stamp ),
                                        [=]( const rclcpp::Time &stamp, const vamex_slam_msgs::msg::FloorCoeffs::ConstSharedPtr &coeffs ) {
                                            return stamp < rclcpp::Time( coeffs->header.stamp );
                                        } );
    floor_coeffs_queue.erase( floor_coeffs_queue.begin(), remove_loc );

    return updated;
}

}  // namespace hdl_graph_slam