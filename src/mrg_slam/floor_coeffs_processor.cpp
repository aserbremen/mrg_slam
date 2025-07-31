// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/edge_se3_plane.hpp>
#include <mrg_slam/floor_coeffs_processor.hpp>


namespace mrg_slam {

void
FloorCoeffsProcessor::onInit( rclcpp::Node::SharedPtr node )
{
    node_ = node;

    floor_plane_node_ptr_ = nullptr;

    if( node_->get_parameter( "enable_floor_coeffs" ).as_bool() ) {
        floor_sub_ = node_->create_subscription<mrg_slam_msgs::msg::FloorCoeffs>( "floor_coeffs", rclcpp::QoS( 1024 ),
                                                                                  std::bind( &FloorCoeffsProcessor::floor_coeffs_callback,
                                                                                             this, std::placeholders::_1 ) );
    }
}


void
FloorCoeffsProcessor::floor_coeffs_callback( mrg_slam_msgs::msg::FloorCoeffs::ConstSharedPtr floor_coeffs_msg )
{
    if( floor_coeffs_msg->coeffs.empty() ) {
        return;
    }

    std::lock_guard<std::mutex> lock( floor_coeffs_queue_mutex_ );
    floor_coeffs_queue_.push_back( floor_coeffs_msg );
}


bool
FloorCoeffsProcessor::flush( std::shared_ptr<GraphDatabase> graph_db, std::shared_ptr<GraphSLAM> &graph_slam )
{
    if( !node_->get_parameter( "enable_floor_coeffs" ).as_bool() ) {
        return false;
    }

    std::lock_guard<std::mutex> lock( floor_coeffs_queue_mutex_ );

    if( graph_db->get_keyframes().empty() ) {
        return false;
    }

    builtin_interfaces::msg::Time latest_keyframe_stamp = graph_db->get_prev_robot_keyframe()->stamp;
    auto                          keyframe_hash         = graph_db->get_keyframe_hash();

    // Get parameters before the loop
    double      floor_edge_stddev             = node_->get_parameter( "floor_edge_stddev" ).as_double();
    std::string floor_edge_robust_kernel      = node_->get_parameter( "floor_edge_robust_kernel" ).as_string();
    double      floor_edge_robust_kernel_size = node_->get_parameter( "floor_edge_robust_kernel_size" ).as_double();

    bool updated = false;
    for( const auto &floor_coeffs : floor_coeffs_queue_ ) {
        if( rclcpp::Time( floor_coeffs->header.stamp ) > rclcpp::Time( latest_keyframe_stamp ) ) {
            break;
        }

        auto found = keyframe_hash.find( floor_coeffs->header.stamp );
        if( found == keyframe_hash.end() ) {
            continue;
        }

        if( !floor_plane_node_ptr_ ) {
            floor_plane_node_ptr_ = graph_slam->add_plane_node( Eigen::Vector4d( 0.0, 0.0, 1.0, 0.0 ) );
            floor_plane_node_ptr_->setFixed( true );
        }

        const auto &keyframe = found->second;

        Eigen::Vector4d coeffs( floor_coeffs->coeffs[0], floor_coeffs->coeffs[1], floor_coeffs->coeffs[2], floor_coeffs->coeffs[3] );
        Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * ( 1.0 / floor_edge_stddev );
        auto            edge        = graph_slam->add_se3_plane_edge( keyframe->node, floor_plane_node_ptr_, coeffs, information );
        graph_slam->add_robust_kernel( edge, floor_edge_robust_kernel, floor_edge_robust_kernel_size );

        keyframe->floor_coeffs = coeffs;

        updated = true;
    }

    auto remove_loc = std::upper_bound( floor_coeffs_queue_.begin(), floor_coeffs_queue_.end(), rclcpp::Time( latest_keyframe_stamp ),
                                        [=]( const rclcpp::Time &stamp, const mrg_slam_msgs::msg::FloorCoeffs::ConstSharedPtr &coeffs ) {
                                            return stamp < rclcpp::Time( coeffs->header.stamp );
                                        } );
    floor_coeffs_queue_.erase( floor_coeffs_queue_.begin(), remove_loc );

    return updated;
}

}  // namespace mrg_slam