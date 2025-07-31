// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/edge_se3.h>
#include <tf2/exceptions.h>

#include <Eigen/Dense>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mrg_slam/imu_processor.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mrg_slam {

void
ImuProcessor::onInit( rclcpp::Node::SharedPtr node )
{
    node_ = node;

    tf2_buffer_   = std::make_unique<tf2_ros::Buffer>( node->get_clock() );
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf2_buffer_ );

    if( node_->get_parameter( "enable_imu_orientation" ).as_bool() || node_->get_parameter( "enable_imu_acceleration" ).as_bool() ) {
        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>( "imu/data", rclcpp::QoS( 1024 ),
                                                                      std::bind( &ImuProcessor::imu_callback, this,
                                                                                 std::placeholders::_1 ) );
    }
}


void
ImuProcessor::imu_callback( const sensor_msgs::msg::Imu::SharedPtr imu_msg )
{
    std::lock_guard<std::mutex> lock( imu_queue_mutex_ );
    imu_msg->header.stamp = ( rclcpp::Time( imu_msg->header.stamp )
                              + rclcpp::Duration::from_seconds( node_->get_parameter( "imu_time_offset" ).as_double() ) )
                                .operator builtin_interfaces::msg::Time();
    imu_queue_.push_back( imu_msg );
}


bool
ImuProcessor::flush( std::shared_ptr<GraphSLAM>& graph_slam, const std::vector<KeyFrame::Ptr>& keyframes, const std::string& base_frame_id )
{
    std::lock_guard<std::mutex> lock( imu_queue_mutex_ );
    if( keyframes.empty() || imu_queue_.empty() || base_frame_id.empty() ) {
        return false;
    }

    bool updated    = false;
    auto imu_cursor = imu_queue_.begin();

    // Get parameters before the loop
    bool        enable_imu_orientation                   = node_->get_parameter( "enable_imu_orientation" ).as_bool();
    bool        enable_imu_acceleration                  = node_->get_parameter( "enable_imu_acceleration" ).as_bool();
    double      imu_orientation_edge_stddev              = node_->get_parameter( "imu_orientation_edge_stddev" ).as_double();
    std::string imu_orientation_edge_robust_kernel       = node_->get_parameter( "imu_orientation_edge_robust_kernel" ).as_string();
    double      imu_orientation_edge_robust_kernel_size  = node_->get_parameter( "imu_orientation_edge_robust_kernel_size" ).as_double();
    double      imu_acceleration_edge_stddev             = node_->get_parameter( "imu_acceleration_edge_stddev" ).as_double();
    std::string imu_acceleration_edge_robust_kernel      = node_->get_parameter( "imu_acceleration_edge_robust_kernel" ).as_string();
    double      imu_acceleration_edge_robust_kernel_size = node_->get_parameter( "imu_acceleration_edge_robust_kernel_size" ).as_double();

    for( auto& keyframe : keyframes ) {
        if( rclcpp::Time( keyframe->stamp ) > rclcpp::Time( imu_queue_.back()->header.stamp ) ) {
            break;
        }

        if( rclcpp::Time( keyframe->stamp ) < rclcpp::Time( ( *imu_cursor )->header.stamp ) || keyframe->acceleration ) {
            continue;
        }

        // find imu data which is closest to the keyframe
        auto closest_imu = imu_cursor;
        for( auto imu = imu_cursor; imu != imu_queue_.end(); imu++ ) {
            auto dt  = ( rclcpp::Time( ( *closest_imu )->header.stamp ) - rclcpp::Time( keyframe->stamp ) ).seconds();
            auto dt2 = ( rclcpp::Time( ( *imu )->header.stamp ) - rclcpp::Time( keyframe->stamp ) ).seconds();
            if( std::abs( dt ) < std::abs( dt2 ) ) {
                break;
            }

            closest_imu = imu;
        }

        imu_cursor = closest_imu;
        if( 0.2 < std::abs( ( rclcpp::Time( ( *closest_imu )->header.stamp ) - rclcpp::Time( keyframe->stamp ) ).seconds() ) ) {
            continue;
        }

        geometry_msgs::msg::Vector3Stamped    acc_imu;
        geometry_msgs::msg::Vector3Stamped    acc_base;
        geometry_msgs::msg::QuaternionStamped quat_imu;
        geometry_msgs::msg::QuaternionStamped quat_base;

        quat_imu.header.frame_id = acc_imu.header.frame_id = ( *closest_imu )->header.frame_id;
        quat_imu.header.stamp = acc_imu.header.stamp = builtin_interfaces::msg::Time();
        acc_imu.vector                               = ( *closest_imu )->linear_acceleration;
        quat_imu.quaternion                          = ( *closest_imu )->orientation;

        geometry_msgs::msg::TransformStamped t;
        try {
            tf2_buffer_->transform<geometry_msgs::msg::Vector3Stamped>( acc_imu, acc_base, base_frame_id );
            tf2_buffer_->transform<geometry_msgs::msg::QuaternionStamped>( quat_imu, quat_base, base_frame_id );
        } catch( const tf2::TransformException& e ) {
            RCLCPP_INFO( node_->get_logger(), "Could not transform from %s to %s: %s", acc_imu.header.frame_id.c_str(),
                         base_frame_id.c_str(), e.what() );
            return false;
        }

        keyframe->acceleration = Eigen::Vector3d( acc_base.vector.x, acc_base.vector.y, acc_base.vector.z );
        keyframe->orientation  = Eigen::Quaterniond( quat_base.quaternion.w, quat_base.quaternion.x, quat_base.quaternion.y,
                                                     quat_base.quaternion.z );
        keyframe->orientation  = keyframe->orientation;
        if( keyframe->orientation->w() < 0.0 ) {
            keyframe->orientation->coeffs() = -keyframe->orientation->coeffs();
        }

        if( enable_imu_orientation ) {
            Eigen::MatrixXd info = Eigen::MatrixXd::Identity( 3, 3 ) / imu_orientation_edge_stddev;
            auto            edge = graph_slam->add_se3_prior_quat_edge( keyframe->node, *keyframe->orientation, info );
            graph_slam->add_robust_kernel( edge, imu_orientation_edge_robust_kernel, imu_orientation_edge_robust_kernel_size );
        }

        if( enable_imu_acceleration ) {
            Eigen::MatrixXd info = Eigen::MatrixXd::Identity( 3, 3 ) / imu_acceleration_edge_stddev;
            auto edge = graph_slam->add_se3_prior_vec_edge( keyframe->node, -Eigen::Vector3d::UnitZ(), *keyframe->acceleration, info );
            graph_slam->add_robust_kernel( edge, imu_acceleration_edge_robust_kernel, imu_acceleration_edge_robust_kernel_size );
        }
        updated = true;
    }

    auto remove_loc = std::upper_bound( imu_queue_.begin(), imu_queue_.end(), rclcpp::Time( keyframes.back()->stamp ),
                                        [=]( const rclcpp::Time& stamp, const sensor_msgs::msg::Imu::ConstSharedPtr imu ) {
                                            return stamp < rclcpp::Time( imu->header.stamp );
                                        } );
    imu_queue_.erase( imu_queue_.begin(), remove_loc );

    return updated;
}

}  // namespace mrg_slam