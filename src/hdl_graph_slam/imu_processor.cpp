// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/edge_se3.h>

#include <Eigen/Dense>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hdl_graph_slam/imu_processor.hpp>

using std::placeholders::_1;

namespace hdl_graph_slam {

// void
// ImuProcessor::onInit( ros::NodeHandle& nh, ros::NodeHandle& mt_nh, ros::NodeHandle& private_nh_ )
// {
//     private_nh = &private_nh_;

//     imu_time_offset              = private_nh->param<double>( "imu_time_offset", 0.0 );
//     enable_imu_orientation       = private_nh->param<bool>( "enable_imu_orientation", false );
//     enable_imu_acceleration      = private_nh->param<bool>( "enable_imu_acceleration", false );
//     imu_orientation_edge_stddev  = private_nh->param<double>( "imu_orientation_edge_stddev", 0.1 );
//     imu_acceleration_edge_stddev = private_nh->param<double>( "imu_acceleration_edge_stddev", 3.0 );

//     imu_sub = nh.subscribe( "/imu/data", 1024, &ImuProcessor::imu_callback, this );
// }
void
ImuProcessor::onInit( rclcpp::Node::SharedPtr& _node )
{
    node = _node;

    tf2_buffer   = std::make_unique<tf2_ros::Buffer>( node->get_clock() );
    tf2_listener = std::make_shared<tf2_ros::TransformListener>( *tf2_buffer );

    // TODO: ROS2 parameter handling
    // imu_time_offset              = private_nh->param<double>( "imu_time_offset", 0.0 );
    // enable_imu_orientation       = private_nh->param<bool>( "enable_imu_orientation", false );
    // enable_imu_acceleration      = private_nh->param<bool>( "enable_imu_acceleration", false );
    // imu_orientation_edge_stddev  = private_nh->param<double>( "imu_orientation_edge_stddev", 0.1 );
    // imu_acceleration_edge_stddev = private_nh->param<double>( "imu_acceleration_edge_stddev", 3.0 );

    imu_sub = node->create_subscription<sensor_msgs::msg::Imu>( "/imu/data", rclcpp::QoS( 1024 ),
                                                                std::bind( &ImuProcessor::imu_callback, this, _1 ) );
}


void
ImuProcessor::imu_callback( const sensor_msgs::msg::Imu::SharedPtr imu_msg )
{
    if( !enable_imu_orientation && !enable_imu_acceleration ) {
        return;
    }

    std::lock_guard<std::mutex> lock( imu_queue_mutex );
    // TODO: probably correct, needs verification
    imu_msg->header.stamp.sec += (int32_t)imu_time_offset;
    imu_msg->header.stamp.nanosec += (uint32_t)( ( imu_time_offset - (int)imu_time_offset ) * 1e9 );
    // TODO discuss which version is better
    // imu_msg->header.stamp = rclcpp::Time( imu_msg->header.stamp )
    //                         + rclcpp::Duration( static_cast<rcl_duration_value_t>( imu_time_offset * 1e9 ) );
    imu_queue.push_back( imu_msg );
}


bool
ImuProcessor::flush( std::shared_ptr<GraphSLAM>& graph_slam, const std::vector<KeyFrame::Ptr>& keyframes, const std::string& base_frame_id )
{
    std::lock_guard<std::mutex> lock( imu_queue_mutex );
    if( keyframes.empty() || imu_queue.empty() || base_frame_id.empty() ) {
        return false;
    }

    bool updated    = false;
    auto imu_cursor = imu_queue.begin();

    for( auto& keyframe : keyframes ) {
        if( rclcpp::Time( keyframe->stamp ) > rclcpp::Time( imu_queue.back()->header.stamp ) ) {
            break;
        }

        if( rclcpp::Time( keyframe->stamp ) < rclcpp::Time( ( *imu_cursor )->header.stamp ) || keyframe->acceleration ) {
            continue;
        }

        // find imu data which is closest to the keyframe
        auto closest_imu = imu_cursor;
        for( auto imu = imu_cursor; imu != imu_queue.end(); imu++ ) {
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

        const auto& imu_ori = ( *closest_imu )->orientation;
        const auto& imu_acc = ( *closest_imu )->linear_acceleration;

        geometry_msgs::msg::Vector3Stamped    acc_imu;
        geometry_msgs::msg::Vector3Stamped    acc_base;
        geometry_msgs::msg::QuaternionStamped quat_imu;
        geometry_msgs::msg::QuaternionStamped quat_base;

        quat_imu.header.frame_id = acc_imu.header.frame_id = ( *closest_imu )->header.frame_id;
        quat_imu.header.stamp = acc_imu.header.stamp = rclcpp::Time( 0 );
        acc_imu.vector                               = ( *closest_imu )->linear_acceleration;
        quat_imu.quaternion                          = ( *closest_imu )->orientation;

        // TODO: The commented code needs to be migrated to ROS2 and verified
        // try {
        //     transformVector( string target_frame, stamped_in, stamped_out );
        //     tf_listener->transformVector( base_frame_id, acc_imu, acc_base );
        //     tf_listener->transformQuaternion( base_frame_id, quat_imu, quat_base );
        // } catch( std::exception& e ) {
        //     std::cerr << "failed to find transform!!" << std::endl;
        //     return false;
        // }
        geometry_msgs::msg::TransformStamped t;
        try {
            // tf2_buffer->transform(const T &in, T &out, const std::string &target_frame, tf2::Duration timeout =
            // tf2::durationFromSec((0.0))) TODO: maybe add timeout? Verify this
            tf2_buffer->transform( acc_imu, acc_base, base_frame_id );
            tf2_buffer->transform( quat_imu, quat_base, base_frame_id );
        } catch( const tf2::TransformException& e ) {
            RCLCPP_INFO( node->get_logger(), "Could not transform from %s to %s: %s", acc_imu.header.frame_id.c_str(),
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
            // TODO: ROS2 parameter handling
            // graph_slam->add_robust_kernel( edge, private_nh->param<std::string>( "imu_orientation_edge_robust_kernel", "NONE" ),
            //                                private_nh->param<double>( "imu_orientation_edge_robust_kernel_size", 1.0 ) );
        }

        if( enable_imu_acceleration ) {
            Eigen::MatrixXd info = Eigen::MatrixXd::Identity( 3, 3 ) / imu_acceleration_edge_stddev;
            auto edge = graph_slam->add_se3_prior_vec_edge( keyframe->node, -Eigen::Vector3d::UnitZ(), *keyframe->acceleration, info );
            // TODO: ROS2 parameter handling
            // graph_slam->add_robust_kernel( edge, private_nh->param<std::string>( "imu_acceleration_edge_robust_kernel", "NONE" ),
            //                                private_nh->param<double>( "imu_acceleration_edge_robust_kernel_size", 1.0 ) );
        }
        updated = true;
    }

    // TODO: verify this lambda expression
    auto remove_loc = std::upper_bound( imu_queue.begin(), imu_queue.end(), rclcpp::Time( keyframes.back()->stamp ),
                                        [=]( const rclcpp::Time& stamp, const sensor_msgs::msg::Imu::ConstPtr imu ) {
                                            return stamp < rclcpp::Time( imu->header.stamp );
                                        } );
    imu_queue.erase( imu_queue.begin(), remove_loc );

    return updated;
}

}  // namespace hdl_graph_slam