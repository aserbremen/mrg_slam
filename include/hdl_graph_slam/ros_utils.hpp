// SPDX-License-Identifier: BSD-2-Clause

#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <ros/ros.h>

#include <tf2/transform_datatypes.h>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hdl_graph_slam {

/**
 * @brief convert Eigen::Matrix to geometry_msgs::TransformStamped
 * @param stamp            timestamp
 * @param pose             Eigen::Matrix to be converted
 * @param frame_id         tf frame_id
 * @param child_frame_id   tf child frame_id
 * @return converted TransformStamped
 */
// static geometry_msgs::TransformStamped
// matrix2transform( const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id )
// {
//     Eigen::Quaternionf quat( pose.block<3, 3>( 0, 0 ) );
//     quat.normalize();
//     geometry_msgs::Quaternion odom_quat;
//     odom_quat.w = quat.w();
//     odom_quat.x = quat.x();
//     odom_quat.y = quat.y();
//     odom_quat.z = quat.z();

//     geometry_msgs::TransformStamped odom_trans;
//     odom_trans.header.stamp    = stamp;
//     odom_trans.header.frame_id = frame_id;
//     odom_trans.child_frame_id  = child_frame_id;

//     odom_trans.transform.translation.x = pose( 0, 3 );
//     odom_trans.transform.translation.y = pose( 1, 3 );
//     odom_trans.transform.translation.z = pose( 2, 3 );
//     odom_trans.transform.rotation      = odom_quat;

//     return odom_trans;
// }
static geometry_msgs::msg::TransformStamped
matrix2transform( const rclcpp::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id )
{
    Eigen::Quaternionf quat( pose.block<3, 3>( 0, 0 ) );
    quat.normalize();
    geometry_msgs::msg::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();

    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp    = stamp.     operator builtin_interfaces::msg::Time();
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id  = child_frame_id;

    odom_trans.transform.translation.x = pose( 0, 3 );
    odom_trans.transform.translation.y = pose( 1, 3 );
    odom_trans.transform.translation.z = pose( 2, 3 );
    odom_trans.transform.rotation      = odom_quat;

    return odom_trans;
}


// static Eigen::Isometry3d
// pose2isometry( const geometry_msgs::Pose& pose )
// {
//     Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
//     mat.translation()     = Eigen::Vector3d( pose.position.x, pose.position.y, pose.position.z );
//     mat.linear() = Eigen::Quaterniond( pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z
//     ).toRotationMatrix(); return mat;
// }
static Eigen::Isometry3d
pose2isometry( const geometry_msgs::msg::Pose& pose )
{
    Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
    mat.translation()     = Eigen::Vector3d( pose.position.x, pose.position.y, pose.position.z );
    mat.linear() = Eigen::Quaterniond( pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z ).toRotationMatrix();
    return mat;
}


// static Eigen::Isometry3d
// tf2isometry( const tf::StampedTransform& trans )
// {
//     Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
//     mat.translation()     = Eigen::Vector3d( trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z() );
//     mat.linear() = Eigen::Quaterniond( trans.getRotation().w(), trans.getRotation().x(), trans.getRotation().y(), trans.getRotation().z()
//     )
//                        .toRotationMatrix();
//     return mat;
// }
static Eigen::Isometry3d
tf2isometry( const geometry_msgs::msg::TransformStamped& trans )
{
    Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
    mat.translation()     = Eigen::Vector3d( trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z );
    mat.linear()          = Eigen::Quaterniond( trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y,
                                                trans.transform.rotation.z )
                       .toRotationMatrix();
    return mat;
}

// static geometry_msgs::Pose
// isometry2pose( const Eigen::Isometry3d& mat )
// {
//     Eigen::Quaterniond quat( mat.linear() );
//     Eigen::Vector3d    trans = mat.translation();

//     geometry_msgs::Pose pose;
//     pose.position.x    = trans.x();
//     pose.position.y    = trans.y();
//     pose.position.z    = trans.z();
//     pose.orientation.w = quat.w();
//     pose.orientation.x = quat.x();
//     pose.orientation.y = quat.y();
//     pose.orientation.z = quat.z();

//     return pose;
// }
static geometry_msgs::msg::Pose
isometry2pose( const Eigen::Isometry3d& mat )
{
    Eigen::Quaterniond quat( mat.linear() );
    Eigen::Vector3d    trans = mat.translation();

    geometry_msgs::msg::Pose pose;
    pose.position.x    = trans.x();
    pose.position.y    = trans.y();
    pose.position.z    = trans.z();
    pose.orientation.w = quat.w();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();

    return pose;
}

// static Eigen::Isometry3d
// odom2isometry( const nav_msgs::OdometryConstPtr& odom_msg )
// {
//     const auto& orientation = odom_msg->pose.pose.orientation;
//     const auto& position    = odom_msg->pose.pose.position;

//     Eigen::Quaterniond quat;
//     quat.w() = orientation.w;
//     quat.x() = orientation.x;
//     quat.y() = orientation.y;
//     quat.z() = orientation.z;

//     Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
//     isometry.linear()          = quat.toRotationMatrix();
//     isometry.translation()     = Eigen::Vector3d( position.x, position.y, position.z );
//     return isometry;
// }
static Eigen::Isometry3d
odom2isometry( const nav_msgs::msg::Odometry::ConstPtr& odom_msg )
{
    const auto& orientation = odom_msg->pose.pose.orientation;
    const auto& position    = odom_msg->pose.pose.position;

    Eigen::Quaterniond quat;
    quat.w() = orientation.w;
    quat.x() = orientation.x;
    quat.y() = orientation.y;
    quat.z() = orientation.z;

    Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
    isometry.linear()          = quat.toRotationMatrix();
    isometry.translation()     = Eigen::Vector3d( position.x, position.y, position.z );
    return isometry;
}

void
print_ros2_parameters( const std::vector<rclcpp::Parameter>& ros_params, const rclcpp::Logger& logger )
{
    // TODO  Simplify the output of the parameters using value_to_string() method, switch case is not needed probably
    for( size_t i = 0; i < ros_params.size(); i++ ) {
        const auto& param = ros_params[i];
        std::string param_array_string;
        switch( param.get_type() ) {
            case rclcpp::ParameterType::PARAMETER_NOT_SET:
                RCLCPP_INFO_STREAM( logger, param.get_name() << " NOT_SET" );
                break;
            case rclcpp::ParameterType::PARAMETER_BOOL:
                RCLCPP_INFO_STREAM( logger, param.get_name() << ( param.as_bool() ? " true" : " false" ) );
                break;
            case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
                RCLCPP_INFO_STREAM( logger, param.get_name() << " " << param.as_int() );
                break;
            case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
                RCLCPP_INFO_STREAM( logger, param.get_name() << " " << param.as_double() );
                break;
            case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
                RCLCPP_INFO_STREAM( logger, param.get_name() << " " << param.as_string() );
                break;
            case rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY:
                for( const auto& byte : param.as_byte_array() ) {
                    param_array_string += byte + " ";
                }
                RCLCPP_INFO_STREAM( logger, param.get_name() << param_array_string );
                break;
            case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY:
                for( const auto& b : param.as_bool_array() ) {
                    param_array_string += b ? " true" : " false";
                }
                RCLCPP_INFO_STREAM( logger, param.get_name() << param_array_string );
                break;
            case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY:
                for( const auto& i : param.as_integer_array() ) {
                    param_array_string += " " + std::to_string( i );
                }
                RCLCPP_INFO_STREAM( logger, param.get_name() << param_array_string );
                break;
            case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY:
                for( const auto& d : param.as_double_array() ) {
                    param_array_string += " " + std::to_string( d );
                }
                RCLCPP_INFO_STREAM( logger, param.get_name() << param_array_string );
                break;
            case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:
                for( const auto& s : param.as_string_array() ) {
                    param_array_string += " " + s;
                }
                RCLCPP_INFO_STREAM( logger, param.get_name() << param_array_string );
                break;


            default:
                RCLCPP_WARN_STREAM( logger, "Cannot identiy parameter type for " << param.get_name() );
                break;
        }
    }
}


}  // namespace hdl_graph_slam

#endif  // ROS_UTILS_HPP
