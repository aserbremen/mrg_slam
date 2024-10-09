#include <mrg_slam/ros_utils.hpp>

namespace mrg_slam {

geometry_msgs::msg::TransformStamped
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
    odom_trans.header.stamp    = stamp.operator builtin_interfaces::msg::Time();
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id  = child_frame_id;

    odom_trans.transform.translation.x = pose( 0, 3 );
    odom_trans.transform.translation.y = pose( 1, 3 );
    odom_trans.transform.translation.z = pose( 2, 3 );
    odom_trans.transform.rotation      = odom_quat;

    return odom_trans;
}


Eigen::Isometry3d
pose2isometry( const geometry_msgs::msg::Pose& pose )
{
    Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
    mat.translation()     = Eigen::Vector3d( pose.position.x, pose.position.y, pose.position.z );
    mat.linear() = Eigen::Quaterniond( pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z ).toRotationMatrix();
    return mat;
}


Eigen::Isometry3d
tf2isometry( const geometry_msgs::msg::TransformStamped& trans )
{
    Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
    mat.translation()     = Eigen::Vector3d( trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z );
    mat.linear()          = Eigen::Quaterniond( trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y,
                                                trans.transform.rotation.z )
                       .toRotationMatrix();
    return mat;
}

geometry_msgs::msg::Pose
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

Eigen::Isometry3d
odom2isometry( const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg )
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

/**
 * @brief Prints ROS2 parameters for a given node.
 * @param param_interface
 * @param logger
 */
void
print_ros2_parameters( rclcpp::node_interfaces::NodeParametersInterface::ConstSharedPtr param_interface, const rclcpp::Logger& logger )
{
    const auto& list_params = param_interface->list_parameters( std::vector<std::string>{}, 0 );
    const auto& params_vec  = param_interface->get_parameters( list_params.names );
    for( const auto& param : params_vec ) {
        RCLCPP_INFO_STREAM( logger,
                            std::left << std::setw( 28 ) << std::setfill( ' ' ) << param.get_name() << " " << param.value_to_string() );
    }
}


}  // namespace mrg_slam
