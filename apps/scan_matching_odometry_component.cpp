// SPDX-License-Identifier: BSD-2-Clause

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hdl_graph_slam/msg/scan_matching_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <ros/ros.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <hdl_graph_slam/ScanMatchingStatus.h>
// #include <nodelet/nodelet.h>
// #include <pcl_ros/point_cloud.h>
// #include <ros/duration.h>
// #include <ros/time.h>
// #include <std_msgs/Time.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>
// #include <tf_conversions/tf_eigen.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
// #include <pluginlib/class_list_macros.h>

#include <hdl_graph_slam/registrations.hpp>
#include <hdl_graph_slam/ros_utils.hpp>
#include <iostream>
#include <memory>

namespace hdl_graph_slam {

// class ScanMatchingOdometryComponent : public nodelet::Nodelet {
class ScanMatchingOdometryComponent : public rclcpp::Node {
public:
    typedef pcl::PointXYZI PointT;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ScanMatchingOdometryComponent() {}
    ScanMatchingOdometryComponent( const rclcpp::NodeOptions& options ) : Node( "scan_matching_odometry_component", options ) {}
    virtual ~ScanMatchingOdometryComponent() {}

    virtual void onInit()
    {
        // NODELET_DEBUG( "initializing scan_matching_odometry_nodelet..." );
        RCLCPP_INFO( this->get_logger(), "Initializing scan_matching_odometry_component..." );
        // This class is the node handle as it is derived from rclcpp::Node
        // nh         = getNodeHandle();
        // private_nh = getPrivateNodeHandle();

        initialize_params();

        // if( private_nh.param<bool>( "enable_imu_frontend", false ) ) {
        //     msf_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        //         "/msf_core/pose", 1, boost::bind( &ScanMatchingOdometryComponent::msf_pose_callback, this, _1, false ) );
        //     msf_pose_after_update_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        //         "/msf_core/pose_after_update", 1, boost::bind( &ScanMatchingOdometryComponent::msf_pose_callback, this, _1, true ) );
        // }
        if( this->declare_parameter<bool>( "enable_imu_frontend", false ) ) {
            // msf_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            //     "/msf_core/pose", 1, boost::bind( &ScanMatchingOdometryComponent::msf_pose_callback, this, _1, false ) );
            // msf_pose_after_update_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            //     "/msf_core/pose_after_update", 1, boost::bind( &ScanMatchingOdometryComponent::msf_pose_callback, this, _1, true ) );

            // We need to define a special function to pass arguments to a ROS2 callback with multiple parameters
            // https://answers.ros.org/question/308386/ros2-add-arguments-to-callback/ TODO: verify
            std::function<void( const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg )> fcn_false =
                std::bind( &ScanMatchingOdometryComponent::msf_pose_callback, this, std::placeholders::_1, false );
            msf_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>( "/msf_core/pose", rclcpp::QoS( 1 ),
                                                                                                     fcn_false );

            std::function<void( const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg )> fcn_true =
                std::bind( &ScanMatchingOdometryComponent::msf_pose_callback, this, std::placeholders::_1, true );
            msf_pose_after_update_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/msf_core/pose_after_update", rclcpp::QoS( 1 ), fcn_true );
        }

        // points_sub         = nh.subscribe( "/filtered_points", 256, &ScanMatchingOdometryComponent::cloud_callback, this );
        points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>( "/filtered_points", rclcpp::QoS( 256 ),
                                                                               std::bind( &ScanMatchingOdometryComponent::cloud_callback,
                                                                                          this, std::placeholders::_1 ) );
        // read_until_pub     = nh.advertise<std_msgs::Header>( "/scan_matching_odometry/read_until", 32 );
        // odom_pub           = nh.advertise<nav_msgs::Odometry>( "/scan_matching_odometry/odom", 32 );
        // trans_pub          = nh.advertise<geometry_msgs::TransformStamped>( "/scan_matching_odometry/transform", 32 );
        // status_pub         = private_nh.advertise<ScanMatchingStatus>( "/scan_matching_odometry/status", 8 );
        // aligned_points_pub = nh.advertise<sensor_msgs::PointCloud2>( "/scan_matching_odometry/aligned_points", 32 );
        read_until_pub = this->create_publisher<std_msgs::msg::Header>( "/scan_matching_odometry/read_until", rclcpp::QoS( 32 ) );
        odom_pub       = this->create_publisher<nav_msgs::msg::Odometry>( "/scan_matching_odometry/odom", rclcpp::QoS( 32 ) );
        trans_pub  = this->create_publisher<geometry_msgs::msg::TransformStamped>( "/scan_matching_odometry/transform", rclcpp::QoS( 32 ) );
        status_pub = this->create_publisher<hdl_graph_slam::msg::ScanMatchingStatus>( "/scan_matching_odometry/status", rclcpp::QoS( 8 ) );
        aligned_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>( "/scan_matching_odometry/aligned_points",
                                                                                    rclcpp::QoS( 32 ) );

        // Initialize the transform broadcaster
        odom_broadcaster     = std::make_unique<tf2_ros::TransformBroadcaster>( *this );
        keyframe_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>( *this );

        // Initialize the transform listener
        tf_buffer   = std::make_unique<tf2_ros::Buffer>( this->get_clock() );
        tf_listener = std::make_shared<tf2_ros::TransformListener>( *tf_buffer );
    }

private:
    /**
     * @brief initialize parameters
     */
    void initialize_params()
    {
        // TODO: ROS2 parameter handling

        // auto& pnh           = private_nh; // No more node handles in ROS2
        // points_topic        = pnh.param<std::string>( "points_topic", "/velodyne_points" );
        // odom_frame_id       = pnh.param<std::string>( "odom_frame_id", "odom" );
        // robot_odom_frame_id = pnh.param<std::string>( "robot_odom_frame_id", "robot_odom" );
        points_topic        = this->declare_parameter<std::string>( "points_topic", "/velodyne_points" );
        odom_frame_id       = this->declare_parameter<std::string>( "odom_frame_id", "odom" );
        robot_odom_frame_id = this->declare_parameter<std::string>( "robot_odom_frame_id", "robot_odom" );

        // The minimum tranlational distance and rotation angle between keyframes.
        // If this value is zero, frames are always compared with the previous frame
        // keyframe_delta_trans = pnh.param<double>( "keyframe_delta_trans", 0.25 );
        // keyframe_delta_angle = pnh.param<double>( "keyframe_delta_angle", 0.15 );
        // keyframe_delta_time  = pnh.param<double>( "keyframe_delta_time", 1.0 );
        keyframe_delta_trans = this->declare_parameter<double>( "keyframe_delta_trans", 0.25 );
        keyframe_delta_angle = this->declare_parameter<double>( "keyframe_delta_angle", 0.15 );
        keyframe_delta_time  = this->declare_parameter<double>( "keyframe_delta_time", 1.0 );

        // Registration validation by thresholding
        // transform_thresholding = pnh.param<bool>( "transform_thresholding", false );
        // max_acceptable_trans   = pnh.param<double>( "max_acceptable_trans", 1.0 );
        // max_acceptable_angle   = pnh.param<double>( "max_acceptable_angle", 1.0 );
        transform_thresholding = this->declare_parameter<bool>( "transform_thresholding", false );
        max_acceptable_trans   = this->declare_parameter<double>( "max_acceptable_trans", 1.0 );
        max_acceptable_angle   = this->declare_parameter<double>( "max_acceptable_angle", 1.0 );

        // select a downsample method (VOXELGRID, APPROX_VOXELGRID, NONE)
        // std::string downsample_method     = pnh.param<std::string>( "downsample_method", "VOXELGRID" );
        // double      downsample_resolution = pnh.param<double>( "downsample_resolution", 0.1 );
        std::string downsample_method     = this->declare_parameter<std::string>( "downsample_method", "VOXELGRID" );
        double      downsample_resolution = this->declare_parameter<double>( "downsample_resolution", 0.1 );
        if( downsample_method == "VOXELGRID" ) {
            std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
            auto voxelgrid = new pcl::VoxelGrid<PointT>();
            voxelgrid->setLeafSize( downsample_resolution, downsample_resolution, downsample_resolution );
            downsample_filter.reset( voxelgrid );
        } else if( downsample_method == "APPROX_VOXELGRID" ) {
            std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
            pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid( new pcl::ApproximateVoxelGrid<PointT>() );
            approx_voxelgrid->setLeafSize( downsample_resolution, downsample_resolution, downsample_resolution );
            downsample_filter = approx_voxelgrid;
        } else {
            if( downsample_method != "NONE" ) {
                std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
                std::cerr << "       : use passthrough filter" << std::endl;
            }
            std::cout << "downsample: NONE" << std::endl;
            pcl::PassThrough<PointT>::Ptr passthrough( new pcl::PassThrough<PointT>() );
            downsample_filter = passthrough;
        }

        // Declare enable_robot_odometry_init_guess at this point once, TODO delete if declaring it is unnecessary
        this->declare_parameter<bool>( "enable_robot_odometry_init_guess", false );

        // TODO: ROS2 verify
        auto node    = shared_from_this();
        registration = select_registration_method( node );
    }

    /**
     * @brief callback for point clouds
     * @param cloud_msg  point cloud msg
     */
    // void cloud_callback(  sensor_msgs::msg::PointCloud2ConstPtr& cloud_msg )
    // {
    //     if( !ros::ok() ) {
    //         return;
    //     }

    //     pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT>() );
    //     pcl::fromROSMsg( *cloud_msg, *cloud );

    //     Eigen::Matrix4f pose = matching( cloud_msg->header.stamp, cloud );
    //     publish_odometry( cloud_msg->header.stamp, cloud_msg->header.frame_id, pose );

    //     // In offline estimation, point clouds until the published time will be supplied
    //     std_msgs::HeaderPtr read_until( new std_msgs::Header() );
    //     read_until->frame_id = points_topic;
    //     read_until->stamp    = cloud_msg->header.stamp + ros::Duration( 1, 0 );
    //     read_until_pub.publish( read_until );

    //     read_until->frame_id = "/filtered_points";
    //     read_until_pub.publish( read_until );
    // }
    // TODO: maybe speed up by using unique_ptr callback https://docs.ros.org/en/foxy/Tutorials/Demos/Intra-Process-Communication.html
    void cloud_callback( sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg )
    {
        // if( !ros::ok() ) {
        if( !rclcpp::ok() ) {
            return;
        }


        pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT>() );
        pcl::fromROSMsg( *cloud_msg, *cloud );

        Eigen::Matrix4f pose = matching( cloud_msg->header.stamp, cloud );
        publish_odometry( cloud_msg->header.stamp, cloud_msg->header.frame_id, pose );

        // In offline estimation, point clouds until the published time will be supplied
        // std_msgs::HeaderPtr read_until( new std_msgs::Header() );
        // read_until->frame_id = points_topic;
        // read_until->stamp    = cloud_msg->header.stamp + ros::Duration( 1, 0 );
        // read_until_pub.publish( read_until );

        // read_until->frame_id = "/filtered_points";
        // read_until_pub.publish( read_until );
        // TODO: ROS2 we use Header msgs instead of Header::Ptr, verify
        std_msgs::msg::Header read_until;
        read_until.frame_id = points_topic;
        read_until.stamp    = rclcpp::Time( cloud_msg->header.stamp ) + rclcpp::Duration( 1, 0 );
        read_until_pub->publish( read_until );

        read_until.frame_id = "/filtered_points";
        read_until_pub->publish( read_until );
    }

    // void msf_pose_callback( const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg, bool after_update )
    void msf_pose_callback( const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg, bool after_update )
    {
        if( after_update ) {
            msf_pose_after_update = pose_msg;
        } else {
            msf_pose = pose_msg;
        }
    }

    /**
     * @brief downsample a point cloud
     * @param cloud  input cloud
     * @return downsampled point cloud
     */
    pcl::PointCloud<PointT>::ConstPtr downsample( const pcl::PointCloud<PointT>::ConstPtr& cloud ) const
    {
        if( !downsample_filter ) {
            return cloud;
        }

        pcl::PointCloud<PointT>::Ptr filtered( new pcl::PointCloud<PointT>() );
        downsample_filter->setInputCloud( cloud );
        downsample_filter->filter( *filtered );

        return filtered;
    }

    /**
     * @brief estimate the relative pose between an input cloud and a keyframe cloud
     * @param stamp  the timestamp of the input cloud
     * @param cloud  the input cloud
     * @return the relative pose between the input cloud and the keyframe cloud
     */
    // Eigen::Matrix4f matching( const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud )
    Eigen::Matrix4f matching( const rclcpp::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud )
    {
        if( !keyframe ) {
            // prev_time = ros::Time();
            prev_time = rclcpp::Time();
            prev_trans.setIdentity();
            keyframe_pose.setIdentity();
            keyframe_stamp = stamp;
            keyframe       = downsample( cloud );
            registration->setInputTarget( keyframe );
            return Eigen::Matrix4f::Identity();
        }

        auto filtered = downsample( cloud );
        registration->setInputSource( filtered );

        std::string       msf_source;
        Eigen::Isometry3f msf_delta = Eigen::Isometry3f::Identity();

        // if( private_nh.param<bool>( "enable_imu_frontend", false ) ) {
        // enable_imu_frontend declared in onInit
        if( this->get_parameter( "enable_imu_frontend" ).as_bool() ) {
            // if( msf_pose && msf_pose->header.stamp > keyframe_stamp && msf_pose_after_update
            //     && msf_pose_after_update->header.stamp > keyframe_stamp ) {
            if( msf_pose && rclcpp::Time( msf_pose->header.stamp ) > keyframe_stamp && msf_pose_after_update
                && rclcpp::Time( msf_pose_after_update->header.stamp ) > keyframe_stamp ) {
                Eigen::Isometry3d pose0 = pose2isometry( msf_pose_after_update->pose.pose );
                Eigen::Isometry3d pose1 = pose2isometry( msf_pose->pose.pose );
                Eigen::Isometry3d delta = pose0.inverse() * pose1;

                msf_source = "imu";
                msf_delta  = delta.cast<float>();
            } else {
                std::cerr << "msf data is too old" << std::endl;
            }
            // } else if( private_nh.param<bool>( "enable_robot_odometry_init_guess", false ) && !prev_time.isZero() ) {
            // ROS2: enable_robot_odometry_init_guess is declared once in the constructor so that it is only evaluated here, maybe declaring
            // it in the constructor is unnecessary, depending on launch script or parameter yaml
        } else if( this->get_parameter( "enable_robot_odometry_init_guess" ).as_bool() && !( prev_time.nanoseconds() == 0 ) ) {
            // tf::StampedTransform transform;
            geometry_msgs::msg::TransformStamped transform;
            // if( tf_listener.waitForTransform( cloud->header.frame_id, stamp, cloud->header.frame_id, prev_time, robot_odom_frame_id,
            //                                   ros::Duration( 0 ) ) ) {
            //     tf_listener.lookupTransform( cloud->header.frame_id, stamp, cloud->header.frame_id, prev_time, robot_odom_frame_id,
            //                                  transform );
            // } else if( tf_listener.waitForTransform( cloud->header.frame_id, ros::Time( 0 ), cloud->header.frame_id, prev_time,
            //                                          robot_odom_frame_id, ros::Duration( 0 ) ) ) {
            //     tf_listener.lookupTransform( cloud->header.frame_id, ros::Time( 0 ), cloud->header.frame_id, prev_time,
            //     robot_odom_frame_id,
            //                                  transform );
            // }
            // According to https://answers.ros.org/question/312648/could-not-find-waitfortransform-function-in-tf2-package-of-ros2/ the
            // equivalent for waitforTranform is to use canTransform of tfBuffer with a timeout, TODO, verify
            if( tf_buffer->canTransform( cloud->header.frame_id, stamp, cloud->header.frame_id, prev_time, robot_odom_frame_id,
                                         rclcpp::Duration( 0 ) ) ) {
                try {
                    transform = tf_buffer->lookupTransform( cloud->header.frame_id, stamp, cloud->header.frame_id, prev_time,
                                                            robot_odom_frame_id );
                } catch( const tf2::TransformException& ex ) {
                    // Lets info a quite verbose message
                    RCLCPP_INFO( this->get_logger(),
                                 "Could not look up transform with target frame %s, target time %9.f, source frame %s, source time %.9f, "
                                 "fixed frame %s: %s",
                                 cloud->header.frame_id.c_str(), stamp.seconds(), cloud->header.frame_id.c_str(), prev_time.seconds(),
                                 robot_odom_frame_id.c_str(), ex.what() );
                }
            } else if( tf_buffer->canTransform( cloud->header.frame_id, rclcpp::Time( 0 ), cloud->header.frame_id, prev_time,
                                                robot_odom_frame_id, rclcpp::Duration( 0 ) ) ) {
                try {
                    transform = tf_buffer->lookupTransform( cloud->header.frame_id, rclcpp::Time( 0 ), cloud->header.frame_id, prev_time,
                                                            robot_odom_frame_id );
                } catch( const tf2::TransformException& ex ) {
                    // Lets info a quite verbose message
                    RCLCPP_INFO( this->get_logger(),
                                 "Could not look up transform with target frame %s, target time %9.f, source frame %s, source time %.9f, "
                                 "fixed frame %s: %s",
                                 cloud->header.frame_id.c_str(), stamp.seconds(), cloud->header.frame_id.c_str(), prev_time.seconds(),
                                 robot_odom_frame_id.c_str(), ex.what() );
                }
            }

            // if( transform.stamp_.isZero() ) {
            if( rclcpp::Time( transform.header.stamp ).nanoseconds() == 0 ) {
                RCLCPP_WARN_STREAM( this->get_logger(),
                                    "failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id );
            } else {
                msf_source = "odometry";
                msf_delta  = tf2isometry( transform ).cast<float>();
            }
        }

        pcl::PointCloud<PointT>::Ptr aligned( new pcl::PointCloud<PointT>() );
        registration->align( *aligned, prev_trans * msf_delta.matrix() );

        publish_scan_matching_status( stamp, cloud->header.frame_id, aligned, msf_source, msf_delta );

        if( !registration->hasConverged() ) {
            RCLCPP_INFO_STREAM( this->get_logger(), "scan matching has not converged!!" );
            // NODELET_INFO_STREAM( "ignore this frame(" << stamp << ")" );
            RCLCPP_INFO_STREAM( this->get_logger(), "ignore this frame(" << stamp.seconds() << ")" );
            return keyframe_pose * prev_trans;
        }

        Eigen::Matrix4f trans = registration->getFinalTransformation();
        Eigen::Matrix4f odom  = keyframe_pose * trans;

        if( transform_thresholding ) {
            Eigen::Matrix4f delta = prev_trans.inverse() * trans;
            double          dx    = delta.block<3, 1>( 0, 3 ).norm();
            double          da    = std::acos( Eigen::Quaternionf( delta.block<3, 3>( 0, 0 ) ).w() );

            if( dx > max_acceptable_trans || da > max_acceptable_angle ) {
                RCLCPP_INFO_STREAM( this->get_logger(), "too large transform!!  " << dx << "[m] " << da << "[rad]" );
                // NODELET_INFO_STREAM( "ignore this frame(" << stamp << ")" );
                RCLCPP_INFO_STREAM( this->get_logger(), "ignore this frame(" << stamp.seconds() << ")" );
                return keyframe_pose * prev_trans;
            }
        }

        prev_time  = stamp;
        prev_trans = trans;

        auto keyframe_trans = matrix2transform( stamp, keyframe_pose, odom_frame_id, "keyframe" );
        // keyframe_broadcaster.sendTransform( keyframe_trans );
        keyframe_broadcaster->sendTransform( keyframe_trans );

        double delta_trans = trans.block<3, 1>( 0, 3 ).norm();
        double delta_angle = std::acos( Eigen::Quaternionf( trans.block<3, 3>( 0, 0 ) ).w() );
        // double delta_time  = ( stamp - keyframe_stamp ).toSec();
        double delta_time = ( stamp - keyframe_stamp ).seconds();
        if( delta_trans > keyframe_delta_trans || delta_angle > keyframe_delta_angle || delta_time > keyframe_delta_time ) {
            keyframe = filtered;
            registration->setInputTarget( keyframe );

            keyframe_pose  = odom;
            keyframe_stamp = stamp;
            prev_time      = stamp;
            prev_trans.setIdentity();
        }

        // if( aligned_points_pub.getNumSubscribers() > 0 ) {
        if( aligned_points_pub->get_subscription_count() > 0 ) {
            pcl::transformPointCloud( *cloud, *aligned, odom );
            aligned->header.frame_id = odom_frame_id;
            sensor_msgs::msg::PointCloud2 aligned_ros2;
            // TODO: ROS2 optimize out this operation somehow?
            pcl::toROSMsg( *aligned, aligned_ros2 );
            // aligned_points_pub.publish( *aligned );
            aligned_points_pub->publish( aligned_ros2 );
        }

        return odom;
    }

    /**
     * @brief publish odometry
     * @param stamp  timestamp
     * @param pose   odometry pose to be published
     */
    // void publish_odometry( const ros::Time& stamp, const std::string& base_frame_id, const Eigen::Matrix4f& pose )
    void publish_odometry( const rclcpp::Time& stamp, const std::string& base_frame_id, const Eigen::Matrix4f& pose )
    {
        // publish transform stamped for IMU integration
        // geometry_msgs::TransformStamped odom_trans = matrix2transform( stamp, pose, odom_frame_id, base_frame_id );
        geometry_msgs::msg::TransformStamped odom_trans = matrix2transform( stamp, pose, odom_frame_id, base_frame_id );
        // trans_pub.publish( odom_trans );
        trans_pub->publish( odom_trans );

        // broadcast the transform over tf
        // odom_broadcaster.sendTransform( odom_trans );
        // TODO: verify
        odom_broadcaster->sendTransform( odom_trans );

        // publish the transform
        // nav_msgs::Odometry odom;
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = stamp;
        odom.header.frame_id = odom_frame_id;

        odom.pose.pose.position.x  = pose( 0, 3 );
        odom.pose.pose.position.y  = pose( 1, 3 );
        odom.pose.pose.position.z  = pose( 2, 3 );
        odom.pose.pose.orientation = odom_trans.transform.rotation;

        odom.child_frame_id        = base_frame_id;
        odom.twist.twist.linear.x  = 0.0;
        odom.twist.twist.linear.y  = 0.0;
        odom.twist.twist.angular.z = 0.0;

        // odom_pub.publish( odom );
        odom_pub->publish( odom );
    }


    /**
     * @brief publish scan matching status
     */
    // void publish_scan_matching_status( const ros::Time& stamp, const std::string& frame_id,
    void publish_scan_matching_status( const rclcpp::Time& stamp, const std::string& frame_id,
                                       pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned, const std::string& msf_source,
                                       const Eigen::Isometry3f& msf_delta )
    {
        // if( !status_pub.getNumSubscribers() ) {
        if( !status_pub->get_subscription_count() ) {
            return;
        }

        hdl_graph_slam::msg::ScanMatchingStatus status;
        status.header.frame_id = frame_id;
        status.header.stamp    = stamp;
        status.has_converged   = registration->hasConverged();
        status.matching_error  = registration->getFitnessScore();

        const double max_correspondence_dist = 0.5;

        int                num_inliers = 0;
        std::vector<int>   k_indices;
        std::vector<float> k_sq_dists;
        for( int i = 0; i < aligned->size(); i++ ) {
            const auto& pt = aligned->at( i );
            registration->getSearchMethodTarget()->nearestKSearch( pt, 1, k_indices, k_sq_dists );
            if( k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist ) {
                num_inliers++;
            }
        }
        status.inlier_fraction = static_cast<float>( num_inliers ) / aligned->size();

        status.relative_pose = isometry2pose( Eigen::Isometry3f( registration->getFinalTransformation() ).cast<double>() );

        if( !msf_source.empty() ) {
            status.prediction_labels.resize( 1 );
            status.prediction_labels[0].data = msf_source;

            status.prediction_errors.resize( 1 );
            Eigen::Isometry3f error     = Eigen::Isometry3f( registration->getFinalTransformation() ).inverse() * msf_delta;
            status.prediction_errors[0] = isometry2pose( error.cast<double>() );
        }

        // status_pub.publish( status );
        status_pub->publish( status );
    }


private:
    // ROS topics
    // ros::NodeHandle nh;
    // ros::NodeHandle private_nh;

    // ros::Subscriber points_sub;
    // ros::Subscriber msf_pose_sub;
    // ros::Subscriber msf_pose_after_update_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr                 points_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr msf_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr msf_pose_after_update_sub;

    // ros::Publisher           odom_pub;
    // ros::Publisher           trans_pub;
    // ros::Publisher           aligned_points_pub;
    // ros::Publisher           status_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                 odom_pub;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr    trans_pub;
    rclcpp::Publisher<hdl_graph_slam::msg::ScanMatchingStatus>::SharedPtr status_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr           aligned_points_pub;


    // tf::TransformListener    tf_listener;
    // tf::TransformBroadcaster odom_broadcaster;
    // tf::TransformBroadcaster keyframe_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener;
    std::unique_ptr<tf2_ros::Buffer>               tf_buffer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
    std::unique_ptr<tf2_ros::TransformBroadcaster> keyframe_broadcaster;

    std::string points_topic;
    std::string odom_frame_id;
    std::string robot_odom_frame_id;
    // ros::Publisher read_until_pub;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr read_until_pub;

    // keyframe parameters
    double keyframe_delta_trans;  // minimum distance between keyframes
    double keyframe_delta_angle;  //
    double keyframe_delta_time;   //

    // registration validation by thresholding
    bool   transform_thresholding;  //
    double max_acceptable_trans;    //
    double max_acceptable_angle;

    // odometry calculation
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msf_pose;
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msf_pose_after_update;

    // ros::Time                         prev_time;
    // ros::Time                         keyframe_stamp;  // keyframe time
    rclcpp::Time                      prev_time;
    Eigen::Matrix4f                   prev_trans;      // previous estimated transform from keyframe
    Eigen::Matrix4f                   keyframe_pose;   // keyframe pose
    rclcpp::Time                      keyframe_stamp;  // keyframe time
    pcl::PointCloud<PointT>::ConstPtr keyframe;        // keyframe point cloud

    //
    pcl::Filter<PointT>::Ptr               downsample_filter;
    pcl::Registration<PointT, PointT>::Ptr registration;
};

}  // namespace hdl_graph_slam


// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE( hdl_graph_slam::ScanMatchingOdometryComponent )
