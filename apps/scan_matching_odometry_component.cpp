// SPDX-License-Identifier: BSD-2-Clause

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <mrg_slam/registrations.hpp>
#include <mrg_slam/ros_utils.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vamex_slam_msgs/msg/scan_matching_status.hpp>

namespace mrg_slam {

class ScanMatchingOdometryComponent : public rclcpp::Node {
public:
    typedef pcl::PointXYZI PointT;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // We need to pass NodeOptions in ROS2 to register a component
    ScanMatchingOdometryComponent( const rclcpp::NodeOptions&            options,
                                   const std::vector<rclcpp::Parameter>& param_vec = std::vector<rclcpp::Parameter>() ) :
        Node( "scan_matching_odometry_component", options )
    {
        RCLCPP_INFO( this->get_logger(), "Initializing scan_matching_odometry_component..." );

        initialize_params( param_vec );

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

        registration = select_registration_method( static_cast<rclcpp::Node*>( this ) );

        if( enable_imu_frontend ) {
            // We need to define a special function to pass arguments to a ROS2 callback with multiple parameters
            // https://answers.ros.org/question/308386/ros2-add-arguments-to-callback/
            std::function<void( const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg )> fcn_false =
                std::bind( &ScanMatchingOdometryComponent::msf_pose_callback, this, std::placeholders::_1, false );
            msf_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>( "/msf_core/pose", rclcpp::QoS( 1 ),
                                                                                                     fcn_false );

            std::function<void( const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg )> fcn_true =
                std::bind( &ScanMatchingOdometryComponent::msf_pose_callback, this, std::placeholders::_1, true );
            msf_pose_after_update_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/msf_core/pose_after_update", rclcpp::QoS( 1 ), fcn_true );
        }

        points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>( "/filtered_points", rclcpp::QoS( 256 ),
                                                                               std::bind( &ScanMatchingOdometryComponent::cloud_callback,
                                                                                          this, std::placeholders::_1 ) );

        read_until_pub = this->create_publisher<std_msgs::msg::Header>( "/scan_matching_odometry/read_until", rclcpp::QoS( 32 ) );
        odom_pub       = this->create_publisher<nav_msgs::msg::Odometry>( "/scan_matching_odometry/odom", rclcpp::QoS( 32 ) );
        trans_pub  = this->create_publisher<geometry_msgs::msg::TransformStamped>( "/scan_matching_odometry/transform", rclcpp::QoS( 32 ) );
        status_pub = this->create_publisher<vamex_slam_msgs::msg::ScanMatchingStatus>( "/scan_matching_odometry/status", rclcpp::QoS( 8 ) );
        aligned_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>( "/scan_matching_odometry/aligned_points",
                                                                                    rclcpp::QoS( 32 ) );

        // Initialize the transform broadcaster
        odom_broadcaster     = std::make_unique<tf2_ros::TransformBroadcaster>( *this );
        keyframe_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>( *this );

        // Initialize the transform listener
        tf_buffer   = std::make_unique<tf2_ros::Buffer>( this->get_clock() );
        tf_listener = std::make_shared<tf2_ros::TransformListener>( *tf_buffer );

        // Optionally print the all parameters declared in this node so far
        print_ros2_parameters( this->get_node_parameters_interface(), this->get_logger() );
    }


    virtual ~ScanMatchingOdometryComponent() {}

private:
    /**
     * @brief initialize parameters
     */
    void initialize_params( const std::vector<rclcpp::Parameter>& param_vec = std::vector<rclcpp::Parameter>() )
    {
        // Declare all parameters first
        this->declare_parameter<std::string>( "points_topic", "/velodyne_points" );
        this->declare_parameter<std::string>( "odom_frame_id", "odom" );
        this->declare_parameter<std::string>( "robot_odom_frame_id", "robot_odom" );

        this->declare_parameter<double>( "keyframe_delta_trans", 0.25 );
        this->declare_parameter<double>( "keyframe_delta_angle", 0.15 );
        this->declare_parameter<double>( "keyframe_delta_time", 1.0 );

        this->declare_parameter<bool>( "transform_thresholding", false );
        this->declare_parameter<double>( "max_acceptable_trans", 1.0 );
        this->declare_parameter<double>( "max_acceptable_angle", 1.0 );

        this->declare_parameter<bool>( "enable_robot_odometry_init_guess", false );
        this->declare_parameter<bool>( "enable_imu_frontend", false );

        this->declare_parameter<std::string>( "downsample_method", "VOXELGRID" );
        this->declare_parameter<double>( "downsample_resolution", 0.1 );

        // Regastration method parameters, used in select_registration_method()
        this->declare_parameter<std::string>( "registration_method", "FAST_GICP" );
        this->declare_parameter<int>( "reg_num_threads", 0 );
        this->declare_parameter<double>( "reg_transformation_epsilon", 0.1 );
        this->declare_parameter<int>( "reg_maximum_iterations", 64 );
        this->declare_parameter<double>( "reg_max_correspondence_distance", 2.0 );
        this->declare_parameter<int>( "reg_max_optimizer_iterations", 20 );
        this->declare_parameter<bool>( "reg_use_reciprocal_correspondences", false );
        this->declare_parameter<int>( "reg_correspondence_randomness", 20 );
        this->declare_parameter<double>( "reg_resolution", 1.0 );
        this->declare_parameter<std::string>( "reg_nn_search_method", "DIRECT7" );
        this->declare_parameter<std::string>( "result_dir", "" );

        // Overwrite parameters if param_vec is provided, use case manual composition (debugging)
        if( !param_vec.empty() ) {
            this->set_parameters( param_vec );
        }

        // Set all member variables
        downsample_method     = this->get_parameter( "downsample_method" ).as_string();
        downsample_resolution = this->get_parameter( "downsample_resolution" ).as_double();

        points_topic        = this->get_parameter( "points_topic" ).as_string();
        odom_frame_id       = this->get_parameter( "odom_frame_id" ).as_string();
        robot_odom_frame_id = this->get_parameter( "robot_odom_frame_id" ).as_string();

        keyframe_delta_trans = this->get_parameter( "keyframe_delta_trans" ).as_double();
        keyframe_delta_angle = this->get_parameter( "keyframe_delta_angle" ).as_double();
        keyframe_delta_time  = this->get_parameter( "keyframe_delta_time" ).as_double();

        transform_thresholding = this->get_parameter( "transform_thresholding" ).as_bool();
        max_acceptable_trans   = this->get_parameter( "max_acceptable_trans" ).as_double();
        max_acceptable_angle   = this->get_parameter( "max_acceptable_angle" ).as_double();

        enable_robot_odometry_init_guess = this->get_parameter( "enable_robot_odometry_init_guess" ).as_bool();
        enable_imu_frontend              = this->get_parameter( "enable_imu_frontend" ).as_bool();
        result_dir                       = this->get_parameter( "result_dir" ).as_string();
        if( result_dir.back() == '/' ) {
            result_dir.pop_back();
        }
    }

    /**
     * @brief callback for point clouds
     * @param cloud_msg  point cloud msg
     */
    void cloud_callback( sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg )
    {
        static int counter = 0;
        if( !rclcpp::ok() ) {
            return;
        }

        pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT>() );
        pcl::fromROSMsg( *cloud_msg, *cloud );

        auto            start = std::chrono::high_resolution_clock::now();
        Eigen::Matrix4f pose  = matching( cloud_msg->header.stamp, cloud );
        auto            end   = std::chrono::high_resolution_clock::now();
        registration_times.push_back( std::chrono::duration_cast<std::chrono::microseconds>( end - start ).count() );
        cloud_sizes.push_back( cloud->size() );
        if( counter % 100 == 0 ) {
            std::cout << "Average scan matching odom registration time: "
                      << std::accumulate( registration_times.begin(), registration_times.end(), 0.0 ) / registration_times.size() << "us"
                      << std::endl;
            std::cout << "average scan matching odom cloud size: "
                      << std::accumulate( cloud_sizes.begin(), cloud_sizes.end(), 0.0 ) / cloud_sizes.size() << std::endl;
        }

        publish_odometry( cloud_msg->header.stamp, cloud_msg->header.frame_id, pose );

        // In offline estimation, point clouds until the published time will be supplied
        std_msgs::msg::Header read_until;
        read_until.frame_id = points_topic;
        read_until.stamp = ( rclcpp::Time( cloud_msg->header.stamp ) + rclcpp::Duration( 1, 0 ) ).operator builtin_interfaces::msg::Time();
        read_until_pub->publish( read_until );

        read_until.frame_id = "/filtered_points";
        read_until_pub->publish( read_until );

        counter++;
    }

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

        if( enable_imu_frontend ) {
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
        } else if( enable_robot_odometry_init_guess && !( prev_time.nanoseconds() == 0 ) ) {
            geometry_msgs::msg::TransformStamped transform;
            // According to https://answers.ros.org/question/312648/could-not-find-waitfortransform-function-in-tf2-package-of-ros2/ the
            // equivalent for waitforTranform is to use canTransform of tfBuffer with a timeout, TODO, verify
            if( tf_buffer->canTransform( cloud->header.frame_id, stamp, cloud->header.frame_id, prev_time, robot_odom_frame_id,
                                         rclcpp::Duration( 0, 0 ) ) ) {
                try {
                    transform = tf_buffer->lookupTransform( cloud->header.frame_id, stamp, cloud->header.frame_id, prev_time,
                                                            robot_odom_frame_id );
                } catch( const tf2::TransformException& ex ) {
                    RCLCPP_WARN( this->get_logger(),
                                 "Could not look up transform with target frame %s, target time %9.f, source frame %s, source time %.9f, "
                                 "fixed frame %s: %s",
                                 cloud->header.frame_id.c_str(), stamp.seconds(), cloud->header.frame_id.c_str(), prev_time.seconds(),
                                 robot_odom_frame_id.c_str(), ex.what() );
                }
            } else if( tf_buffer->canTransform( cloud->header.frame_id, rclcpp::Time( 0 ), cloud->header.frame_id, prev_time,
                                                robot_odom_frame_id, rclcpp::Duration( 0, 0 ) ) ) {
                try {
                    transform = tf_buffer->lookupTransform( cloud->header.frame_id, rclcpp::Time( 0 ), cloud->header.frame_id, prev_time,
                                                            robot_odom_frame_id );
                } catch( const tf2::TransformException& ex ) {
                    RCLCPP_WARN( this->get_logger(),
                                 "Could not look up transform with target frame %s, target time %9.f, source frame %s, source time %.9f, "
                                 "fixed frame %s: %s",
                                 cloud->header.frame_id.c_str(), stamp.seconds(), cloud->header.frame_id.c_str(), prev_time.seconds(),
                                 robot_odom_frame_id.c_str(), ex.what() );
                }
            }

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
                RCLCPP_INFO_STREAM( this->get_logger(), "ignore this frame(" << stamp.seconds() << ")" );
                return keyframe_pose * prev_trans;
            }
        }

        prev_time  = stamp;
        prev_trans = trans;

        // broadcast keyframe with namespace aware topic name
        std::string keyframe_str   = this->get_effective_namespace() == "/" ? "keyframe" : this->get_effective_namespace() + "/keyframe";
        auto        keyframe_trans = matrix2transform( stamp, keyframe_pose, odom_frame_id, keyframe_str );
        keyframe_broadcaster->sendTransform( keyframe_trans );

        double delta_trans = trans.block<3, 1>( 0, 3 ).norm();
        double delta_angle = std::acos( Eigen::Quaternionf( trans.block<3, 3>( 0, 0 ) ).w() );
        double delta_time  = ( stamp - keyframe_stamp ).seconds();
        if( delta_trans > keyframe_delta_trans || delta_angle > keyframe_delta_angle || delta_time > keyframe_delta_time ) {
            keyframe = filtered;
            registration->setInputTarget( keyframe );

            keyframe_pose  = odom;
            keyframe_stamp = stamp;
            prev_time      = stamp;
            prev_trans.setIdentity();
        }

        if( aligned_points_pub->get_subscription_count() > 0 ) {
            pcl::transformPointCloud( *cloud, *aligned, odom );
            aligned->header.frame_id = odom_frame_id;
            sensor_msgs::msg::PointCloud2 aligned_ros2;
            pcl::toROSMsg( *aligned, aligned_ros2 );
            aligned_points_pub->publish( aligned_ros2 );
        }

        return odom;
    }

    /**
     * @brief publish odometry
     * @param stamp  timestamp
     * @param pose   odometry pose to be published
     */
    void publish_odometry( const rclcpp::Time& stamp, const std::string& base_frame_id, const Eigen::Matrix4f& pose )
    {
        // publish transform stamped for IMU integration
        geometry_msgs::msg::TransformStamped odom_trans = matrix2transform( stamp, pose, odom_frame_id, base_frame_id );
        trans_pub->publish( odom_trans );

        // broadcast the transform over tf
        odom_broadcaster->sendTransform( odom_trans );

        // publish the transform
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = stamp.operator builtin_interfaces::msg::Time();
        odom.header.frame_id = odom_frame_id;

        odom.pose.pose.position.x  = pose( 0, 3 );
        odom.pose.pose.position.y  = pose( 1, 3 );
        odom.pose.pose.position.z  = pose( 2, 3 );
        odom.pose.pose.orientation = odom_trans.transform.rotation;

        odom.child_frame_id        = base_frame_id;
        odom.twist.twist.linear.x  = 0.0;
        odom.twist.twist.linear.y  = 0.0;
        odom.twist.twist.angular.z = 0.0;

        // TODO transform odometry into correct frame for displaying it correctly in rviz?
        odom_pub->publish( odom );
    }


    /**
     * @brief publish scan matching status
     */
    void publish_scan_matching_status( const rclcpp::Time& stamp, const std::string& frame_id,
                                       pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned, const std::string& msf_source,
                                       const Eigen::Isometry3f& msf_delta )
    {
        if( !status_pub->get_subscription_count() ) {
            return;
        }

        vamex_slam_msgs::msg::ScanMatchingStatus status;
        status.header.frame_id = frame_id;
        status.header.stamp    = stamp.operator builtin_interfaces::msg::Time();
        status.has_converged   = registration->hasConverged();
        status.matching_error  = registration->getFitnessScore();

        const double max_correspondence_dist = 0.5;

        int                num_inliers = 0;
        std::vector<int>   k_indices;
        std::vector<float> k_sq_dists;
        for( int i = 0; i < (int)aligned->size(); i++ ) {
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

        status_pub->publish( status );
    }


private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr                 points_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr msf_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr msf_pose_after_update_sub;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                  odom_pub;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr     trans_pub;
    rclcpp::Publisher<vamex_slam_msgs::msg::ScanMatchingStatus>::SharedPtr status_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr            aligned_points_pub;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr                    read_until_pub;

    std::shared_ptr<tf2_ros::TransformListener>    tf_listener;
    std::unique_ptr<tf2_ros::Buffer>               tf_buffer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
    std::unique_ptr<tf2_ros::TransformBroadcaster> keyframe_broadcaster;

    // odometry calculation
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msf_pose;
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msf_pose_after_update;

    rclcpp::Time                      prev_time;
    Eigen::Matrix4f                   prev_trans;      // previous estimated transform from keyframe
    Eigen::Matrix4f                   keyframe_pose;   // keyframe pose
    rclcpp::Time                      keyframe_stamp;  // keyframe time
    pcl::PointCloud<PointT>::ConstPtr keyframe;        // keyframe point cloud

    //
    pcl::Filter<PointT>::Ptr               downsample_filter;
    pcl::Registration<PointT, PointT>::Ptr registration;

    // Algorithm, ROS2 parameters
    std::string points_topic;
    std::string odom_frame_id;
    std::string robot_odom_frame_id;

    // keyframe parameters
    double keyframe_delta_trans;  // minimum distance between keyframes
    double keyframe_delta_angle;  //
    double keyframe_delta_time;   //

    // registration validation by thresholding
    bool   transform_thresholding;  //
    double max_acceptable_trans;    //
    double max_acceptable_angle;

    bool enable_robot_odometry_init_guess;
    bool enable_imu_frontend;

    std::string downsample_method;
    double      downsample_resolution;

    std::string         result_dir;
    std::vector<double> registration_times;
    std::vector<int>    cloud_sizes;
};

}  // namespace mrg_slam


// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE( mrg_slam::ScanMatchingOdometryComponent )
