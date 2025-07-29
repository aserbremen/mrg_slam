// SPDX-License-Identifier: BSD-2-Clause

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mrg_slam/ros_utils.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

namespace mrg_slam {

class PrefilteringComponent : public rclcpp::Node {
public:
    typedef pcl::PointXYZI PointT;

    // We need to pass NodeOptions in ROS2 to register a component
    PrefilteringComponent( const rclcpp::NodeOptions& options ) : Node( "prefiltering_component", options )
    {
        RCLCPP_INFO( get_logger(), "Initializing prefiltering_component..." );

        initialize_params();

        // Initialize voxel grid filters
        voxelgrid_filter_        = std::make_shared<pcl::VoxelGrid<PointT>>();
        approx_voxelgrid_filter_ = std::make_shared<pcl::ApproximateVoxelGrid<PointT>>();

        double      downsample_resolution = get_parameter( "downsample_resolution" ).as_double();
        std::string downsample_method_    = get_parameter( "downsample_method" ).as_string();

        if( downsample_method_ == "VOXELGRID" ) {
            RCLCPP_INFO_STREAM( get_logger(), "downsample: VOXELGRID " << downsample_resolution );
        } else if( downsample_method_ == "APPROX_VOXELGRID" ) {
            RCLCPP_INFO_STREAM( get_logger(), "downsample: APPROX_VOXELGRID " << downsample_resolution );
        } else {
            if( downsample_method_ != "NONE" ) {
                RCLCPP_WARN_STREAM( get_logger(), "unknown downsampling type (" << downsample_method_ << "), use passthrough filter" );
            }
            RCLCPP_INFO( get_logger(), "downsample: NONE" );
        }

        // Initialize outlier removal filters
        statistical_outlier_removal_filter_ = std::make_shared<pcl::StatisticalOutlierRemoval<PointT>>();
        radius_outlier_removal_filter_      = std::make_shared<pcl::RadiusOutlierRemoval<PointT>>();

        std::string outlier_removal_method_ = get_parameter( "outlier_removal_method" ).as_string();
        if( outlier_removal_method_ == "STATISTICAL" ) {
            RCLCPP_INFO_STREAM( get_logger(), "outlier_removal: STATISTICAL mean_k = "
                                                  << get_parameter( "statistical_mean_k" ).as_int()
                                                  << ", stddev = " << get_parameter( "statistical_stddev" ).as_double() );
        } else if( outlier_removal_method_ == "RADIUS" ) {
            RCLCPP_INFO_STREAM( get_logger(), "outlier_removal: RADIUS radius = " << get_parameter( "radius_radius" ).as_double()
                                                                                  << ", min_neighbors = "
                                                                                  << get_parameter( "radius_min_neighbors" ).as_int() );
        } else {
            RCLCPP_INFO_STREAM( get_logger(), "outlier_removal: NONE" );
        }

        if( get_parameter( "enable_deskewing" ).as_bool() ) {
            imu_sub_ = create_subscription<sensor_msgs::msg::Imu>( "imu/data", rclcpp::QoS( 1 ),
                                                                   std::bind( &PrefilteringComponent::imu_callback, this,
                                                                              std::placeholders::_1 ) );
        }

        points_sub_  = create_subscription<sensor_msgs::msg::PointCloud2>( "velodyne_points", rclcpp::QoS( 64 ),
                                                                           std::bind( &PrefilteringComponent::cloud_callback, this,
                                                                                      std::placeholders::_1 ) );
        points_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>( "prefiltering/filtered_points", rclcpp::QoS( 32 ) );
        colored_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>( "prefiltering/colored_points", rclcpp::QoS( 32 ) );

        // setup transform listener
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>( get_clock() );
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_ );

        // Optionally print the all parameters declared in this node so far
        print_ros2_parameters( get_node_parameters_interface(), get_logger() );
    }

    virtual ~PrefilteringComponent() {}

private:
    void initialize_params()
    {
        base_link_frame_ = declare_parameter<std::string>( "base_link_frame", "base_link" );
        // Downsampling parameters
        declare_parameter<std::string>( "downsample_method", "VOXELGRID" );
        declare_parameter<double>( "downsample_resolution", 0.1 );
        declare_parameter<int>( "downsample_min_points_per_voxel", 1 );
        // Outlier removal parameters
        declare_parameter<std::string>( "outlier_removal_method", "STATISTICAL" );
        declare_parameter<int>( "statistical_mean_k", 20 );
        declare_parameter<double>( "statistical_stddev", 1.0 );
        declare_parameter<double>( "radius_radius", 0.8 );
        declare_parameter<int>( "radius_min_neighbors", 2 );
        // Distance filter parameters
        declare_parameter<bool>( "enable_distance_filter", true );
        declare_parameter<double>( "distance_near_thresh", 1.0 );
        declare_parameter<double>( "distance_far_thresh", 35.0 );
        // Deskewing parameters
        declare_parameter<bool>( "enable_deskewing", false );
        declare_parameter<double>( "scan_period", 0.1 );
    }

    void imu_callback( sensor_msgs::msg::Imu::ConstSharedPtr imu_msg ) { imu_queue_.push_back( imu_msg ); }

    void cloud_callback( sensor_msgs::msg::PointCloud2::ConstSharedPtr src_cloud_ros )
    {
        // Convert to pcl pointcloud from ros PointCloud2
        pcl::PointCloud<PointT>::Ptr src_cloud = std::make_shared<pcl::PointCloud<PointT>>();
        pcl::fromROSMsg( *src_cloud_ros, *src_cloud );
        if( src_cloud->empty() ) {
            return;
        }

        src_cloud = deskewing( src_cloud );

        // if base_link_frame is defined, transform the input cloud to the frame
        if( !base_link_frame_.empty() ) {
            geometry_msgs::msg::TransformStamped transform;
            try {
                // lookupTransform contains a Duration as parameter
                transform = tf_buffer_->lookupTransform( base_link_frame_, src_cloud->header.frame_id, rclcpp::Time( 0 ),
                                                         rclcpp::Duration( 2, 0 ) );
            } catch( const tf2::TransformException& ex ) {
                RCLCPP_WARN( get_logger(), "Could not transform source frame %s to target frame %s: %s", src_cloud->header.frame_id.c_str(),
                             base_link_frame_.c_str(), ex.what() );
                RCLCPP_WARN( get_logger(), "Returning early in cloud_callback from prefiltering component" );
                return;
            }

            pcl::PointCloud<PointT>::Ptr transformed( new pcl::PointCloud<PointT>() );
            pcl_ros::transformPointCloud( *src_cloud, *transformed, transform );
            transformed->header.frame_id = base_link_frame_;
            transformed->header.stamp    = src_cloud->header.stamp;
            src_cloud                    = transformed;
        }

        pcl::PointCloud<PointT>::ConstPtr filtered = distance_filter( src_cloud );
        filtered                                   = downsample( filtered );
        filtered                                   = outlier_removal( filtered );
        sensor_msgs::msg::PointCloud2 filtered_ros;
        pcl::toROSMsg( *filtered, filtered_ros );

        points_pub_->publish( filtered_ros );
    }

    pcl::PointCloud<PointT>::ConstPtr downsample( const pcl::PointCloud<PointT>::ConstPtr& cloud ) const
    {
        std::string downsample_method_ = get_parameter( "downsample_method" ).as_string();
        if( downsample_method_ == "NONE" ) {
            return cloud;
        }

        pcl::PointCloud<PointT>::Ptr filtered( new pcl::PointCloud<PointT>() );
        double                       downsample_resolution = get_parameter( "downsample_resolution" ).as_double();
        if( downsample_method_ == "VOXELGRID" ) {
            voxelgrid_filter_->setLeafSize( downsample_resolution, downsample_resolution, downsample_resolution );
            voxelgrid_filter_->setMinimumPointsNumberPerVoxel( get_parameter( "downsample_min_points_per_voxel" ).as_int() );
            voxelgrid_filter_->setInputCloud( cloud );
            voxelgrid_filter_->filter( *filtered );
        } else if( downsample_method_ == "APPROX_VOXELGRID" ) {
            approx_voxelgrid_filter_->setLeafSize( downsample_resolution, downsample_resolution, downsample_resolution );
            approx_voxelgrid_filter_->setInputCloud( cloud );
            approx_voxelgrid_filter_->filter( *filtered );
        }
        filtered->header = cloud->header;

        return filtered;
    }

    pcl::PointCloud<PointT>::ConstPtr outlier_removal( const pcl::PointCloud<PointT>::ConstPtr& cloud ) const
    {
        std::string outlier_removal_method = get_parameter( "outlier_removal_method" ).as_string();
        if( outlier_removal_method == "NONE" ) {
            return cloud;
        }

        pcl::PointCloud<PointT>::Ptr filtered( new pcl::PointCloud<PointT>() );
        if( outlier_removal_method == "STATISTICAL" ) {
            statistical_outlier_removal_filter_->setMeanK( get_parameter( "statistical_mean_k" ).as_int() );
            statistical_outlier_removal_filter_->setStddevMulThresh( get_parameter( "statistical_stddev" ).as_double() );
            statistical_outlier_removal_filter_->setInputCloud( cloud );
            statistical_outlier_removal_filter_->filter( *filtered );
        } else if( outlier_removal_method == "RADIUS" ) {
            radius_outlier_removal_filter_->setRadiusSearch( get_parameter( "radius_radius" ).as_double() );
            radius_outlier_removal_filter_->setMinNeighborsInRadius( get_parameter( "radius_min_neighbors" ).as_int() );
            radius_outlier_removal_filter_->setInputCloud( cloud );
            radius_outlier_removal_filter_->filter( *filtered );
        }
        filtered->header = cloud->header;

        return filtered;
    }

    pcl::PointCloud<PointT>::ConstPtr distance_filter( const pcl::PointCloud<PointT>::ConstPtr& cloud ) const
    {
        if( !get_parameter( "enable_distance_filter" ).as_bool() ) {
            return cloud;
        }

        pcl::PointCloud<PointT>::Ptr filtered( new pcl::PointCloud<PointT>() );
        filtered->reserve( cloud->size() );

        double distance_near_thresh = get_parameter( "distance_near_thresh" ).as_double();
        double distance_far_thresh  = get_parameter( "distance_far_thresh" ).as_double();
        std::copy_if( cloud->begin(), cloud->end(), std::back_inserter( filtered->points ), [&]( const PointT& p ) {
            double d = p.getVector3fMap().norm();
            return d > distance_near_thresh && d < distance_far_thresh;
        } );

        filtered->width    = filtered->size();
        filtered->height   = 1;
        filtered->is_dense = false;

        filtered->header = cloud->header;

        return filtered;
    }

    pcl::PointCloud<PointT>::Ptr deskewing( const pcl::PointCloud<PointT>::Ptr& cloud )
    {
        rclcpp::Time stamp = pcl_conversions::fromPCL( cloud->header.stamp );
        if( imu_queue_.empty() ) {
            return cloud;
        }

        // the color encodes the point number in the point sequence
        if( colored_pub_->get_subscription_count() ) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored( new pcl::PointCloud<pcl::PointXYZRGB>() );
            colored->header   = cloud->header;
            colored->is_dense = cloud->is_dense;
            colored->width    = cloud->width;
            colored->height   = cloud->height;
            colored->resize( cloud->size() );

            for( int i = 0; i < (int)cloud->size(); i++ ) {
                double t                          = static_cast<double>( i ) / cloud->size();
                colored->at( i ).getVector4fMap() = cloud->at( i ).getVector4fMap();
                colored->at( i ).r                = 255 * t;
                colored->at( i ).g                = 128;
                colored->at( i ).b                = 255 * ( 1 - t );
            }
            sensor_msgs::msg::PointCloud2 colored_ros;
            pcl::toROSMsg( *colored, colored_ros );
            colored_pub_->publish( colored_ros );
        }

        sensor_msgs::msg::Imu::ConstSharedPtr imu_msg = imu_queue_.front();

        auto loc = imu_queue_.begin();
        for( ; loc != imu_queue_.end(); loc++ ) {
            imu_msg = ( *loc );
            if( rclcpp::Time( ( *loc )->header.stamp ) > stamp ) {
                break;
            }
        }

        imu_queue_.erase( imu_queue_.begin(), loc );

        Eigen::Vector3f ang_v( imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z );
        ang_v *= -1;

        pcl::PointCloud<PointT>::Ptr deskewed( new pcl::PointCloud<PointT>() );
        deskewed->header   = cloud->header;
        deskewed->is_dense = cloud->is_dense;
        deskewed->width    = cloud->width;
        deskewed->height   = cloud->height;
        deskewed->resize( cloud->size() );

        double scan_period = get_parameter( "scan_period" ).as_double();
        for( int i = 0; i < (int)cloud->size(); i++ ) {
            const auto& pt = cloud->at( i );

            // TODO (from original repo): transform IMU data into the LIDAR frame
            double             delta_t = scan_period * static_cast<double>( i ) / cloud->size();
            Eigen::Quaternionf delta_q( 1, delta_t / 2.0 * ang_v[0], delta_t / 2.0 * ang_v[1], delta_t / 2.0 * ang_v[2] );
            Eigen::Vector3f    pt_ = delta_q.inverse() * pt.getVector3fMap();

            deskewed->at( i )                  = cloud->at( i );
            deskewed->at( i ).getVector3fMap() = pt_;
        }

        return deskewed;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>     imu_queue_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    points_pub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pub_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;

    pcl::VoxelGrid<PointT>::Ptr            voxelgrid_filter_;
    pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid_filter_;

    // pcl::Filter<PointT>::Ptr                    outlier_removal_filter_;
    pcl::StatisticalOutlierRemoval<PointT>::Ptr statistical_outlier_removal_filter_;
    pcl::RadiusOutlierRemoval<PointT>::Ptr      radius_outlier_removal_filter_;

    // ROS2 parameters, not changed at runtime
    std::string base_link_frame_;  // the frame to which the point cloud will be transformed
};

}  // namespace mrg_slam

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE( mrg_slam::PrefilteringComponent )
