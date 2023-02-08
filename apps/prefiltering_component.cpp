// SPDX-License-Identifier: BSD-2-Clause

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hdl_graph_slam/ros_utils.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <nodelet/nodelet.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_ros/transforms.h>
// #include <ros/ros.h>
// #include <ros/time.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <tf/transform_listener.h>
// #include <pluginlib/class_list_macros.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

namespace hdl_graph_slam {

// class PrefilteringNodelet : public nodelet::Nodelet {
class PrefilteringComponent : public rclcpp::Node {
public:
    typedef pcl::PointXYZI PointT;

    // We need to pass NodeOptions in ROS2 to register a component
    PrefilteringComponent( const rclcpp::NodeOptions& options ) : Node( "prefiltering_component", options )
    {
        RCLCPP_INFO( this->get_logger(), "Initializing prefiltering_component..." );

        initialize_params();

        if( this->declare_parameter<bool>( "deskewing", false ) ) {
            // TODO: QOS? sensordata qos?
            imu_sub = this->create_subscription<sensor_msgs::msg::Imu>( "/imu/data", rclcpp::QoS( 1 ),
                                                                        std::bind( &PrefilteringComponent::imu_callback, this,
                                                                                   std::placeholders::_1 ) );
        }

        points_sub  = this->create_subscription<sensor_msgs::msg::PointCloud2>( "/velodyne_points", rclcpp::QoS( 64 ),
                                                                               std::bind( &PrefilteringComponent::cloud_callback, this,
                                                                                           std::placeholders::_1 ) );
        points_pub  = this->create_publisher<sensor_msgs::msg::PointCloud2>( "/prefiltering/filtered_points", rclcpp::QoS( 32 ) );
        colored_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>( "/prefiltering/colored_points", rclcpp::QoS( 32 ) );


        // setup transform listener
        tf_buffer   = std::make_unique<tf2_ros::Buffer>( this->get_clock() );
        tf_listener = std::make_shared<tf2_ros::TransformListener>( *tf_buffer );

        // auto node = shared_from_this();
        const auto& list_params = this->list_parameters( std::vector<std::string>{}, 0 );
        print_ros2_parameters( this->get_parameters( list_params.names ), this->get_logger() );
    }

    virtual ~PrefilteringComponent() {}

    // It seems like there is no onInit() method in ROS2, so we have to initiliaze the node in the constructor
    /*
    virtual void onInit()
    {
        // This class is the node handle as it is derived from rclcpp::Node
        // nh         = getNodeHandle();
        // private_nh = getPrivateNodeHandle();

        RCLCPP_INFO( this->get_logger(), "Initializing prefiltering_component..." );

        initialize_params();

        // if( private_nh.param<bool>( "deskewing", false ) ) {
        //     imu_sub = nh.subscribe( "/imu/data", 1, &PrefilteringComponent::imu_callback, this );
        // }
        if( this->declare_parameter<bool>( "deskewing", false ) ) {
            // TODO: QOS? sensordata qos?
            imu_sub = this->create_subscription<sensor_msgs::msg::Imu>( "/imu/data", rclcpp::QoS( 1 ),
                                                                        std::bind( &PrefilteringComponent::imu_callback, this,
                                                                                   std::placeholders::_1 ) );
        }

        // points_sub  = nh.subscribe( "/velodyne_points", 64, &PrefilteringComponent::cloud_callback, this );
        // points_pub  = nh.advertise<sensor_msgs::PointCloud2>( "/prefiltering/filtered_points", 32 );
        // colored_pub = nh.advertise<sensor_msgs::PointCloud2>( "/prefiltering/colored_points", 32 );
        points_sub  = this->create_subscription<sensor_msgs::msg::PointCloud2>( "/velodyne_points", rclcpp::QoS( 64 ),
                                                                               std::bind( &PrefilteringComponent::cloud_callback, this,
                                                                                           std::placeholders::_1 ) );
        points_pub  = this->create_publisher<sensor_msgs::msg::PointCloud2>( "/prefiltering/filtered_points", rclcpp::QoS( 32 ) );
        colored_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>( "/prefiltering/colored_points", rclcpp::QoS( 32 ) );
    }
    */

private:
    void initialize_params()
    {
        // TODO ROS2 parameter handling

        // std::string downsample_method     = private_nh.param<std::string>( "downsample_method", "VOXELGRID" );
        // double      downsample_resolution = private_nh.param<double>( "downsample_resolution", 0.1 );
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
        }

        // std::string outlier_removal_method = private_nh.param<std::string>( "outlier_removal_method", "STATISTICAL" );
        std::string outlier_removal_method = this->declare_parameter<std::string>( "outlier_removal_method", "STATISTICAL" );
        if( outlier_removal_method == "STATISTICAL" ) {
            // int    mean_k            = private_nh.param<int>( "statistical_mean_k", 20 );
            // double stddev_mul_thresh = private_nh.param<double>( "statistical_stddev", 1.0 );
            int    mean_k            = this->declare_parameter<int>( "statistical_mean_k", 20 );
            double stddev_mul_thresh = this->declare_parameter<double>( "statistical_stddev", 1.0 );
            std::cout << "outlier_removal: STATISTICAL " << mean_k << " - " << stddev_mul_thresh << std::endl;

            pcl::StatisticalOutlierRemoval<PointT>::Ptr sor( new pcl::StatisticalOutlierRemoval<PointT>() );
            sor->setMeanK( mean_k );
            sor->setStddevMulThresh( stddev_mul_thresh );
            outlier_removal_filter = sor;
        } else if( outlier_removal_method == "RADIUS" ) {
            // double radius        = private_nh.param<double>( "radius_radius", 0.8 );
            // int    min_neighbors = private_nh.param<int>( "radius_min_neighbors", 2 );
            double radius        = this->declare_parameter<double>( "radius_radius", 0.8 );
            int    min_neighbors = this->declare_parameter<int>( "radius_min_neighbors", 2 );
            std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;

            pcl::RadiusOutlierRemoval<PointT>::Ptr rad( new pcl::RadiusOutlierRemoval<PointT>() );
            rad->setRadiusSearch( radius );
            rad->setMinNeighborsInRadius( min_neighbors );
            outlier_removal_filter = rad;
        } else {
            std::cout << "outlier_removal: NONE" << std::endl;
        }

        // use_distance_filter  = private_nh.param<bool>( "use_distance_filter", true );
        // distance_near_thresh = private_nh.param<double>( "distance_near_thresh", 1.0 );
        // distance_far_thresh  = private_nh.param<double>( "distance_far_thresh", 100.0 );
        // base_link_frame = private_nh.param<std::string>( "base_link_frame", "" );

        use_distance_filter  = this->declare_parameter<bool>( "use_distance_filter", true );
        distance_near_thresh = this->declare_parameter<double>( "distance_near_thresh", 1.0 );
        distance_far_thresh  = this->declare_parameter<double>( "distance_far_thresh", 100.0 );

        // TODO set base_link_frame externally since it is not set in this node?
        base_link_frame = this->declare_parameter<std::string>( "base_link_frame", "" );
    }

    // void imu_callback( const sensor_msgs::ImuConstPtr& imu_msg ) { imu_queue.push_back( imu_msg ); }
    void imu_callback( sensor_msgs::msg::Imu::ConstSharedPtr imu_msg ) { imu_queue.push_back( imu_msg ); }

    // void cloud_callback( const pcl::PointCloud<PointT>& src_cloud_r )
    // TODO ROS2 pass unique_ptr for efficient intra process communication?
    void cloud_callback( sensor_msgs::msg::PointCloud2::ConstSharedPtr src_cloud_ros )
    {
        // Convert to pcl pointcloud from ros PointCloud2
        pcl::PointCloud<PointT>::Ptr src_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
        pcl::fromROSMsg( *src_cloud_ros, *src_cloud );
        // src_cloud_ros is already a shared ptr with msg type
        // pcl::PointCloud<PointT>::ConstPtr src_cloud = src_cloud_r.makeShared();
        // TODO: verify if statement
        if( src_cloud->empty() ) {
            return;
        }

        // auto src_cloud = deskewing( src_cloud_r );
        src_cloud = deskewing( src_cloud );

        // if base_link_frame is defined, transform the input cloud to the frame
        if( !base_link_frame.empty() ) {
            // if( !tf_listener.canTransform( base_link_frame, src_cloud->header.frame_id, ros::Time( 0 ) ) ) {
            if( !tf_buffer->canTransform( base_link_frame, src_cloud->header.frame_id, rclcpp::Time( 0 ) ) ) {
                std::cerr << "failed to find transform between " << base_link_frame << " and " << src_cloud->header.frame_id << std::endl;
            }

            // tf::StampedTransform transform;
            geometry_msgs::msg::TransformStamped transform;
            // tf_listener.waitForTransform( base_link_frame, src_cloud->header.frame_id, ros::Time( 0 ), ros::Duration( 2.0 ) );
            // tf_listener.lookupTransform( base_link_frame, src_cloud->header.frame_id, ros::Time( 0 ), transform );
            try {
                // lookupTransform contains a Duration as parameter
                transform = tf_buffer->lookupTransform( base_link_frame, src_cloud->header.frame_id, rclcpp::Time( 0 ),
                                                        rclcpp::Duration( 2, 0 ) );
            } catch( const tf2::TransformException& ex ) {
                RCLCPP_WARN( this->get_logger(), "Could not transform source frame %s to target frame %s: %s",
                             src_cloud->header.frame_id.c_str(), base_link_frame.c_str(), ex.what() );
                // TODO return here?
                return;
            }

            pcl::PointCloud<PointT>::Ptr transformed( new pcl::PointCloud<PointT>() );
            pcl_ros::transformPointCloud( *src_cloud, *transformed, transform );
            transformed->header.frame_id = base_link_frame;
            transformed->header.stamp    = src_cloud->header.stamp;
            src_cloud                    = transformed;
        }

        // TODO, verify
        pcl::PointCloud<PointT>::ConstPtr filtered = distance_filter( src_cloud );
        filtered                                   = downsample( filtered );
        filtered                                   = outlier_removal( filtered );
        sensor_msgs::msg::PointCloud2 filtered_ros;
        // TODO optimize out this conversion?
        pcl::toROSMsg( *filtered, filtered_ros );

        // points_pub.publish( *filtered );
        points_pub->publish( filtered_ros );
    }

    pcl::PointCloud<PointT>::ConstPtr downsample( const pcl::PointCloud<PointT>::ConstPtr& cloud ) const
    {
        if( !downsample_filter ) {
            return cloud;
        }

        pcl::PointCloud<PointT>::Ptr filtered( new pcl::PointCloud<PointT>() );
        downsample_filter->setInputCloud( cloud );
        downsample_filter->filter( *filtered );
        filtered->header = cloud->header;

        return filtered;
    }

    pcl::PointCloud<PointT>::ConstPtr outlier_removal( const pcl::PointCloud<PointT>::ConstPtr& cloud ) const
    {
        if( !outlier_removal_filter ) {
            return cloud;
        }

        pcl::PointCloud<PointT>::Ptr filtered( new pcl::PointCloud<PointT>() );
        outlier_removal_filter->setInputCloud( cloud );
        outlier_removal_filter->filter( *filtered );
        filtered->header = cloud->header;

        return filtered;
    }

    pcl::PointCloud<PointT>::ConstPtr distance_filter( const pcl::PointCloud<PointT>::ConstPtr& cloud ) const
    {
        pcl::PointCloud<PointT>::Ptr filtered( new pcl::PointCloud<PointT>() );
        filtered->reserve( cloud->size() );

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

    // pcl::PointCloud<PointT>::ConstPtr deskewing( const pcl::PointCloud<PointT>::ConstPtr& cloud )
    // For ROS2 pass shared_ptr and return shared_ptr instead of const shared_ptr, since we need to convert to sensor_msgs::
    pcl::PointCloud<PointT>::Ptr deskewing( const pcl::PointCloud<PointT>::Ptr& cloud )
    {
        // ros::Time stamp = pcl_conversions::fromPCL( cloud->header.stamp );
        rclcpp::Time stamp = pcl_conversions::fromPCL( cloud->header.stamp );
        if( imu_queue.empty() ) {
            return cloud;
        }

        // the color encodes the point number in the point sequence
        // if( colored_pub.getNumSubscribers() ) {
        if( colored_pub->get_subscription_count() ) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored( new pcl::PointCloud<pcl::PointXYZRGB>() );
            colored->header   = cloud->header;
            colored->is_dense = cloud->is_dense;
            colored->width    = cloud->width;
            colored->height   = cloud->height;
            colored->resize( cloud->size() );

            for( int i = 0; i < cloud->size(); i++ ) {
                double t                          = static_cast<double>( i ) / cloud->size();
                colored->at( i ).getVector4fMap() = cloud->at( i ).getVector4fMap();
                colored->at( i ).r                = 255 * t;
                colored->at( i ).g                = 128;
                colored->at( i ).b                = 255 * ( 1 - t );
            }
            // TODO optimize out this conversion?
            sensor_msgs::msg::PointCloud2 colored_ros;
            pcl::toROSMsg( *colored, colored_ros );
            // colored_pub.publish( *colored );
            colored_pub->publish( colored_ros );
        }

        // sensor_msgs::ImuConstPtr imu_msg = imu_queue.front();
        sensor_msgs::msg::Imu::ConstSharedPtr imu_msg = imu_queue.front();

        auto loc = imu_queue.begin();
        for( ; loc != imu_queue.end(); loc++ ) {
            imu_msg = ( *loc );
            // if( ( *loc )->header.stamp > stamp ) {
            if( rclcpp::Time( ( *loc )->header.stamp ) > stamp ) {
                break;
            }
        }

        imu_queue.erase( imu_queue.begin(), loc );

        Eigen::Vector3f ang_v( imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z );
        ang_v *= -1;

        pcl::PointCloud<PointT>::Ptr deskewed( new pcl::PointCloud<PointT>() );
        deskewed->header   = cloud->header;
        deskewed->is_dense = cloud->is_dense;
        deskewed->width    = cloud->width;
        deskewed->height   = cloud->height;
        deskewed->resize( cloud->size() );

        // double scan_period = private_nh.param<double>( "scan_period", 0.1 );
        // TODO potentially add class member for scan period if it doesnt change during runtime
        double scan_period = this->has_parameter( "scan_period" ) ? this->get_parameter( "scan_period" ).as_double()
                                                                  : this->declare_parameter<double>( "scan_period", 0.1 );
        for( int i = 0; i < cloud->size(); i++ ) {
            const auto& pt = cloud->at( i );

            // TODO: transform IMU data into the LIDAR frame
            double             delta_t = scan_period * static_cast<double>( i ) / cloud->size();
            Eigen::Quaternionf delta_q( 1, delta_t / 2.0 * ang_v[0], delta_t / 2.0 * ang_v[1], delta_t / 2.0 * ang_v[2] );
            Eigen::Vector3f    pt_ = delta_q.inverse() * pt.getVector3fMap();

            deskewed->at( i )                  = cloud->at( i );
            deskewed->at( i ).getVector3fMap() = pt_;
        }

        return deskewed;
    }

private:
    // ros::NodeHandle nh;
    // ros::NodeHandle private_nh;

    // ros::Subscriber                       imu_sub;
    // std::vector<sensor_msgs::ImuConstPtr> imu_queue;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>     imu_queue;

    // ros::Subscriber points_sub;
    // ros::Publisher  points_pub;
    // TODO: define pointcloud2 as unique ptr for efficient intra process communication?
    // https://docs.ros.org/en/foxy/Tutorials/Demos/Intra-Process-Communication.html
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    points_pub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pub;

    // tf::TransformListener tf_listener;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::Buffer>            tf_buffer;

    std::string base_link_frame;

    bool   use_distance_filter;
    double distance_near_thresh;
    double distance_far_thresh;

    pcl::Filter<PointT>::Ptr downsample_filter;
    pcl::Filter<PointT>::Ptr outlier_removal_filter;
};

}  // namespace hdl_graph_slam

// PLUGINLIB_EXPORT_CLASS( hdl_graph_slam::PrefilteringComponent, nodelet::Nodelet )
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE( hdl_graph_slam::PrefilteringComponent )
