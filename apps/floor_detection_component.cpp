// SPDX-License-Identifier: BSD-2-Clause

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/optional.hpp>
#include <iostream>
#include <memory>
#include <mrg_slam/ros_utils.hpp>
#include <mrg_slam_msgs/msg/floor_coeffs.hpp>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/search/impl/search.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mrg_slam {

class FloorDetectionComponent : public rclcpp::Node {
public:
    typedef pcl::PointXYZI PointT;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FloorDetectionComponent( const rclcpp::NodeOptions& options ) : Node( "floor_detection_component", options )
    {
        RCLCPP_INFO( get_logger(), "Initializing floor_detection_component ..." );

        initialize_params();

        points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>( "prefiltering/filtered_points", rclcpp::QoS( 256 ),
                                                                          std::bind( &FloorDetectionComponent::cloud_callback, this,
                                                                                     std::placeholders::_1 ) );

        floor_pub_ = create_publisher<mrg_slam_msgs::msg::FloorCoeffs>( "floor_detection/floor_coeffs", rclcpp::QoS( 32 ) );

        floor_filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>( "floor_detection/floor_filtered_points", rclcpp::QoS( 32 ) );
        floor_points_pub_   = create_publisher<sensor_msgs::msg::PointCloud2>( "floor_detection/floor_points", rclcpp::QoS( 32 ) );

        // Optionally print the all parameters declared in this node so far
        print_ros2_parameters( get_node_parameters_interface(), get_logger() );
    }

    virtual ~FloorDetectionComponent() {}

private:
    /**
     * @brief initialize ROS2 parameters
     */
    void initialize_params()
    {
        // Declare and set parameters
        declare_parameter<double>( "tilt_deg", 0.0 );          // approximate sensor tilt angle [deg]
        declare_parameter<double>( "sensor_height", 2.0 );     // approximate sensor height [m]
        declare_parameter<double>( "height_clip_range", 1.0 ); /* points with heights in [sensor_height - height_clip_range, sensor_height +
                                                                  height_clip_range] will be used for floor detection */
        declare_parameter<int>( "floor_pts_thresh", 512 );  // minimum number of support points of RANSAC to accept a detected floor plane
        declare_parameter<double>( "floor_normal_thresh_deg", 10.0 );   // verticality check thresold for the detected floor plane [deg]
        declare_parameter<bool>( "enable_normal_filtering", true );     // points with "non-"vertical normals will be filtered before RANSAC
        declare_parameter<double>( "normal_filter_thresh_deg", 20.0 );  // "non-"verticality check threshold [deg]
    }

    /**
     * @brief callback for point clouds
     * @param cloud_msg  point cloud msg
     */
    void cloud_callback( const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg )
    {
        pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT>() );
        pcl::fromROSMsg( *cloud_msg, *cloud );

        if( cloud->empty() ) {
            RCLCPP_WARN_STREAM_THROTTLE( get_logger(), *get_clock(), 1000, "Empty point cloud" );
            return;
        }

        // floor detection
        boost::optional<Eigen::Vector4f> floor = detect( cloud );

        // publish the detected floor coefficients
        mrg_slam_msgs::msg::FloorCoeffs coeffs;
        coeffs.header = cloud_msg->header;
        if( floor ) {
            coeffs.coeffs.resize( 4 );
            for( int i = 0; i < 4; i++ ) {
                coeffs.coeffs[i] = ( *floor )[i];
            }
        }

        floor_pub_->publish( coeffs );
    }

    /**
     * @brief detect the floor plane from a point cloud
     * @param cloud  input cloud
     * @return detected floor plane coefficients
     */
    boost::optional<Eigen::Vector4f> detect( const pcl::PointCloud<PointT>::Ptr& cloud ) const
    {
        // compensate the tilt rotation
        Eigen::Matrix4f tilt_matrix = Eigen::Matrix4f::Identity();
        tilt_matrix.topLeftCorner( 3, 3 ) =
            Eigen::AngleAxisf( get_parameter( "tilt_deg" ).as_double() * M_PI / 180.0f, Eigen::Vector3f::UnitY() ).toRotationMatrix();

        // filtering before RANSAC (height and normal filtering)
        pcl::PointCloud<PointT>::Ptr filtered( new pcl::PointCloud<PointT> );
        pcl::transformPointCloud( *cloud, *filtered, tilt_matrix );
        double sensor_height     = get_parameter( "sensor_height" ).as_double();
        double height_clip_range = get_parameter( "height_clip_range" ).as_double();
        filtered                 = plane_clip( filtered, Eigen::Vector4f( 0.0f, 0.0f, 1.0f, sensor_height + height_clip_range ), false );
        filtered                 = plane_clip( filtered, Eigen::Vector4f( 0.0f, 0.0f, 1.0f, sensor_height - height_clip_range ), true );

        if( filtered->empty() ) {
            RCLCPP_WARN_STREAM_THROTTLE( get_logger(), *get_clock(), 1000,
                                         "No points after height clipping. Check sensor_height and height_clip_range params" );
            return boost::none;
        }

        if( get_parameter( "use_normal_filtering" ).as_bool() ) {
            filtered = normal_filtering( filtered );
        }

        pcl::transformPointCloud( *filtered, *filtered, static_cast<Eigen::Matrix4f>( tilt_matrix.inverse() ) );

        if( floor_filtered_pub_->get_subscription_count() ) {
            filtered->header = cloud->header;
            sensor_msgs::msg::PointCloud2 filtered_ros;
            pcl::toROSMsg( *filtered, filtered_ros );
            floor_filtered_pub_->publish( filtered_ros );
        }

        // too few points for RANSAC
        if( (int)filtered->size() < get_parameter( "floor_pts_thresh" ).as_int() ) {
            return boost::none;
        }

        // RANSAC
        pcl::SampleConsensusModelPlane<PointT>::Ptr model_p( new pcl::SampleConsensusModelPlane<PointT>( filtered ) );
        pcl::RandomSampleConsensus<PointT>          ransac( model_p );
        ransac.setDistanceThreshold( 0.1 );
        ransac.computeModel();

        pcl::PointIndices::Ptr inliers( new pcl::PointIndices );
        ransac.getInliers( inliers->indices );

        // too few inliers
        if( (int)inliers->indices.size() < get_parameter( "floor_pts_thresh" ).as_int() ) {
            return boost::none;
        }

        // verticality check of the detected floor's normal
        Eigen::Vector4f reference = tilt_matrix.inverse() * Eigen::Vector4f::UnitZ();

        Eigen::VectorXf coeffs;
        ransac.getModelCoefficients( coeffs );

        double dot = coeffs.head<3>().dot( reference.head<3>() );
        if( std::abs( dot ) < std::cos( get_parameter( "floor_normal_thresh_deg" ).as_double() * M_PI / 180.0 ) ) {
            // the normal is not vertical
            return boost::none;
        }

        // make the normal upward
        if( coeffs.head<3>().dot( Eigen::Vector3f::UnitZ() ) < 0.0f ) {
            coeffs *= -1.0f;
        }

        if( floor_points_pub_->get_subscription_count() ) {
            pcl::PointCloud<PointT>::Ptr inlier_cloud( new pcl::PointCloud<PointT> );
            pcl::ExtractIndices<PointT>  extract;
            extract.setInputCloud( filtered );
            extract.setIndices( inliers );
            extract.filter( *inlier_cloud );
            inlier_cloud->header = cloud->header;

            sensor_msgs::msg::PointCloud2 inlier_cloud_ros;
            pcl::toROSMsg( *inlier_cloud, inlier_cloud_ros );
            floor_points_pub_->publish( inlier_cloud_ros );
        }

        return Eigen::Vector4f( coeffs );
    }

    /**
     * @brief plane_clip
     * @param src_cloud
     * @param plane
     * @param negative
     * @return
     */
    pcl::PointCloud<PointT>::Ptr plane_clip( const pcl::PointCloud<PointT>::Ptr& src_cloud, const Eigen::Vector4f& plane,
                                             bool negative ) const
    {
        pcl::PlaneClipper3D<PointT> clipper( plane );
        pcl::PointIndices::Ptr      indices( new pcl::PointIndices );

        clipper.clipPointCloud3D( *src_cloud, indices->indices );

        pcl::PointCloud<PointT>::Ptr dst_cloud( new pcl::PointCloud<PointT> );

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud( src_cloud );
        extract.setIndices( indices );
        extract.setNegative( negative );
        extract.filter( *dst_cloud );

        return dst_cloud;
    }

    /**
     * @brief filter points with non-vertical normals
     * @param cloud  input cloud
     * @return filtered cloud
     */
    pcl::PointCloud<PointT>::Ptr normal_filtering( const pcl::PointCloud<PointT>::Ptr& cloud ) const
    {
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        ne.setInputCloud( cloud );

        pcl::search::KdTree<PointT>::Ptr tree( new pcl::search::KdTree<PointT> );
        ne.setSearchMethod( tree );

        pcl::PointCloud<pcl::Normal>::Ptr normals( new pcl::PointCloud<pcl::Normal> );
        ne.setKSearch( 10 );
        ne.setViewPoint( 0.0f, 0.0f, get_parameter( "sensor_height" ).as_double() );
        ne.compute( *normals );

        pcl::PointCloud<PointT>::Ptr filtered( new pcl::PointCloud<PointT> );
        filtered->reserve( cloud->size() );

        for( int i = 0; i < (int)cloud->size(); i++ ) {
            float dot = normals->at( i ).getNormalVector3fMap().normalized().dot( Eigen::Vector3f::UnitZ() );
            if( std::abs( dot ) > std::cos( get_parameter( "normal_filter_thresh_deg" ).as_double() * M_PI / 180.0 ) ) {
                filtered->push_back( cloud->at( i ) );
            }
        }

        filtered->width    = filtered->size();
        filtered->height   = 1;
        filtered->is_dense = false;

        return filtered;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;

    rclcpp::Publisher<mrg_slam_msgs::msg::FloorCoeffs>::SharedPtr floor_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   floor_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   floor_filtered_pub_;
};

}  // namespace mrg_slam


// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE( mrg_slam::FloorDetectionComponent )
