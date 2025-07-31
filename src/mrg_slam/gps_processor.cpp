// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/edge_se3.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>

#include <builtin_interfaces/msg/duration.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <mrg_slam/gps_processor.hpp>
#include <rclcpp/macros.hpp>

using std::placeholders::_1;

namespace mrg_slam {

void
GpsProcessor::onInit( rclcpp::Node::SharedPtr node )
{
    node_ = node;
    nmea_parser_.reset( new NmeaSentenceParser() );

    if( node_->get_parameter( "gps_use_enu" ).as_bool() && !node_->get_parameter( "gps_enu_origin_from_msg" ).as_bool() ) {
        auto gps_enu_origin_vec = node_->get_parameter( "gps_enu_origin" ).as_double_array();
        if( gps_enu_origin_vec.size() == 3 ) {
            gps_enu_origin_ = geodesy::toMsg( gps_enu_origin_vec[0], gps_enu_origin_vec[1], gps_enu_origin_vec[2] );
            local_cartesian_.Reset( gps_enu_origin_->latitude, gps_enu_origin_->longitude, gps_enu_origin_->altitude );
        } else {
            std::cerr << "warning: gps_enu_origin not set to a proper value ([latitude, longitude, altitude]). "
                      << "ENU frame origin will be determined from the GPS position." << std::endl;
        }
    }

    if( node_->get_parameter( "enable_gps" ).as_bool() ) {
        nmea_sub_   = node_->create_subscription<nmea_msgs::msg::Sentence>( "gpsimu_driver/nmea_sentence", rclcpp::QoS( 1024 ),
                                                                            std::bind( &GpsProcessor::nmea_callback, this, _1 ) );
        navsat_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>( "gps/navsat", rclcpp::QoS( 1024 ),
                                                                               std::bind( &GpsProcessor::navsat_callback, this, _1 ) );
        gps_sub_    = node_->create_subscription<geographic_msgs::msg::GeoPointStamped>( "gps/geopoint", rclcpp::QoS( 1024 ),
                                                                                         std::bind( &GpsProcessor::gps_callback, this, _1 ) );
    }
}


void
GpsProcessor::nmea_callback( const nmea_msgs::msg::Sentence::SharedPtr nmea_msg )
{
    GPRMC grmc = nmea_parser_->parse( nmea_msg->sentence );

    if( grmc.status != 'A' ) {
        return;
    }

    geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg( new geographic_msgs::msg::GeoPointStamped() );
    gps_msg->header             = nmea_msg->header;
    gps_msg->position.latitude  = grmc.latitude;
    gps_msg->position.longitude = grmc.longitude;
    gps_msg->position.altitude  = NAN;

    gps_callback( gps_msg );
}


void
GpsProcessor::navsat_callback( const sensor_msgs::msg::NavSatFix::SharedPtr navsat_msg )
{
    geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg( new geographic_msgs::msg::GeoPointStamped() );
    gps_msg->header             = navsat_msg->header;
    gps_msg->position.latitude  = navsat_msg->latitude;
    gps_msg->position.longitude = navsat_msg->longitude;
    gps_msg->position.altitude  = navsat_msg->altitude;
    gps_callback( gps_msg );
}

/**
 * @brief received gps data is added to #gps_queue
 * @param gps_msg
 */
void
GpsProcessor::gps_callback( geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg )
{
    std::lock_guard<std::mutex> lock( gps_queue_mutex_ );
    gps_msg->header.stamp = ( rclcpp::Time( gps_msg->header.stamp )
                              + rclcpp::Duration::from_seconds( node_->get_parameter( "gps_time_offset" ).as_double() ) )
                                .operator builtin_interfaces::msg::Time();
    gps_queue_.push_back( gps_msg );
}

/**
 * @brief
 * @return
 */
bool
GpsProcessor::flush( std::shared_ptr<GraphSLAM>& graph_slam, const std::vector<KeyFrame::Ptr>& keyframes )
{
    std::lock_guard<std::mutex> lock( gps_queue_mutex_ );

    if( keyframes.empty() || gps_queue_.empty() ) {
        return false;
    }

    bool updated    = false;
    auto gps_cursor = gps_queue_.begin();

    // Get parameters before the loop to avoid repeated calls
    bool        gps_use_enu                 = node_->get_parameter( "gps_use_enu" ).as_bool();
    double      gps_edge_stddev_xy          = node_->get_parameter( "gps_edge_stddev_xy" ).as_double();
    double      gps_edge_stddev_z           = node_->get_parameter( "gps_edge_stddev_z" ).as_double();
    std::string gps_edge_robust_kernel      = node_->get_parameter( "gps_edge_robust_kernel" ).as_string();
    double      gps_edge_robust_kernel_size = node_->get_parameter( "gps_edge_robust_kernel_size" ).as_double();

    for( auto& keyframe : keyframes ) {
        // As there are no operators implemented for builtin_interfaces::msg::Time we use rclcpp::Time instances instead in this function
        if( rclcpp::Time( keyframe->stamp ) > rclcpp::Time( gps_queue_.back()->header.stamp ) ) {
            break;
        }

        if( rclcpp::Time( keyframe->stamp ) < rclcpp::Time( ( *gps_cursor )->header.stamp ) || keyframe->utm_coord ) {
            continue;
        }

        // find the gps data which is closest to the keyframe
        auto closest_gps = gps_cursor;
        for( auto gps = gps_cursor; gps != gps_queue_.end(); gps++ ) {
            auto dt  = ( rclcpp::Time( ( *closest_gps )->header.stamp ) - rclcpp::Time( keyframe->stamp ) ).seconds();
            auto dt2 = ( rclcpp::Time( ( *gps )->header.stamp ) - rclcpp::Time( keyframe->stamp ) ).seconds();
            if( std::abs( dt ) < std::abs( dt2 ) ) {
                break;
            }

            closest_gps = gps;
        }

        // if the time residual between the gps and keyframe is too large, skip it
        gps_cursor = closest_gps;
        if( 0.2 < std::abs( ( rclcpp::Time( ( *closest_gps )->header.stamp ) - rclcpp::Time( keyframe->stamp ) ).seconds() ) ) {
            continue;
        }

        Eigen::Vector3d xyz;
        if( gps_use_enu ) {
            // ENU mode
            auto&& nav_pos = ( *closest_gps )->position;
            if( !gps_enu_origin_ ) {
                gps_enu_origin_ = geodesy::toMsg( std::round( nav_pos.latitude * 1e5 ) / 1e5, std::round( nav_pos.longitude * 1e5 ) / 1e5,
                                                  std::floor( nav_pos.altitude ) );
                local_cartesian_.Reset( gps_enu_origin_->latitude, gps_enu_origin_->longitude, gps_enu_origin_->altitude );
            }
            double x, y, z;
            local_cartesian_.Forward( nav_pos.latitude, nav_pos.longitude, nav_pos.altitude, x, y, z );
            xyz = { x, y, z };
            if( !zero_enu_vec_ ) {
                zero_enu_vec_ = xyz;
            }
            xyz -= ( *zero_enu_vec_ );
        } else {
            // UTM mode
            // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
            geodesy::UTMPoint utm;
            geodesy::fromMsg( ( *closest_gps )->position, utm );
            xyz = { utm.easting, utm.northing, utm.altitude };

            // the first gps data position will be the origin of the map
            if( !zero_utm_vec_ ) {
                zero_utm_vec_ = xyz;
            }
            xyz -= ( *zero_utm_vec_ );
        }

        keyframe->utm_coord = xyz;

        g2o::OptimizableGraph::Edge* edge;
        if( std::isnan( xyz.z() ) ) {
            Eigen::Matrix2d information_matrix = Eigen::Matrix2d::Identity() / gps_edge_stddev_xy;
            edge                               = graph_slam->add_se3_prior_xy_edge( keyframe->node, xyz.head<2>(), information_matrix );
        } else {
            Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
            information_matrix.block<2, 2>( 0, 0 ) /= gps_edge_stddev_xy;
            information_matrix( 2, 2 ) /= gps_edge_stddev_z;
            edge = graph_slam->add_se3_prior_xyz_edge( keyframe->node, xyz, information_matrix );
        }
        graph_slam->add_robust_kernel( edge, gps_edge_robust_kernel, gps_edge_robust_kernel_size );

        updated = true;
    }

    auto remove_loc = std::upper_bound( gps_queue_.begin(), gps_queue_.end(), rclcpp::Time( keyframes.back()->stamp ),
                                        [=]( const rclcpp::Time&                                          stamp,
                                             const geographic_msgs::msg::GeoPointStamped::ConstSharedPtr& geopoint ) {
                                            return stamp < rclcpp::Time( geopoint->header.stamp );
                                        } );
    gps_queue_.erase( gps_queue_.begin(), remove_loc );
    return updated;
}

}  // namespace mrg_slam