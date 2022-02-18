// SPDX-License-Identifier: BSD-2-Clause

#include <hdl_graph_slam/gps_processor.hpp>

#include <geodesy/utm.h>
#include <geodesy/wgs84.h>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>


namespace hdl_graph_slam {

void GpsProcessor::onInit(ros::NodeHandle &nh, ros::NodeHandle &mt_nh, ros::NodeHandle &private_nh_) {
    private_nh = &private_nh_;
    nmea_parser.reset(new NmeaSentenceParser());

    gps_time_offset = private_nh->param<double>("gps_time_offset", 0.0);
    gps_edge_stddev_xy = private_nh->param<double>("gps_edge_stddev_xy", 10000.0);
    gps_edge_stddev_z = private_nh->param<double>("gps_edge_stddev_z", 10.0);

    if(private_nh->param<bool>("enable_gps", true)) {
      gps_sub = mt_nh.subscribe("/gps/geopoint", 1024, &GpsProcessor::gps_callback, this);
      nmea_sub = mt_nh.subscribe("/gpsimu_driver/nmea_sentence", 1024, &GpsProcessor::nmea_callback, this);
      navsat_sub = mt_nh.subscribe("/gps/navsat", 1024, &GpsProcessor::navsat_callback, this);
    }
}


void GpsProcessor::nmea_callback(const nmea_msgs::SentenceConstPtr& nmea_msg) {
    GPRMC grmc = nmea_parser->parse(nmea_msg->sentence);

    if(grmc.status != 'A') {
        return;
    }

    geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
    gps_msg->header = nmea_msg->header;
    gps_msg->position.latitude = grmc.latitude;
    gps_msg->position.longitude = grmc.longitude;
    gps_msg->position.altitude = NAN;

    gps_callback(gps_msg);
}

void GpsProcessor::navsat_callback(const sensor_msgs::NavSatFixConstPtr& navsat_msg) {
    geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
    gps_msg->header = navsat_msg->header;
    gps_msg->position.latitude = navsat_msg->latitude;
    gps_msg->position.longitude = navsat_msg->longitude;
    gps_msg->position.altitude = navsat_msg->altitude;
    gps_callback(gps_msg);
}

/**
 * @brief received gps data is added to #gps_queue
 * @param gps_msg
 */
void GpsProcessor::gps_callback(const geographic_msgs::GeoPointStampedPtr& gps_msg) {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);
    gps_msg->header.stamp += ros::Duration(gps_time_offset);
    gps_queue.push_back(gps_msg);
}

/**
 * @brief
 * @return
 */
bool GpsProcessor::flush(std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes) {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);

    if(keyframes.empty() || gps_queue.empty()) {
        return false;
    }

    bool updated = false;
    auto gps_cursor = gps_queue.begin();

    for(auto& keyframe : keyframes) {
        if(keyframe->stamp > gps_queue.back()->header.stamp) {
        break;
        }

        if(keyframe->stamp < (*gps_cursor)->header.stamp || keyframe->utm_coord) {
        continue;
        }

        // find the gps data which is closest to the keyframe
        auto closest_gps = gps_cursor;
        for(auto gps = gps_cursor; gps != gps_queue.end(); gps++) {
        auto dt = ((*closest_gps)->header.stamp - keyframe->stamp).toSec();
        auto dt2 = ((*gps)->header.stamp - keyframe->stamp).toSec();
        if(std::abs(dt) < std::abs(dt2)) {
            break;
        }

        closest_gps = gps;
        }

        // if the time residual between the gps and keyframe is too large, skip it
        gps_cursor = closest_gps;
        if(0.2 < std::abs(((*closest_gps)->header.stamp - keyframe->stamp).toSec())) {
            continue;
        }

        // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
        geodesy::UTMPoint utm;
        geodesy::fromMsg((*closest_gps)->position, utm);
        Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);

        // the first gps data position will be the origin of the map
        if(!zero_utm_vec) {
            zero_utm_vec = xyz;
        }
        xyz -= (*zero_utm_vec);

        keyframe->utm_coord = xyz;

        g2o::OptimizableGraph::Edge* edge;
        if(std::isnan(xyz.z())) {
            Eigen::Matrix2d information_matrix = Eigen::Matrix2d::Identity() / gps_edge_stddev_xy;
            edge = graph_slam->add_se3_prior_xy_edge(keyframe->node, xyz.head<2>(), information_matrix);
        } else {
            Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
            information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy;
            information_matrix(2, 2) /= gps_edge_stddev_z;
            edge = graph_slam->add_se3_prior_xyz_edge(keyframe->node, xyz, information_matrix);
        }
        graph_slam->add_robust_kernel(edge, private_nh->param<std::string>("gps_edge_robust_kernel", "NONE"), private_nh->param<double>("gps_edge_robust_kernel_size", 1.0));

        updated = true;
    }

    auto remove_loc = std::upper_bound(gps_queue.begin(), gps_queue.end(), keyframes.back()->stamp, [=](const ros::Time& stamp, const geographic_msgs::GeoPointStampedConstPtr& geopoint) { return stamp < geopoint->header.stamp; });
    gps_queue.erase(gps_queue.begin(), remove_loc);
    return updated;
}

} // namespace hdl_graph_slam