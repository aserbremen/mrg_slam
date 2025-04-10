// SPDX-License-Identifier: BSD-2-Clause

#ifndef GPS_PROCESSOR_HPP
#define GPS_PROCESSOR_HPP

#include <boost/optional.hpp>
#include <deque>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <mrg_slam/graph_slam.hpp>
#include <mrg_slam/keyframe.hpp>
#include <mrg_slam/nmea_sentence_parser.hpp>
#include <mutex>
#include <nmea_msgs/msg/sentence.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <GeographicLib/LocalCartesian.hpp>


namespace mrg_slam {

class GpsProcessor {
public:
    GpsProcessor() {}

    void onInit( rclcpp::Node::SharedPtr _node );

    void nmea_callback( const nmea_msgs::msg::Sentence::SharedPtr nmea_msg );
    void navsat_callback( const sensor_msgs::msg::NavSatFix::SharedPtr navsat_msg );
    void gps_callback( geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg );

    bool flush( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes );

    const boost::optional<Eigen::Vector3d> &zero_utm() const { return zero_utm_vec; }
    const boost::optional<Eigen::Vector3d> &zero_enu() const { return zero_enu_vec; }
    const boost::optional<geographic_msgs::msg::GeoPoint> &enu_origin() const { return gps_enu_origin; }

private:
    rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr              nmea_sub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr           navsat_sub;
    rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr gps_sub;

    std::unique_ptr<NmeaSentenceParser> nmea_parser;

    bool                                                         enable_gps;
    double                                                       gps_time_offset;
    double                                                       gps_edge_stddev_xy;
    double                                                       gps_edge_stddev_z;
    std::string                                                  gps_edge_robust_kernel;
    double                                                       gps_edge_robust_kernel_size;
    bool                                                         gps_use_enu;
    bool                                                         gps_enu_origin_from_msg;
    boost::optional<Eigen::Vector3d>                             zero_utm_vec;
    boost::optional<Eigen::Vector3d>                             zero_enu_vec;
    boost::optional<geographic_msgs::msg::GeoPoint>              gps_enu_origin;
    std::mutex                                                   gps_queue_mutex;
    std::deque<geographic_msgs::msg::GeoPointStamped::SharedPtr> gps_queue;
    GeographicLib::LocalCartesian                                local_cartesian;
};

}  // namespace mrg_slam

#endif  // GPS_PROCESSOR_HPP