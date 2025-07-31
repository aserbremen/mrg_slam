// SPDX-License-Identifier: BSD-2-Clause

#ifndef GPS_PROCESSOR_HPP
#define GPS_PROCESSOR_HPP

#include <GeographicLib/LocalCartesian.hpp>
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


namespace mrg_slam {

class GpsProcessor {
public:
    GpsProcessor() {}

    void onInit( rclcpp::Node::SharedPtr node );

    void nmea_callback( const nmea_msgs::msg::Sentence::SharedPtr nmea_msg );
    void navsat_callback( const sensor_msgs::msg::NavSatFix::SharedPtr navsat_msg );
    void gps_callback( geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg );

    bool flush( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes );

    const boost::optional<Eigen::Vector3d>                &zero_utm() const { return zero_utm_vec_; }
    const boost::optional<Eigen::Vector3d>                &zero_enu() const { return zero_enu_vec_; }
    const boost::optional<geographic_msgs::msg::GeoPoint> &enu_origin() const { return gps_enu_origin_; }

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr              nmea_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr           navsat_sub_;
    rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr gps_sub_;

    std::unique_ptr<NmeaSentenceParser> nmea_parser_;

    boost::optional<Eigen::Vector3d>                             zero_utm_vec_;
    boost::optional<Eigen::Vector3d>                             zero_enu_vec_;
    boost::optional<geographic_msgs::msg::GeoPoint>              gps_enu_origin_;
    std::mutex                                                   gps_queue_mutex_;
    std::deque<geographic_msgs::msg::GeoPointStamped::SharedPtr> gps_queue_;
    GeographicLib::LocalCartesian                                local_cartesian_;
};

}  // namespace mrg_slam

#endif  // GPS_PROCESSOR_HPP