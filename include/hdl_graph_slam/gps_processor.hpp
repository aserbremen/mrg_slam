// SPDX-License-Identifier: BSD-2-Clause

#ifndef GPS_PROCESSOR_HPP
#define GPS_PROCESSOR_HPP

#include <mutex>
#include <deque>
#include <boost/optional.hpp>

#include <ros/ros.h>

#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPointStamped.h>

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/nmea_sentence_parser.hpp>


namespace hdl_graph_slam {

class GpsProcessor {
public:
    GpsProcessor() : private_nh(nullptr) {}
    
    void onInit(ros::NodeHandle &nh, ros::NodeHandle &mt_nh, ros::NodeHandle &private_nh);

    void nmea_callback(const nmea_msgs::SentenceConstPtr& nmea_msg);
    void navsat_callback(const sensor_msgs::NavSatFixConstPtr& navsat_msg);
    void gps_callback(const geographic_msgs::GeoPointStampedPtr& gps_msg);

    bool flush(std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes);

    const boost::optional<Eigen::Vector3d>& zero_utm() const {
        return zero_utm_vec;
    }

private:
    ros::NodeHandle *private_nh;
    ros::Subscriber gps_sub;
    ros::Subscriber nmea_sub;
    ros::Subscriber navsat_sub;

    std::unique_ptr<NmeaSentenceParser> nmea_parser;

    double gps_time_offset;
    double gps_edge_stddev_xy;
    double gps_edge_stddev_z;
    boost::optional<Eigen::Vector3d> zero_utm_vec;
    std::mutex gps_queue_mutex;
    std::deque<geographic_msgs::GeoPointStampedConstPtr> gps_queue;
};

} // namespace hdl_graph_slam

#endif // GPS_PROCESSOR_HPP