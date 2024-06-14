#ifndef GRAPH_DATA_BASE_HPP
#define GRAPH_DATA_BASE_HPP

#include <unordered_map>
#include <unordered_set>
#include <vector>
// boost
#include <boost/functional/hash.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
// g2o
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
// mrg_slam
#include <mrg_slam/edge.hpp>
#include <mrg_slam/graph_slam.hpp>
#include <mrg_slam/keyframe.hpp>
// pcl
#include <pcl/point_cloud.h>
// ROS2
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mrg_slam {

class GraphDataBase {
public:
    // Not needed for now
    // GraphDataBase();
    typedef pcl::PointXYZI PointT;

    // Some other classes use this function to initialize the GraphDataBase, we stick to the same signature
    void onInit( rclcpp::Node::SharedPtr _node, std::shared_ptr<GraphSLAM> _graph_slam );

    void add_odom_keyframe_to_queue( const builtin_interfaces::msg::Time& stamp, const Eigen::Isometry3d& odom, double accum_distance,
                                     pcl::PointCloud<PointT>::ConstPtr cloud, sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg );


private:
    std::shared_ptr<GraphSLAM> graph_slam;

    std::string own_name;
    int         odom_keyframe_counter;

    std::mutex                keyframe_queue_mutex;
    std::deque<KeyFrame::Ptr> keyframe_queue;

    // keyframes and edges in the graph in order of addition
    std::vector<KeyFrame::Ptr> keyframes;
    std::vector<Edge::Ptr>     edges;

    // keyframes and edges successfully added to the graph
    std::unordered_map<boost::uuids::uuid, KeyFrame::Ptr, boost::hash<boost::uuids::uuid>> keyframe_map;
    std::unordered_set<boost::uuids::uuid, boost::hash<boost::uuids::uuid>>                edge_uuid_set;
    std::unordered_set<boost::uuids::uuid, boost::hash<boost::uuids::uuid>> edge_ignore_uuids;  // TODO test if this is still necessary

    // Anchor node/keyframe and edge which are treated differently
    g2o::VertexSE3* anchor_node;
    g2o::EdgeSE3*   anchor_edge_g2o;
    KeyFrame::Ptr   anchor_kf;
    Edge::Ptr       anchor_edge_ptr;

    // keyframes to be added to the graph
    std::deque<KeyFrame::Ptr> new_keyframes;
    KeyFrame::ConstPtr        prev_robot_keyframe;

    // keyframe and edges container for loading graph from a previously save_graph call
    std::vector<KeyFrame::Ptr> loaded_keyframes;
    std::vector<Edge::Ptr>     loaded_edges;

    // unique id generators
    boost::uuids::random_generator_pure uuid_generator;
    boost::uuids::string_generator      uuid_from_string_generator;
};

}  // namespace mrg_slam


#endif  // GRAPH_DATA_BASE_HPP