#ifndef GRAPH_DATA_BASE_HPP
#define GRAPH_DATA_BASE_HPP

#include <memory>
#include <mutex>
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
#include <mrg_slam/information_matrix_calculator.hpp>
#include <mrg_slam/keyframe.hpp>
#include <mrg_slam/loop_detector.hpp>
#include <mrg_slam/ros_time_hash.hpp>
#include <mrg_slam_msgs/msg/graph_ros.hpp>
// mrg_slam pcl
#include <pcl/fill_ground_plane.hpp>
// pcl
#include <pcl/point_cloud.h>

// ROS2
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mrg_slam {

class GraphDatabase {
public:
    typedef pcl::PointXYZI PointT;

    GraphDatabase( rclcpp::Node::SharedPtr node, std::shared_ptr<GraphSLAM> graph_slam );

    bool empty() const { return keyframes_.empty() && new_keyframes_.empty(); }

    void add_odom_keyframe( const builtin_interfaces::msg::Time& stamp, const Eigen::Isometry3d& odom, double accum_distance,
                            pcl::PointCloud<PointT>::ConstPtr cloud, sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg );

    /**
     * @brief Process keyframes in the keyframe queue and create edges between them
     * @param odom2map current odometry to map transform
     * @return true if at least one keyframe was processed
     */
    bool flush_keyframe_queue( const Eigen::Isometry3d& odom2map );

    /**
     * @brief Adds static keyframes to the queue
     * @param keyframes vector of keyframes from the static map provider
     */
    void add_static_keyframes( const std::vector<mrg_slam_msgs::msg::KeyFrameRos>& keyframes_ros );

    /**
     * @brief Process keyframes in the static keyframe queue and add them to the graph
     *
     * @return true if at least one static keyframe was processed
     */
    bool flush_static_keyframe_queue();

    /**
     * @brief Adds a GraphRos message to the graph queue for later processing
     * @param graph_ros
     */
    void add_graph_ros( const mrg_slam_msgs::msg::GraphRos& graph_ros );

    /**
     * @brief Flushes the graph queue and adds the keyframes and edges to the graph
     * @param others_prev_robot_keyframes map of the last keyframes of other robots
     * @param last_loop_map map of the last loop closures which might be updated due to loops already present in the graph
     * @return true if at least one unique keyframe or edge was added to the graph
     */
    bool flush_graph_queue( std::unordered_map<std::string, std::pair<KeyFrame::ConstPtr, Eigen::Isometry3d>>& others_prev_robot_keyframes,
                            LoopManager::Ptr                                                                   loop_manager );

    /**
     * @brief Loads the graph from a directory, which has previously been saved with save_graph service
     * @param directory
     * @return true if the graph was successfully loaded
     */
    bool load_graph( const std::string& directory );

    /**
     * @brief Flushes the loaded graph and adds the unique keyframes and edges to the graph
     * @return true if at least one unique keyframe or edge was added to the graph
     */
    bool flush_loaded_graph( LoopManager::Ptr loop_manager );

    /**
     * @brief Adds SE3 edges between two keyframes for which a loop was detected. Adds the new_keyframes checked for loop closures to
     * keyframes vector
     * @param loops vector of loops
     */
    void insert_loops( const std::vector<Loop::Ptr>& loops );

    /**
     * @brief Saves the poses of the keyframes to a file if the result_dir parameter is not empty
     */
    void save_keyframe_poses();

    const std::deque<KeyFrame::Ptr>&                                               get_keyframe_queue() const { return keyframe_queue_; }
    const std::vector<KeyFrame::Ptr>&                                              get_keyframes() const { return keyframes_; }
    const std::deque<KeyFrame::Ptr>&                                               get_new_keyframes() const { return new_keyframes_; }
    const std::vector<Edge::Ptr>&                                                  get_edges() const { return edges_; }
    const std::unordered_set<boost::uuids::uuid, boost::hash<boost::uuids::uuid>>& get_edge_ignore_uuids() const { return edge_uuids_; }
    KeyFrame::ConstPtr get_prev_robot_keyframe() const { return prev_robot_keyframe_; }
    const std::unordered_map<builtin_interfaces::msg::Time, KeyFrame::Ptr, RosTimeHash>& get_keyframe_hash() const
    {
        return keyframe_hash_;
    }
    const g2o::VertexSE3*     get_anchor_node() const { return anchor_node_; }
    const g2o::EdgeSE3*       get_anchor_edge_g2o() const { return anchor_edge_g2o_; }
    const boost::uuids::uuid& get_slam_uuid() const { return slam_uuid_; }

private:
    rclcpp::Node::SharedPtr node_;

    std::shared_ptr<GraphSLAM>                   graph_slam_;
    std::unique_ptr<InformationMatrixCalculator> inf_calculator_;

    int odom_keyframe_counter_;

    std::mutex                keyframe_queue_mutex_;  // keyframes to be added to the graph
    std::deque<KeyFrame::Ptr> keyframe_queue_;

    std::mutex                static_keyframe_queue_mutex_;  // keyframes from the static keyframe provider
    std::deque<KeyFrame::Ptr> static_keyframe_queue_;

    std::deque<KeyFrame::Ptr> new_keyframes_;  // keyframes to be checked for loop closure
    KeyFrame::ConstPtr        prev_robot_keyframe_;

    // keyframe and edges container for loading graph from a previously save_graph call
    std::mutex                 loaded_graph_mutex_;
    std::vector<KeyFrame::Ptr> loaded_keyframes_;
    std::vector<Edge::Ptr>     loaded_edges_;

    // keyframes and edges in the graph in order of addition
    std::vector<KeyFrame::Ptr> keyframes_;
    std::vector<Edge::Ptr>     edges_;

    std::deque<mrg_slam_msgs::msg::GraphRos::ConstSharedPtr> graph_queue_;

    // keyframes and edges successfully added to the graph
    std::unordered_map<builtin_interfaces::msg::Time, KeyFrame::Ptr, RosTimeHash>          keyframe_hash_;
    std::unordered_map<boost::uuids::uuid, KeyFrame::Ptr, boost::hash<boost::uuids::uuid>> uuid_keyframe_map_;
    std::unordered_set<boost::uuids::uuid, boost::hash<boost::uuids::uuid>>                edge_uuids_;
    std::unordered_set<boost::uuids::uuid, boost::hash<boost::uuids::uuid>> edge_ignore_uuids_;  // TODO test if this is still necessary

    // Anchor node/keyframe and edge which are treated differently
    g2o::VertexSE3* anchor_node_;
    g2o::EdgeSE3*   anchor_edge_g2o_;
    KeyFrame::Ptr   anchor_kf_;
    Edge::Ptr       anchor_edge_ptr_;

    // unique id generators
    boost::uuids::random_generator_pure uuid_generator_;
    boost::uuids::string_generator      uuid_from_string_generator_;

    // unique id the current instance of the slam graph. unique for each robot run
    boost::uuids::uuid slam_uuid_;
    std::string        slam_uuid_str_;

    // ROS2 parameters
    std::string own_name_;
};

}  // namespace mrg_slam


#endif  // GRAPH_DATA_BASE_HPP