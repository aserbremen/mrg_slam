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
// pcl
#include <pcl/point_cloud.h>
// ROS2
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mrg_slam {

class GraphDatabase {
public:
    typedef pcl::PointXYZI PointT;

    GraphDatabase( rclcpp::Node::SharedPtr _node, std::shared_ptr<GraphSLAM> _graph_slam );

    bool empty() const { return keyframes.empty() && new_keyframes.empty(); }

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
    void add_static_keyframes( const std::vector<mrg_slam_msgs::msg::KeyFrameRos>& keyframes );

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
     * @param others_prev_robot_keyframes
     * @return true if at least one unique keyframe or edge was added to the graph
     */
    bool flush_graph_queue(
        std::unordered_map<std::string, std::pair<KeyFrame::ConstPtr, Eigen::Isometry3d>>& others_prev_robot_keyframes );

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
    bool flush_loaded_graph();

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

    const std::deque<KeyFrame::Ptr>&                                               get_keyframe_queue() const { return keyframe_queue; }
    const std::vector<KeyFrame::Ptr>&                                              get_keyframes() const { return keyframes; }
    const std::deque<KeyFrame::Ptr>&                                               get_new_keyframes() const { return new_keyframes; }
    const std::vector<Edge::Ptr>&                                                  get_edges() const { return edges; }
    const std::unordered_set<boost::uuids::uuid, boost::hash<boost::uuids::uuid>>& get_edge_ignore_uuids() const { return edge_uuids; }
    KeyFrame::ConstPtr get_prev_robot_keyframe() const { return prev_robot_keyframe; }
    const std::unordered_map<builtin_interfaces::msg::Time, KeyFrame::Ptr, RosTimeHash>& get_keyframe_hash() const { return keyframe_hash; }
    const g2o::VertexSE3*                                                                get_anchor_node() const { return anchor_node; }
    const g2o::EdgeSE3*       get_anchor_edge_g2o() const { return anchor_edge_g2o; }
    const boost::uuids::uuid& get_slam_instance_uuid() const { return slam_instance_uuid; }

private:
    std::shared_ptr<GraphSLAM>                   graph_slam;
    std::unique_ptr<InformationMatrixCalculator> inf_calclator;

    int odom_keyframe_counter;

    std::mutex                keyframe_queue_mutex;  // keyframes to be added to the graph
    std::deque<KeyFrame::Ptr> keyframe_queue;

    std::mutex                static_keyframe_queue_mutex;  // keyframes from the static keyframe provider
    std::deque<KeyFrame::Ptr> static_keyframe_queue;

    std::deque<KeyFrame::Ptr> new_keyframes;  // keyframes to be checked for loop closure
    KeyFrame::ConstPtr        prev_robot_keyframe;

    // keyframe and edges container for loading graph from a previously save_graph call
    std::mutex                 loaded_graph_mutex;
    std::vector<KeyFrame::Ptr> loaded_keyframes;
    std::vector<Edge::Ptr>     loaded_edges;

    // keyframes and edges in the graph in order of addition
    std::vector<KeyFrame::Ptr> keyframes;
    std::vector<Edge::Ptr>     edges;

    std::deque<mrg_slam_msgs::msg::GraphRos::ConstSharedPtr> graph_queue;

    // keyframes and edges successfully added to the graph
    std::unordered_map<builtin_interfaces::msg::Time, KeyFrame::Ptr, RosTimeHash>          keyframe_hash;
    std::unordered_map<boost::uuids::uuid, KeyFrame::Ptr, boost::hash<boost::uuids::uuid>> uuid_keyframe_map;
    std::unordered_set<boost::uuids::uuid, boost::hash<boost::uuids::uuid>>                edge_uuids;
    std::unordered_set<boost::uuids::uuid, boost::hash<boost::uuids::uuid>> edge_ignore_uuids;  // TODO test if this is still necessary

    // Anchor node/keyframe and edge which are treated differently
    g2o::VertexSE3* anchor_node;
    g2o::EdgeSE3*   anchor_edge_g2o;
    KeyFrame::Ptr   anchor_kf;
    Edge::Ptr       anchor_edge_ptr;

    // unique id generators
    boost::uuids::random_generator_pure uuid_generator;
    boost::uuids::string_generator      uuid_from_string_generator;

    // unique id the current instance of the slam graph. unique for each robot run
    boost::uuids::uuid slam_instance_uuid;
    std::string        slam_instance_uuid_str;

    // ROS2 parameters
    std::string         own_name;
    bool                fix_first_node;
    bool                fix_first_node_adaptive;
    std::vector<double> fix_first_node_stddev_vec;
    int                 max_keyframes_per_update;
    std::string         odometry_edge_robust_kernel;
    double              odometry_edge_robust_kernel_size;
    std::string         loop_closure_edge_robust_kernel;
    double              loop_closure_edge_robust_kernel_size;
    std::string         result_dir;
};

}  // namespace mrg_slam


#endif  // GRAPH_DATA_BASE_HPP