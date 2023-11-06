// SPDX-License-Identifier: BSD-2-Clause

#ifndef LOOP_DETECTOR_HPP
#define LOOP_DETECTOR_HPP

#include <g2o/types/slam3d/vertex_se3.h>

#include <boost/format.hpp>
#include <hdl_graph_slam/global_id.hpp>
#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/registrations.hpp>
#include <unordered_map>

namespace hdl_graph_slam {

struct Loop {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<Loop>;

    Loop( const KeyFrame::Ptr& key1, const KeyFrame::Ptr& key2, const Eigen::Matrix4f& relpose ) :
        key1( key1 ), key2( key2 ), relative_pose( relpose )
    {
    }

public:
    KeyFrame::Ptr   key1;
    KeyFrame::Ptr   key2;
    Eigen::Matrix4f relative_pose;
};

/**
 * @brief this class finds loops by scam matching and adds them to the pose graph
 */
class LoopDetector {
public:
    typedef pcl::PointXYZI PointT;

    // /**
    //  * @brief constructor
    //  * @param pnh
    //  */
    // LoopDetector( ros::NodeHandle& pnh );

    /**
     * @brief Construct a new Loop Detector object
     * @param _node Shared pointer to the main node
     */
    LoopDetector( rclcpp::Node::SharedPtr _node, std::shared_ptr<GlobalIdGenerator> _gid_generator );

    /**
     * @brief detect loops and add them to the pose graph
     * @param keyframes       keyframes
     * @param new_keyframes   newly registered keyframes
     * @param graph_slam      pose graph
     */
    std::vector<Loop::Ptr> detect( const std::vector<KeyFrame::Ptr>& keyframes, const std::deque<KeyFrame::Ptr>& new_keyframes,
                                   hdl_graph_slam::GraphSLAM& graph_slam, const std::vector<Edge::Ptr>& edges,
                                   const std::unordered_map<GlobalId, KeyFrame::Ptr>& gid_keyframe_map );

    double get_distance_thresh() const;

private:
    /**
     * @brief find loop candidates. A detected loop begins at one of #keyframes and ends at #new_keyframe
     * @param keyframes      candidate keyframes of loop start
     * @param new_keyframe   loop end keyframe
     * @return loop candidates
     */
    std::vector<KeyFrame::Ptr> find_candidates( const std::vector<KeyFrame::Ptr>& keyframes, const KeyFrame::Ptr& new_keyframe ) const;

    /**
     * @brief To validate a loop candidate this function applies a scan matching between keyframes consisting the loop. If they are matched
     * well, the loop is added to the pose graph
     * @param candidate_keyframes  candidate keyframes of loop start
     * @param new_keyframe         loop end keyframe
     * @param graph_slam           graph slam
     */
    Loop::Ptr matching( const std::vector<KeyFrame::Ptr>& candidate_keyframes, const KeyFrame::Ptr& new_keyframe,
                        hdl_graph_slam::GraphSLAM& graph_slam, const std::vector<KeyFrame::Ptr>& keyframes,
                        const std::vector<Edge::Ptr>& edges, const std::unordered_map<GlobalId, KeyFrame::Ptr>& gid_keyframe_map );


private:
    rclcpp::Node::SharedPtr            node_ros;
    std::shared_ptr<GlobalIdGenerator> gid_generator;

    double distance_thresh,
        distance_thresh_squared;            // estimated distance between keyframes consisting a loop must be less than this distance
    double accum_distance_thresh;           // traveled distance between ...
    double distance_from_last_edge_thresh;  // a new loop edge must far from the last one at least this distance

    double fitness_score_max_range;  // maximum allowable distance between corresponding points
    double fitness_score_thresh;     // threshold for scan matching

    bool   use_loop_closure_consistency_check;        // whether to check
    double loop_closure_consistency_max_delta_trans;  // maximum allowed translation distance for loop closure graph consistency check (m)
    double loop_closure_consistency_max_delta_angle;  // maximum allowed rotation angle for loop closure graph consistency check (deg)

    bool use_planar_registration_guess;  // Whether to set z=0 for the registration guess

    std::unordered_map<std::string, double> last_edge_accum_distance_map;
    // double                                  last_edge_accum_distance;


    pcl::Registration<PointT, PointT>::Ptr registration;
};

}  // namespace hdl_graph_slam

#endif  // LOOP_DETECTOR_HPP
