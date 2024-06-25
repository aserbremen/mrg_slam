// SPDX-License-Identifier: BSD-2-Clause

#ifndef LOOP_DETECTOR_HPP
#define LOOP_DETECTOR_HPP

#include <g2o/types/slam3d/vertex_se3.h>

#include <boost/format.hpp>
#include <mrg_slam/graph_slam.hpp>
#include <mrg_slam/keyframe.hpp>
#include <mrg_slam/registrations.hpp>
#include <unordered_map>

namespace mrg_slam {

// Forward declarations
class GraphDatabase;

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

    /**
     * @brief Construct a new Loop Detector object
     * @param _node Shared pointer to the main node
     */
    LoopDetector( rclcpp::Node::SharedPtr _node );

    /**
     * @brief Detect loops and perform loop consistency checks
     */
    std::vector<Loop::Ptr> detect( std::shared_ptr<GraphDatabase> graph_db );

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
                        const std::vector<KeyFrame::Ptr>& keyframes, const std::vector<Edge::Ptr>& edges );


private:
    rclcpp::Node::SharedPtr node_ros;

    double distance_thresh,
        distance_thresh_squared;   // estimated distance between new keyframe and candidate be less than this distance, for the candidate to
                                   // be considered
    double accum_distance_thresh;  // accumulated distance difference between new keyframe and candidandate keyframe must be larger than
                                   // this, otherwise candidate is ignored
    double distance_from_last_loop_edge_thresh;  // a new loop edge must far from the last one at least this distance

    double fitness_score_max_range;  // maximum allowable distance between corresponding points
    double fitness_score_thresh;     // threshold for scan matching

    bool   use_loop_closure_consistency_check;        // whether to check
    double loop_closure_consistency_max_delta_trans;  // maximum allowed translation distance for loop closure graph consistency check (m)
    double loop_closure_consistency_max_delta_angle;  // maximum allowed rotation angle for loop closure graph consistency check (deg)

    bool use_planar_registration_guess;  // Whether to set z=0 for the registration guess

    std::unordered_map<std::string, double> last_loop_edge_accum_distance_map;

    pcl::Registration<PointT, PointT>::Ptr registration;

public:
    // Statistics
    std::vector<int64_t> loop_detection_times;
    std::vector<int>     loop_candidates_sizes;
};

}  // namespace mrg_slam

#endif  // LOOP_DETECTOR_HPP
