// SPDX-License-Identifier: BSD-2-Clause

#ifndef LOOP_DETECTOR_HPP
#define LOOP_DETECTOR_HPP

#include <unordered_map>
// g2o
#include <g2o/types/slam3d/vertex_se3.h>
// mrg_slam
#include <mrg_slam/graph_database.hpp>
#include <mrg_slam/graph_slam.hpp>
#include <mrg_slam/keyframe.hpp>
#include <mrg_slam/registrations.hpp>
// boost
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_hash.hpp>

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
 * @brief this class finds loops by scan matching and adds them to the pose graph
 */
class LoopDetector {
public:
    typedef pcl::PointXYZI PointT;

    /**
     * @brief Construct a new Loop Detector object
     * @param _node Shared pointer to the main slam node
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
    std::vector<KeyFrame::Ptr> find_candidates( const KeyFrame::Ptr& new_keyframe, const std::vector<KeyFrame::Ptr>& keyframes,
                                                const boost::uuids::uuid& slam_instance_id ) const;

    /**
     * @brief To validate a loop candidate this function applies a scan matching between keyframes consisting the loop. If they are matched
     * well, the loop is added to the pose graph
     * @param candidate_keyframes  candidate keyframes of loop start
     * @param new_keyframe         loop end keyframe
     * @param graph_slam           graph slam
     */
    Loop::Ptr matching( const std::vector<KeyFrame::Ptr>& candidate_keyframes, const KeyFrame::Ptr& new_keyframe );


    /**
     * @brief Normalize the estimate by normalizing the quaternion
     *
     * @param estimate Eigen::Isoemtry3d given by the g2o node
     * @return Eigen::Isometry3d
     */
    Eigen::Isometry3d normalize_estimate( const Eigen::Isometry3d& estimate ) const;

    /**
     * @brief Calculate relative transformation from new keyframe to candidate to its previous/next keyframe and back to new keyframe, which
     * should be the identity transformation if the loop closure hypothesis is correct.
     *
     * @param new_keyframe_estimate Normalized graph estimate of the new keyframe
     * @param rel_pose_new_to_best_matched Relative transformation from new keyframe to best matched candidate
     * @param best_matched Best matched candidate keyframe
     * @param best_score Fitness score of the best matched candidate
     * @return true if the loop closure consistency check passed
     */
    bool perform_loop_closure_consistency_check( const Eigen::Isometry3d& new_keyframe_estimate,
                                                 const Eigen::Matrix4f& rel_pose_new_to_best_matched, const KeyFrame::Ptr& best_matched,
                                                 double best_score ) const;

    /**
     * @brief Performs the consistency check with the previous keyframe
     *
     * @param new_keyframe_estimate Normalized graph estimate of the new keyframe
     * @param rel_pose_new_to_best_matched Relative transformation from new keyframe to best matched candidate
     * @param best_matched Best matched candidate keyframe
     * @return true if the consistency check passed
     */
    bool check_consistency_with_prev_keyframe( const Eigen::Isometry3d& new_keyframe_estimate,
                                               const Eigen::Matrix4f&   rel_pose_new_to_best_matched,
                                               const KeyFrame::Ptr&     best_matched ) const;

    /**
     * @brief Performs the consistency check with the next keyframe
     *
     * @param new_keyframe_estimate Normalized graph estimate of the new keyframe
     * @param rel_pose_new_to_best_matched Relative transformation from new keyframe to best matched candidate
     * @param best_matched Best matched candidate keyframe
     * @return true if the consistency check passed
     */
    bool check_consistency_with_next_keyframe( const Eigen::Isometry3d& new_keyframe_estimate,
                                               const Eigen::Matrix4f&   rel_pose_new_to_best_matched,
                                               const KeyFrame::Ptr&     best_matched ) const;


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

    // map of the slam instance uuid to the accumulated distance of the last loop edge
    std::unordered_map<boost::uuids::uuid, double> last_loop_edge_accum_distance_map;

    pcl::Registration<PointT, PointT>::Ptr registration;

public:
    // Statistics
    std::vector<int64_t> loop_detection_times;
    std::vector<int>     loop_candidates_sizes;
};

}  // namespace mrg_slam

#endif  // LOOP_DETECTOR_HPP
