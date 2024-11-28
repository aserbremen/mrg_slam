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
    KeyFrame::Ptr   key1;           // new keyframe testing for loop closure
    KeyFrame::Ptr   key2;           // best matched candidate keyframe
    Eigen::Matrix4f relative_pose;  // relative pose from key1 to key2
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
     * @brief find loop preliminary loop closure candidates in the vicinity of the new keyframe
     * @param keyframes keyframes to search for candidates
     * @param new_keyframe  new keyframe testing for loop closure against existing keyframes of the graph
     */
    std::vector<KeyFrame::Ptr> find_candidates( const KeyFrame::Ptr& new_keyframe, const std::vector<KeyFrame::Ptr>& keyframes ) const;

    /**
     * @brief Filter the candidates by distance thresholds and accumulated distance differences
     *
     * @param candidates candidate keyframes
     * @return std::vector<KeyFrame::Ptr> filtered candidates
     */
    std::vector<KeyFrame::Ptr> filter_candidates( const std::vector<KeyFrame::Ptr>& candidates, const KeyFrame::Ptr& new_keyframe );

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
    double accum_distance_thresh;  // accumulated distance difference between new keyframe and candidandate keyframe of the same slam
                                   // instance  must be larger than  this, otherwise candidate is ignored
    double accum_distance_thresh_other_slam_instance;  // accumulated distance difference between new keyframe and candidandate keyframe of
                                                       // another slam instance must be larger than this, otherwise candidate is ignored
    double distance_from_last_loop_edge_thresh;        // a new loop edge must far from the last one at least this distance

    double fitness_score_max_range;  // maximum allowable distance between corresponding points
    double fitness_score_thresh;     // threshold for scan matching

    bool   use_loop_closure_consistency_check;        // whether to check
    double loop_closure_consistency_max_delta_trans;  // maximum allowed translation distance for loop closure graph consistency check (m)
    double loop_closure_consistency_max_delta_angle;  // maximum allowed rotation angle for loop closure graph consistency check (deg)

    bool use_planar_registration_guess;  // Whether to set z=0 for the registration guess

    // map of a new keyframe slam instance uuid to candidate instance uuid and their accumulated distances
    struct LoopClosureInfo {
        LoopClosureInfo() : new_keyframe( nullptr ), candidate( nullptr ) {}
        LoopClosureInfo( KeyFrame::ConstPtr _new_keyframe, KeyFrame::ConstPtr _candidate ) :
            new_keyframe( _new_keyframe ), candidate( _candidate )
        {
        }
        KeyFrame::ConstPtr new_keyframe;
        KeyFrame::ConstPtr candidate;
    };
    // map of a new keyframe slam instance uuid to the last loop edge information
    std::unordered_map<boost::uuids::uuid, LoopClosureInfo> last_loop_info_map;

    pcl::Registration<PointT, PointT>::Ptr registration;

public:
    // Statistics
    std::vector<int64_t> loop_detection_times;
    std::vector<int>     loop_candidates_sizes;
};

}  // namespace mrg_slam

#endif  // LOOP_DETECTOR_HPP
