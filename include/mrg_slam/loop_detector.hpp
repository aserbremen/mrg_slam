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

class LoopManager {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr      = std::shared_ptr<LoopManager>;
    using ConstPtr = std::shared_ptr<const LoopManager>;

    LoopManager() = default;

    /**
     * @brief Get the loop object for a given new keyframe and candidate keyframe uuid
     *
     * @param new_keyframe_slam_uuid new keyframe uuid
     * @param candidate_slam_uuid candidate keyframe uuid
     * @return Loop::Ptr most recent loop object if found, nullptr otherwise
     */
    Loop::Ptr get_loop( const boost::uuids::uuid& new_keyframe_slam_uuid, const boost::uuids::uuid& candidate_slam_uuid ) const
    {
        auto new_keyframe_it = loop_map.find( new_keyframe_slam_uuid );
        if( new_keyframe_it != loop_map.end() ) {
            auto candidate_it = new_keyframe_it->second.find( candidate_slam_uuid );
            if( candidate_it != new_keyframe_it->second.end() ) {
                return candidate_it->second;
            }
        }
        // There is no registered loop closure for the given combination of slam uuids
        return nullptr;
    }

    /**
     * @brief Get the most recent loops for a given new keyframe uuid
     *
     * @param new_keyframe_slam_uuid new keyframe uuid
     * @return std::unordered_map<boost::uuids::uuid, Loop::Ptr> most recent loops for the given new keyframe uuid. The loops are given in a
     * map. If no loops are found, an empty map is returned
     */
    std::unordered_map<boost::uuids::uuid, Loop::Ptr> get_loops( const boost::uuids::uuid& new_keyframe_slam_uuid ) const
    {
        auto new_keyframe_it = loop_map.find( new_keyframe_slam_uuid );
        if( new_keyframe_it != loop_map.end() ) {
            return new_keyframe_it->second;
        }
        return {};
    }

    /**
     * @brief Adds a loop to the loop manager
     *
     * @param loop loop to be added
     */
    void add_loop( const Loop::Ptr& loop ) { loop_map[loop->key1->slam_uuid][loop->key2->slam_uuid] = loop; }

    /**
     * @brief Adds a loop to the loop manager. If a loop with the same new keyframe uuid and candidate keyframe uuid already exists, the
     * loop with the higher accum distance is kept
     *
     * @param loop loop to be added
     */
    void add_loop_accum_distance_check( const Loop::Ptr& loop )
    {
        auto available_loop = get_loop( loop->key1->slam_uuid, loop->key2->slam_uuid );
        if( !available_loop ) {
            std::cout << "Adding " << loop->key1->readable_id << " to LoopManager" << std::endl;
            add_loop( loop );
            return;
        }
        // overwrite if the loop to be added is more recent than the previously registered loop
        if( loop->key1->accum_distance > available_loop->key1->accum_distance ) {
            std::cout << "Overwriting " << available_loop->key1->readable_id << " with " << loop->key1->readable_id
                      << " due to higher accum distance" << std::endl;
            add_loop( loop );
        }
    }

private:
    // The nested unordered_map keeps track of loops registered to the new keyframe being tested for loop closures. Only the loop with the
    // highest accumulated distance is kept
    // new_keyframe_slam_uuid -> <candidate_slam_uuid, Loop::Ptr>
    std::unordered_map<boost::uuids::uuid, std::unordered_map<boost::uuids::uuid, Loop::Ptr>> loop_map;
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
    LoopDetector( rclcpp::Node::SharedPtr node );

    /**
     * @brief Detect loops and perform loop consistency checks
     */
    std::vector<Loop::Ptr> detect( std::shared_ptr<GraphDatabase> graph_db );

    LoopManager::Ptr get_loop_manager() { return loop_manager_; }

    // Statistics
    std::vector<int64_t> loop_detection_times;
    std::vector<int>     loop_candidates_sizes;

private:
    /**
     * @brief find loop preliminary loop closure candidates in the vicinity of the new keyframe
     * @param keyframes keyframes to search for candidates
     * @param new_keyframe  new keyframe testing for loop closure against existing keyframes of the graph
     */
    std::vector<KeyFrame::Ptr> find_candidates( const KeyFrame::Ptr& new_keyframe, const std::vector<KeyFrame::Ptr>& keyframes ) const;

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
    rclcpp::Node::SharedPtr node_;

    LoopManager::Ptr loop_manager_;

    pcl::Registration<PointT, PointT>::Ptr registration_;
};

}  // namespace mrg_slam

#endif  // LOOP_DETECTOR_HPP
