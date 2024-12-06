// SPDX-License-Identifier: BSD-2-Clause

#include <mrg_slam/edge.hpp>
#include <mrg_slam/graph_database.hpp>
#include <mrg_slam/loop_detector.hpp>


namespace mrg_slam {

LoopDetector::LoopDetector( rclcpp::Node::SharedPtr _node ) : node_ros( _node )
{
    distance_thresh                           = node_ros->get_parameter( "distance_thresh" ).as_double();
    distance_thresh_squared                   = distance_thresh * distance_thresh;
    accum_distance_thresh                     = node_ros->get_parameter( "accum_distance_thresh" ).as_double();
    accum_distance_thresh_other_slam_instance = node_ros->get_parameter( "accum_distance_thresh_other_slam_instance" ).as_double();
    distance_from_last_loop_edge_thresh       = node_ros->get_parameter( "min_edge_interval" ).as_double();

    fitness_score_max_range = node_ros->get_parameter( "fitness_score_max_range" ).as_double();
    fitness_score_thresh    = node_ros->get_parameter( "fitness_score_thresh" ).as_double();

    use_loop_closure_consistency_check       = node_ros->get_parameter( "use_loop_closure_consistency_check" ).as_bool();
    loop_closure_consistency_max_delta_trans = node_ros->get_parameter( "loop_closure_consistency_max_delta_trans" ).as_double();
    loop_closure_consistency_max_delta_angle = node_ros->get_parameter( "loop_closure_consistency_max_delta_angle" ).as_double();

    use_planar_registration_guess = node_ros->get_parameter( "use_planar_registration_guess" ).as_bool();

    loop_manager = std::make_shared<LoopManager>();

    registration = select_registration_method( node_ros.get() );
}


std::vector<Loop::Ptr>
LoopDetector::detect( std::shared_ptr<GraphDatabase> graph_db )
{
    std::cout << std::endl << "--- loop detection ---" << std::endl;

    const auto&            keyframes     = graph_db->get_keyframes();
    const auto&            new_keyframes = graph_db->get_new_keyframes();
    std::vector<Loop::Ptr> detected_loops;
    for( const auto& new_keyframe : new_keyframes ) {
        auto start = std::chrono::high_resolution_clock::now();

        auto candidates = find_candidates( new_keyframe, keyframes );
        candidates      = filter_candidates( candidates, new_keyframe );
        auto loop       = matching( candidates, new_keyframe );
        if( loop ) {
            detected_loops.push_back( loop );
        }

        if( candidates.size() > 0 ) {
            loop_candidates_sizes.push_back( candidates.size() );
            auto end = std::chrono::high_resolution_clock::now();
            loop_detection_times.push_back( std::chrono::duration_cast<std::chrono::microseconds>( end - start ).count() );
        }
    }

    return detected_loops;
}


double
LoopDetector::get_distance_thresh() const
{
    return distance_thresh;
}


std::vector<KeyFrame::Ptr>
LoopDetector::find_candidates( const KeyFrame::Ptr& new_keyframe, const std::vector<KeyFrame::Ptr>& keyframes ) const
{
    std::vector<KeyFrame::Ptr> candidates = std::vector<KeyFrame::Ptr>();
    // TODO reserve size with random number?
    candidates.reserve( 32 );

    for( const auto& k : keyframes ) {
        // there is already an edge
        if( new_keyframe->edge_exists( *k, rclcpp::get_logger( "find_candidates" ) ) ) {
            continue;
        }

        // dont consider first keyframes since they don't filter out points that hit other rovers
        if( k->first_keyframe ) {
            continue;
        }

        const auto& pos1 = k->node->estimate().translation();
        const auto& pos2 = new_keyframe->node->estimate().translation();

        // estimated distance between keyframes is too big for given distance threshold
        double dist_squared = ( pos1.head<2>() - pos2.head<2>() ).squaredNorm();
        if( dist_squared > distance_thresh_squared ) {
            continue;
        }

        candidates.push_back( k );
    }

    // TODO? add candidates from the new_keyframes vector itself? This is necessary if a lot of new keyframes are added at once

    return candidates;
}


std::vector<KeyFrame::Ptr>
LoopDetector::filter_candidates( const std::vector<KeyFrame::Ptr>& candidates, const KeyFrame::Ptr& new_keyframe )
{
    auto logger = rclcpp::get_logger( "LoopDetector::filter_candidates" );

    if( candidates.empty() ) {
        RCLCPP_INFO_STREAM( logger, "No candidates for " << new_keyframe->readable_id );
        return {};
    }

    RCLCPP_INFO_STREAM( logger, "Filtering " << candidates.size() << " candidates for " << new_keyframe->readable_id );
    std::vector<KeyFrame::Ptr> filtered_candidates;

    // Add final candidates if they are fit for loop closure matching
    for( const auto& cand : candidates ) {
        if( new_keyframe->slam_uuid == cand->slam_uuid && new_keyframe->accum_distance - cand->accum_distance < accum_distance_thresh ) {
            RCLCPP_INFO_STREAM( logger, "Not enough distance accumulated between new and candidate keyframe << "
                                            << new_keyframe->accum_distance - cand->accum_distance );
            continue;
        }


        auto last_loop = loop_manager->get_loop( new_keyframe->slam_uuid, cand->slam_uuid );

        // Check if the candidate is too close in distance for the same SLAM instance to avoid loops very close to each other
        if( last_loop && new_keyframe->slam_uuid == cand->slam_uuid
            && new_keyframe->accum_distance - last_loop->key1->accum_distance < accum_distance_thresh ) {
            RCLCPP_INFO_STREAM( logger, "Removing candidate "
                                            << cand->readable_id
                                            << " since not enough distance accumulated from last loop with same slam uuid at "
                                            << last_loop->key1->readable_id << " with delta dist "
                                            << new_keyframe->accum_distance - last_loop->key1->accum_distance );
            continue;
        }

        if( last_loop && new_keyframe->slam_uuid != cand->slam_uuid
            && new_keyframe->accum_distance - last_loop->key1->accum_distance < accum_distance_thresh_other_slam_instance ) {
            RCLCPP_INFO_STREAM( logger, "Removing candidate "
                                            << cand->readable_id
                                            << " since not enough distance accumulated from last loop at other slam uuid at "
                                            << last_loop->key1->readable_id << " with delta dist "
                                            << new_keyframe->accum_distance - last_loop->key1->accum_distance );
            continue;
        }

        // Add candidate to filtered_candidates if it should be kept
        filtered_candidates.push_back( cand );
    }

    RCLCPP_INFO_STREAM( logger, "Returning " << filtered_candidates.size() << " candidates for " << new_keyframe->readable_id );

    return filtered_candidates;
}


Loop::Ptr
LoopDetector::matching( const std::vector<KeyFrame::Ptr>& candidate_keyframes, const KeyFrame::Ptr& new_keyframe )
{
    if( candidate_keyframes.empty() ) {
        return nullptr;
    }

    registration->setInputTarget( new_keyframe->cloud );

    double          best_score   = std::numeric_limits<double>::max();
    KeyFrame::Ptr   best_matched = nullptr;
    Eigen::Matrix4f rel_pose_new_to_best_matched;

    // std::cout << std::endl << "--- loop detection ---" << std::endl;
    std::cout << "new keyframe " << new_keyframe->readable_id << " has " << candidate_keyframes.size() << " candidates, matching..."
              << std::endl;
    for( const auto& cand : candidate_keyframes ) {
        std::cout << cand->readable_id << " ";
    }
    std::cout << std::endl;
    auto t1 = std::chrono::system_clock::now();

    pcl::PointCloud<PointT>::Ptr aligned( new pcl::PointCloud<PointT>() );
    Eigen::Isometry3d            new_keyframe_estimate = normalize_estimate( new_keyframe->node->estimate() );

    for( const auto& candidate : candidate_keyframes ) {
        registration->setInputSource( candidate->cloud );

        Eigen::Isometry3d candidate_estimate = normalize_estimate( candidate->node->estimate() );
        Eigen::Matrix4f   guess              = ( new_keyframe_estimate.inverse() * candidate_estimate ).matrix().cast<float>();
        if( use_planar_registration_guess ) {
            guess( 2, 3 ) = 0.0;
        }
        registration->align( *aligned, guess );
        std::cout << "." << std::flush;

        double score = registration->getFitnessScore( fitness_score_max_range );
        if( !registration->hasConverged() || score > best_score ) {
            continue;
        }

        best_score                   = score;
        best_matched                 = candidate;
        rel_pose_new_to_best_matched = registration->getFinalTransformation();  // New to candidate
    }

    if( best_matched != nullptr ) {
        std::cout << std::endl << "best matched candidate: " << best_matched->readable_id << " score " << best_score << std::endl;
    }

    bool consistency_check_passed = perform_loop_closure_consistency_check( new_keyframe_estimate, rel_pose_new_to_best_matched,
                                                                            best_matched, best_score );

    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now() - t1 );
    std::cout << "elapsed time in loop closure: " << elapsed_ms.count() << "[msec]" << std::endl;
    if( best_score > fitness_score_thresh ) {
        std::cout << "loop not found... best score " << best_score << " > " << fitness_score_thresh << " fitness thresh" << std::endl;
        return nullptr;
    }

    if( use_loop_closure_consistency_check && best_matched != nullptr && best_matched->first_keyframe == false
        && !consistency_check_passed ) {
        std::cout << "didnt pass either consistency check" << std::endl;
        return nullptr;
    }

    std::cout << "loop found! from " << new_keyframe->readable_id << " to " << best_matched->readable_id << std::endl;
    std::cout << "relpose: " << rel_pose_new_to_best_matched.block<3, 1>( 0, 3 ).transpose() << " quat "
              << Eigen::Quaternionf( rel_pose_new_to_best_matched.block<3, 3>( 0, 0 ) ).coeffs().transpose() << std::endl;

    // Last edge accum distance is only updated if the new keyframe is a keyframe of this robot
    std::cout << "updating last loop edge accum distance for " << new_keyframe->readable_id << " to " << new_keyframe->accum_distance
              << std::endl;
    // last_loop_info_map[new_keyframe->slam_uuid] = LoopClosureInfo( new_keyframe, best_matched );
    auto new_loop = std::make_shared<Loop>( new_keyframe, best_matched, rel_pose_new_to_best_matched );
    loop_manager->add_loop( new_loop );

    return new_loop;
}


Eigen::Isometry3d
LoopDetector::normalize_estimate( const Eigen::Isometry3d& estimate ) const
{
    Eigen::Isometry3d normalized_estimate = estimate;
    normalized_estimate.linear()          = Eigen::Quaterniond( normalized_estimate.linear() ).normalized().toRotationMatrix();
    return normalized_estimate;
}


bool
LoopDetector::perform_loop_closure_consistency_check( const Eigen::Isometry3d& new_keyframe_estimate,
                                                      const Eigen::Matrix4f&   rel_pose_new_to_best_matched,
                                                      const KeyFrame::Ptr& best_matched, double best_score ) const
{
    // If the best matched candidate is the first keyframe from a robot, we dont perform the consistency check
    // Also if the keyframe is static (no prev or next edge), i.e. given by the static map provider we dont perform the consistency check
    if( best_matched != nullptr && ( best_matched->first_keyframe || best_matched->static_keyframe ) ) {
        return true;
    }

    if( best_matched == nullptr || !use_loop_closure_consistency_check || best_score > fitness_score_thresh ) {
        return false;
    }

    std::cout << "Performing loop closure consistency check with best matched candidate " << best_matched->readable_id << " score "
              << best_score << std::endl;

    if( check_consistency_with_prev_keyframe( new_keyframe_estimate, rel_pose_new_to_best_matched, best_matched ) ) {
        return true;
    }
    // Check with next keyframe only if the consistency check with prev keyframe failed
    return check_consistency_with_next_keyframe( new_keyframe_estimate, rel_pose_new_to_best_matched, best_matched );
}

bool
LoopDetector::check_consistency_with_prev_keyframe( const Eigen::Isometry3d& new_keyframe_estimate,
                                                    const Eigen::Matrix4f&   rel_pose_new_to_best_matched,
                                                    const KeyFrame::Ptr&     best_matched ) const
{
    if( best_matched->prev_edge == nullptr ) {
        std::cout << "candidate " << best_matched->readable_id << " has no prev edge" << std::endl;
        return false;
    }

    Eigen::Matrix4f rel_pose_candidate_to_prev = best_matched->prev_edge->relative_pose().matrix().cast<float>();

    const auto& prev_kf = best_matched->prev_edge->to_keyframe;
    registration->setInputSource( prev_kf->cloud );
    Eigen::Isometry3d prev_kf_estimate = normalize_estimate( prev_kf->node->estimate() );
    Eigen::Matrix4f   prev_guess       = ( new_keyframe_estimate.inverse() * prev_kf_estimate ).matrix().cast<float>();
    if( use_planar_registration_guess ) {
        prev_guess( 2, 3 ) = 0.0;
    }
    pcl::PointCloud<PointT>::Ptr prev_aligned( new pcl::PointCloud<PointT>() );
    registration->align( *prev_aligned, prev_guess );

    // We dont check if the registration has converged here, since we are only interested in the relative pose
    Eigen::Matrix4f rel_pose_new_to_prev = registration->getFinalTransformation();

    // Calculate the transformation from candidate to prev to new keyframe which should be identity matrix
    auto  rel_pose_identity_check_prev = rel_pose_new_to_prev.inverse() * rel_pose_new_to_best_matched * rel_pose_candidate_to_prev;
    float delta_trans                  = rel_pose_identity_check_prev.block<3, 1>( 0, 3 ).norm();
    float delta_angle =
        Eigen::Quaternionf( rel_pose_identity_check_prev.block<3, 3>( 0, 0 ) ).angularDistance( Eigen::Quaternionf::Identity() );

    if( delta_trans > loop_closure_consistency_max_delta_trans || delta_angle > loop_closure_consistency_max_delta_angle ) {
        std::cout << "Inconsistent with prev keyframe " << prev_kf->readable_id << " delta trans " << delta_trans << " delta angle (deg)"
                  << delta_angle * 180.0 / M_PI << std::endl;
        return false;
    }
    // Success
    std::cout << "Consistent with prev keyframe " << prev_kf->readable_id << " delta trans " << delta_trans << " delta angle (deg) "
              << delta_angle * 180.0 / M_PI << std::endl;
    return true;
}


bool
LoopDetector::check_consistency_with_next_keyframe( const Eigen::Isometry3d& new_keyframe_estimate,
                                                    const Eigen::Matrix4f&   rel_pose_new_to_best_matched,
                                                    const KeyFrame::Ptr&     best_matched ) const
{
    if( best_matched->next_edge == nullptr ) {
        std::cout << "candidate " << best_matched->readable_id << " has no next edge" << std::endl;
        return false;
    }

    Eigen::Matrix4f rel_pose_next_to_candidate = best_matched->next_edge->relative_pose().matrix().cast<float>();

    const auto& next_kf = best_matched->next_edge->from_keyframe;
    registration->setInputSource( next_kf->cloud );
    Eigen::Isometry3d next_kf_estimate = normalize_estimate( next_kf->node->estimate() );
    Eigen::Matrix4f   next_guess       = ( new_keyframe_estimate.inverse() * next_kf_estimate ).matrix().cast<float>();
    if( use_planar_registration_guess ) {
        next_guess( 2, 3 ) = 0.0;
    }
    pcl::PointCloud<PointT>::Ptr next_aligned( new pcl::PointCloud<PointT>() );
    registration->align( *next_aligned, next_guess );

    // We dont check if the registration has converged here, since we are only interested in the relative pose
    Eigen::Matrix4f rel_pose_new_to_next = registration->getFinalTransformation();

    // Calculate the transformation from candidate to next to new keyframe which should be identity matrix
    auto rel_pose_identity_check_next = rel_pose_new_to_best_matched.inverse() * rel_pose_new_to_next * rel_pose_next_to_candidate;
    // Alternative way to calculate the identity transform
    // auto rel_pose_identity_check_next2 = rel_pose_new_to_next.inverse() * rel_pose_new_to_best_matched
    //                                      * rel_pose_next_to_candidate.inverse();
    float delta_trans_next = rel_pose_identity_check_next.block<3, 1>( 0, 3 ).norm();
    float delta_angle_next =
        Eigen::Quaternionf( rel_pose_identity_check_next.block<3, 3>( 0, 0 ) ).angularDistance( Eigen::Quaternionf::Identity() );

    if( delta_trans_next > loop_closure_consistency_max_delta_trans || delta_angle_next > loop_closure_consistency_max_delta_angle ) {
        std::cout << "Inconsistent with next keyframe " << next_kf->readable_id << " delta trans " << delta_trans_next << " delta angle "
                  << delta_angle_next * 180.0 / M_PI << std::endl;
        return false;
    }

    std::cout << "Consistent with next keyframe " << next_kf->readable_id << " delta trans " << delta_trans_next << " delta angle "
              << delta_angle_next * 180.0 / M_PI << std::endl;
    return true;
}


}  // namespace mrg_slam