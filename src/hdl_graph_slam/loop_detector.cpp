// SPDX-License-Identifier: BSD-2-Clause

#include <hdl_graph_slam/edge.hpp>
#include <hdl_graph_slam/global_id.hpp>
#include <hdl_graph_slam/loop_detector.hpp>


namespace hdl_graph_slam {

LoopDetector::LoopDetector( rclcpp::Node::SharedPtr _node, std::shared_ptr<GlobalIdGenerator> _gid_generator ) :
    node_ros( _node ), gid_generator( _gid_generator )
{
    distance_thresh                = node_ros->get_parameter( "distance_thresh" ).as_double();
    distance_thresh_squared        = distance_thresh * distance_thresh;
    accum_distance_thresh          = node_ros->get_parameter( "accum_distance_thresh" ).as_double();
    distance_from_last_edge_thresh = node_ros->get_parameter( "min_edge_interval" ).as_double();

    fitness_score_max_range = node_ros->get_parameter( "fitness_score_max_range" ).as_double();
    fitness_score_thresh    = node_ros->get_parameter( "fitness_score_thresh" ).as_double();

    use_loop_closure_consistency_check       = node_ros->get_parameter( "use_loop_closure_consistency_check" ).as_bool();
    loop_closure_consistency_max_delta_trans = node_ros->get_parameter( "loop_closure_consistency_max_delta_trans" ).as_double();
    loop_closure_consistency_max_delta_angle = node_ros->get_parameter( "loop_closure_consistency_max_delta_angle" ).as_double();
    // Convert to rad
    loop_closure_consistency_max_delta_angle = loop_closure_consistency_max_delta_angle * M_PI / 180.0;

    use_planar_registration_guess = node_ros->get_parameter( "use_planar_registration_guess" ).as_bool();

    registration = select_registration_method( node_ros.get() );

    last_edge_accum_distance = 0.0;
}

/**
 * @brief detect loops and add them to the pose graph
 * @param keyframes       keyframes
 * @param new_keyframes   newly registered keyframes
 * @param graph_slam      pose graph
 */
std::vector<Loop::Ptr>
LoopDetector::detect( const std::vector<KeyFrame::Ptr>& keyframes, const std::deque<KeyFrame::Ptr>& new_keyframes,
                      hdl_graph_slam::GraphSLAM& graph_slam, const std::vector<Edge::Ptr>& edges,
                      const std::unordered_map<GlobalId, KeyFrame::Ptr>& gid_keyframe_map )
{
    std::vector<Loop::Ptr> detected_loops;
    for( const auto& new_keyframe : new_keyframes ) {
        auto candidates = find_candidates( keyframes, new_keyframe );
        auto loop       = matching( candidates, new_keyframe, graph_slam, keyframes, edges, gid_keyframe_map );
        if( loop ) {
            detected_loops.push_back( loop );
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
LoopDetector::find_candidates( const std::vector<KeyFrame::Ptr>& keyframes, const KeyFrame::Ptr& new_keyframe ) const
{
    // too close to the last registered loop edge
    if( new_keyframe->accum_distance >= 0 && new_keyframe->accum_distance - last_edge_accum_distance < distance_from_last_edge_thresh ) {
        return std::vector<KeyFrame::Ptr>();
    }

    std::vector<KeyFrame::Ptr> candidates;
    candidates.reserve( 32 );

    for( const auto& k : keyframes ) {
        // traveled distance between keyframes is too small
        if( new_keyframe->accum_distance >= 0 && k->accum_distance >= 0
            && new_keyframe->accum_distance - k->accum_distance < accum_distance_thresh ) {
            continue;
        }

        // there is already an edge
        if( new_keyframe->edge_exists( *k, node_ros->get_logger() ) ) {
            continue;
        }

        const auto& pos1 = k->node->estimate().translation();
        const auto& pos2 = new_keyframe->node->estimate().translation();

        // estimated distance between keyframes is too small
        double dist_squared = ( pos1.head<2>() - pos2.head<2>() ).squaredNorm();
        if( dist_squared > distance_thresh_squared ) {
            continue;
        }

        candidates.push_back( k );
    }

    return candidates;
}


Loop::Ptr
LoopDetector::matching( const std::vector<KeyFrame::Ptr>& candidate_keyframes, const KeyFrame::Ptr& new_keyframe,
                        hdl_graph_slam::GraphSLAM& graph_slam, const std::vector<KeyFrame::Ptr>& keyframes,
                        const std::vector<Edge::Ptr>& edges, const std::unordered_map<GlobalId, KeyFrame::Ptr>& gid_keyframe_map )
{
    if( candidate_keyframes.empty() ) {
        return nullptr;
    }

    registration->setInputTarget( new_keyframe->cloud );

    bool            consistency_check_passed = false;
    double          best_score               = std::numeric_limits<double>::max();
    KeyFrame::Ptr   best_matched;
    Eigen::Matrix4f relative_pose;

    std::cout << std::endl;
    std::cout << "--- loop detection ---" << std::endl;
    std::cout << "keyframe " << gid_generator->getHumanReadableId( new_keyframe->gid ) << " has " << candidate_keyframes.size()
              << " candidates" << std::endl;
    for( const auto& cand : candidate_keyframes ) {
        std::cout << gid_generator->getHumanReadableId( cand->gid ) << std::endl;
    }
    std::cout << "matching" << std::endl;
    auto t1 = std::chrono::system_clock::now();

    pcl::PointCloud<PointT>::Ptr aligned( new pcl::PointCloud<PointT>() );
    Eigen::Isometry3d            new_keyframe_estimate = new_keyframe->node->estimate();
    new_keyframe_estimate.linear() = Eigen::Quaterniond( new_keyframe_estimate.linear() ).normalized().toRotationMatrix();
    for( const auto& candidate : candidate_keyframes ) {
        RCLCPP_INFO_STREAM( node_ros->get_logger(), "candidate: " << gid_generator->getHumanReadableId( candidate->gid ) );
        registration->setInputSource( candidate->cloud );

        Eigen::Isometry3d candidate_estimate = candidate->node->estimate();
        candidate_estimate.linear()          = Eigen::Quaterniond( candidate_estimate.linear() ).normalized().toRotationMatrix();
        Eigen::Matrix4f guess                = ( new_keyframe_estimate.inverse() * candidate_estimate ).matrix().cast<float>();
        if( use_planar_registration_guess ) {
            guess( 2, 3 ) = 0.0;
        }
        registration->align( *aligned, guess );
        std::cout << "." << std::flush;

        double score = registration->getFitnessScore( fitness_score_max_range );
        if( !registration->hasConverged() || score > best_score ) {
            RCLCPP_INFO_STREAM( node_ros->get_logger(), "registration did not converge" );
            continue;
        }

        best_score    = score;
        best_matched  = candidate;
        relative_pose = registration->getFinalTransformation();  // New to candidate
    }

    if( use_loop_closure_consistency_check ) {
        // Save the relative pose and check previous keyframe
        // loop closure hypothesis check. Calculate relative transformation from candidate to its previous keyframe and calculate
        // transformation from new keyframe to candidate to previous keyframe which should be the identity transformation if the loop
        // closure hypothesis is correct.

        pcl::PointCloud<PointT>::Ptr prev_aligned( new pcl::PointCloud<PointT>() );
        Eigen::Matrix4f              rel_pose_candidate_to_prev;
        if( best_matched->prev_edge != nullptr ) {
            rel_pose_candidate_to_prev = best_matched->prev_edge->relative_pose().matrix().cast<float>();
            RCLCPP_INFO_STREAM( node_ros->get_logger(), "prev edge " << gid_generator->getHumanReadableId( best_matched->prev_edge->gid )
                                                                     << " to "
                                                                     << gid_generator->getHumanReadableId( best_matched->prev_edge->to_gid )
                                                                     << " has relative pose\n"
                                                                     << rel_pose_candidate_to_prev );

            Eigen::Matrix4f rel_pose_new_to_prev;
            const auto&     prev_kf = gid_keyframe_map.at( best_matched->prev_edge->to_gid );
            registration->setInputSource( prev_kf->cloud );
            Eigen::Isometry3d prev_kf_estimate = prev_kf->node->estimate();
            prev_kf_estimate.linear()          = Eigen::Quaterniond( prev_kf_estimate.linear() ).normalized().toRotationMatrix();
            Eigen::Matrix4f prev_guess         = ( new_keyframe_estimate.inverse() * prev_kf_estimate ).matrix().cast<float>();
            if( use_planar_registration_guess ) {
                prev_guess( 2, 3 ) = 0.0;
            }
            registration->align( *prev_aligned, prev_guess );
            if( !registration->hasConverged() ) {
                RCLCPP_INFO_STREAM( node_ros->get_logger(), "registration did not converge for prev" );
            }
            rel_pose_new_to_prev = registration->getFinalTransformation();
            // Calculate the transformation from candidate to prev to new keyframe which should be identity matrix
            auto rel_pose_identity_check_prev = rel_pose_new_to_prev.inverse() * relative_pose * rel_pose_candidate_to_prev;
            RCLCPP_INFO_STREAM( node_ros->get_logger(), "identity check matrix for prev\n" << rel_pose_identity_check_prev );
            float delta_trans_prev = rel_pose_identity_check_prev.block<3, 1>( 0, 3 ).norm();
            float delta_angle_prev =
                Eigen::Quaternionf( rel_pose_identity_check_prev.block<3, 3>( 0, 0 ) ).angularDistance( Eigen::Quaternionf::Identity() );
            if( delta_trans_prev < loop_closure_consistency_max_delta_trans
                && delta_angle_prev < loop_closure_consistency_max_delta_angle ) {
                RCLCPP_INFO_STREAM( node_ros->get_logger(), "Consistent with prev keyframe "
                                                                << gid_generator->getHumanReadableId( prev_kf->gid ) << " delta trans "
                                                                << delta_trans_prev << " delta angle " << delta_angle_prev * 180.0 / M_PI );
                consistency_check_passed = true;
            } else {
                RCLCPP_WARN_STREAM( node_ros->get_logger(), "Inconsistent with prev keyframe "
                                                                << gid_generator->getHumanReadableId( prev_kf->gid ) << " delta trans "
                                                                << delta_trans_prev << " delta angle " << delta_angle_prev * 180.0 / M_PI );
            }
        } else {
            RCLCPP_WARN_STREAM( node_ros->get_logger(),
                                "candidate " << gid_generator->getHumanReadableId( best_matched->gid ) << " has no previous edge" );
        }
        // Identity transform consistency check with next keyframe, maybe only done for best score?
        pcl::PointCloud<PointT>::Ptr next_aligned( new pcl::PointCloud<PointT>() );
        Eigen::Matrix4f              rel_pose_next_to_candidate;
        if( best_matched->next_edge != nullptr ) {
            rel_pose_next_to_candidate = best_matched->next_edge->relative_pose().matrix().cast<float>();
            RCLCPP_INFO_STREAM( node_ros->get_logger(), "next edge "
                                                            << gid_generator->getHumanReadableId( best_matched->next_edge->gid ) << " from "
                                                            << gid_generator->getHumanReadableId( best_matched->next_edge->from_gid )
                                                            << " has relative pose\n"
                                                            << rel_pose_next_to_candidate );

            Eigen::Matrix4f rel_pose_new_to_next;
            const auto&     next_kf = gid_keyframe_map.at( best_matched->next_edge->from_gid );
            registration->setInputSource( next_kf->cloud );
            Eigen::Isometry3d next_kf_estimate = next_kf->node->estimate();
            next_kf_estimate.linear()          = Eigen::Quaterniond( next_kf_estimate.linear() ).normalized().toRotationMatrix();
            Eigen::Matrix4f next_guess         = ( new_keyframe_estimate.inverse() * next_kf_estimate ).matrix().cast<float>();
            if( use_planar_registration_guess ) {
                next_guess( 2, 3 ) = 0.0;
            }
            registration->align( *next_aligned, next_guess );
            if( !registration->hasConverged() ) {
                RCLCPP_INFO_STREAM( node_ros->get_logger(), "registration did not converge for next" );
            }
            rel_pose_new_to_next = registration->getFinalTransformation();
            // Calculate the transformation from candidate to next to new keyframe which should be identity matrix
            auto rel_pose_identity_check_next = relative_pose.inverse() * rel_pose_new_to_next * rel_pose_next_to_candidate;
            RCLCPP_INFO_STREAM( node_ros->get_logger(), "identity check matrix for next\n" << rel_pose_identity_check_next );
            float delta_trans_next = rel_pose_identity_check_next.block<3, 1>( 0, 3 ).norm();
            float delta_angle_next =
                Eigen::Quaternionf( rel_pose_identity_check_next.block<3, 3>( 0, 0 ) ).angularDistance( Eigen::Quaternionf::Identity() );
            if( delta_trans_next < loop_closure_consistency_max_delta_trans
                && delta_angle_next < loop_closure_consistency_max_delta_angle ) {
                RCLCPP_INFO_STREAM( node_ros->get_logger(), "Consistent with next keyframe "
                                                                << gid_generator->getHumanReadableId( next_kf->gid ) << " delta trans "
                                                                << delta_trans_next << " delta angle " << delta_angle_next * 180.0 / M_PI );
                consistency_check_passed = true;
            } else {
                RCLCPP_WARN_STREAM( node_ros->get_logger(), "Inconsistent with next keyframe "
                                                                << gid_generator->getHumanReadableId( next_kf->gid ) << " delta trans "
                                                                << delta_trans_next << " delta angle " << delta_angle_next * 180.0 / M_PI );
            }
        } else {
            RCLCPP_WARN_STREAM( node_ros->get_logger(),
                                "candidate " << gid_generator->getHumanReadableId( best_matched->gid ) << " has no next edge" );
        }
    }

    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now() - t1 );
    std::cout << "best_score: " << boost::format( "%.3f" ) % best_score << " time: " << boost::format( "%.3f" ) % elapsed_ms.count()
              << "[msec]" << std::endl;

    if( best_score > fitness_score_thresh ) {
        std::cout << "loop not found..." << std::endl;
        return nullptr;
    }

    if( use_loop_closure_consistency_check && !consistency_check_passed ) {
        std::cout << "didnt pass either consistency check" << std::endl;
        return nullptr;
    }

    std::cout << "loop found! from " << gid_generator->getHumanReadableId( best_matched->gid ) << " to "
              << gid_generator->getHumanReadableId( new_keyframe->gid ) << std::endl;
    std::cout << "relpose: " << relative_pose.block<3, 1>( 0, 3 ).transpose() << " - "
              << Eigen::Quaternionf( relative_pose.block<3, 3>( 0, 0 ) ).coeffs().transpose() << std::endl;

    if( new_keyframe->accum_distance >= 0 ) {
        last_edge_accum_distance = new_keyframe->accum_distance;
    }

    return std::make_shared<Loop>( new_keyframe, best_matched, relative_pose );
}


}  // namespace hdl_graph_slam