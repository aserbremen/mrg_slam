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

    double          best_score = std::numeric_limits<double>::max();
    KeyFrame::Ptr   best_matched;
    Eigen::Matrix4f relative_pose;

    std::cout << std::endl;
    std::cout << "--- loop detection ---" << std::endl;
    std::cout << "num_candidates: " << candidate_keyframes.size() << std::endl;
    for( const auto& cand : candidate_keyframes ) {
        std::cout << " " << gid_generator->getHumanReadableId( cand->gid ) << std::endl;
    }
    std::cout << "matching" << std::endl;
    auto t1 = std::chrono::system_clock::now();

    std::map<uint64_t, std::pair<uint64_t, Eigen::Matrix4f>> relative_poses;

    pcl::PointCloud<PointT>::Ptr aligned( new pcl::PointCloud<PointT>() );
    for( const auto& candidate : candidate_keyframes ) {
        RCLCPP_INFO_STREAM( node_ros->get_logger(), "candidate: " << gid_generator->getHumanReadableId( candidate->gid ) );
        registration->setInputSource( candidate->cloud );
        Eigen::Isometry3d new_keyframe_estimate = new_keyframe->node->estimate();
        new_keyframe_estimate.linear()          = Eigen::Quaterniond( new_keyframe_estimate.linear() ).normalized().toRotationMatrix();
        Eigen::Isometry3d candidate_estimate    = candidate->node->estimate();
        candidate_estimate.linear()             = Eigen::Quaterniond( candidate_estimate.linear() ).normalized().toRotationMatrix();
        Eigen::Matrix4f guess                   = ( new_keyframe_estimate.inverse() * candidate_estimate ).matrix().cast<float>();
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


        // Save the relative pose and check previous keyframe
        // loop closure hypothesis check. Calculate relative transformation from candidate to its previous keyframe and calculate
        // transformation from new keyframe to candidate to previous keyframe which should be the identity transformation if the loop
        // closure hypothesis is correct.

        // Identity transform consistency check, maybe only done for best score?
        // TODO handle the anchor node case??
        pcl::PointCloud<PointT>::Ptr prev_aligned( new pcl::PointCloud<PointT>() );
        Eigen::Matrix4f              rel_pose_candidate_to_prev;
        if( candidate->prev_edge != nullptr ) {
            rel_pose_candidate_to_prev = candidate->prev_edge->relative_pose().matrix().cast<float>();
        } else {
            RCLCPP_WARN_STREAM( node_ros->get_logger(),
                                "candidate " << gid_generator->getHumanReadableId( candidate->gid ) << " has no previous edge" );
            rel_pose_candidate_to_prev = Eigen::Matrix4f::Identity();
            continue;
        }
        RCLCPP_INFO_STREAM( node_ros->get_logger(), "prev edge " << gid_generator->getHumanReadableId( candidate->prev_edge->gid ) << " to "
                                                                 << gid_generator->getHumanReadableId( candidate->prev_edge->to_gid )
                                                                 << " has relative pose\n"
                                                                 << rel_pose_candidate_to_prev );

        Eigen::Matrix4f rel_pose_new_to_prev;
        const auto&     prev_kf = gid_keyframe_map.at( candidate->prev_edge->to_gid );
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
            continue;
        }
        rel_pose_new_to_prev = registration->getFinalTransformation();
        // Calculate the transformation from candidate to prev to new keyframe which should be identity matrix
        auto rel_pose_identity_check = rel_pose_new_to_prev.inverse() * relative_pose * rel_pose_candidate_to_prev;
        RCLCPP_INFO_STREAM( node_ros->get_logger(), "identity check matrix for prev\n" << rel_pose_identity_check );
    }

    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now() - t1 );
    std::cout << " done" << std::endl;
    std::cout << "best_score: " << boost::format( "%.3f" ) % best_score << " time: " << boost::format( "%.3f" ) % elapsed_ms.count()
              << "[msec]" << std::endl;

    if( best_score > fitness_score_thresh ) {
        std::cout << "loop not found..." << std::endl;
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