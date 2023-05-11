// SPDX-License-Identifier: BSD-2-Clause

#include <hdl_graph_slam/loop_detector.hpp>


namespace hdl_graph_slam {

// LoopDetector::LoopDetector( ros::NodeHandle& pnh )
LoopDetector::LoopDetector( rclcpp::Node::SharedPtr _node ) : node( _node )
{
    // distance_thresh                = pnh.param<double>( "distance_thresh", 5.0 );
    // distance_thresh_squared        = distance_thresh * distance_thresh;
    // accum_distance_thresh          = pnh.param<double>( "accum_distance_thresh", 8.0 );
    // distance_from_last_edge_thresh = pnh.param<double>( "min_edge_interval", 5.0 );
    // fitness_score_max_range = pnh.param<double>( "fitness_score_max_range", std::numeric_limits<double>::max() );
    // fitness_score_thresh    = pnh.param<double>( "fitness_score_thresh", 0.5 );
    // registration             = select_registration_method( pnh );

    distance_thresh                = node->declare_parameter<double>( "distance_thresh", 5.0 );
    distance_thresh_squared        = distance_thresh * distance_thresh;
    accum_distance_thresh          = node->declare_parameter<double>( "accum_distance_thresh", 8.0 );
    distance_from_last_edge_thresh = node->declare_parameter<double>( "min_edge_interval", 5.0 );

    fitness_score_max_range = node->declare_parameter<double>( "fitness_score_max_range", std::numeric_limits<double>::max() );
    // Parameter also used in loop detector, make sure to declare it once
    fitness_score_thresh = node->has_parameter( "fitness_score_thresh" ) ? node->get_parameter( "fitness_score_thresh" ).as_double()
                                                                         : node->declare_parameter<double>( "fitness_score_thresh", 0.5 );

    // TODO pass a raw rclpp::Node pointer to select_registration_method
    registration             = select_registration_method( node.get() );
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
                      hdl_graph_slam::GraphSLAM& graph_slam )
{
    std::vector<Loop::Ptr> detected_loops;
    for( const auto& new_keyframe : new_keyframes ) {
        auto candidates = find_candidates( keyframes, new_keyframe );
        auto loop       = matching( candidates, new_keyframe, graph_slam );
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
        if( new_keyframe->edge_exists( *k ) ) {
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
                        hdl_graph_slam::GraphSLAM& graph_slam )
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
    std::cout << "matching" << std::flush;
    // auto t1 = ros::Time::now();
    auto t1 = node->now();

    pcl::PointCloud<PointT>::Ptr aligned( new pcl::PointCloud<PointT>() );
    for( const auto& candidate : candidate_keyframes ) {
        registration->setInputSource( candidate->cloud );
        Eigen::Isometry3d new_keyframe_estimate = new_keyframe->node->estimate();
        new_keyframe_estimate.linear()          = Eigen::Quaterniond( new_keyframe_estimate.linear() ).normalized().toRotationMatrix();
        Eigen::Isometry3d candidate_estimate    = candidate->node->estimate();
        candidate_estimate.linear()             = Eigen::Quaterniond( candidate_estimate.linear() ).normalized().toRotationMatrix();
        Eigen::Matrix4f guess                   = ( new_keyframe_estimate.inverse() * candidate_estimate ).matrix().cast<float>();
        guess( 2, 3 )                           = 0.0;
        registration->align( *aligned, guess );
        std::cout << "." << std::flush;

        double score = registration->getFitnessScore( fitness_score_max_range );
        if( !registration->hasConverged() || score > best_score ) {
            continue;
        }

        best_score    = score;
        best_matched  = candidate;
        relative_pose = registration->getFinalTransformation();
    }

    // auto t2 = ros::Time::now();
    auto t2 = node->now();
    std::cout << " done" << std::endl;
    std::cout << "best_score: " << boost::format( "%.3f" ) % best_score << "    time: " << boost::format( "%.3f" ) % ( t2 - t1 ).seconds()
              << "[sec]" << std::endl;

    if( best_score > fitness_score_thresh ) {
        std::cout << "loop not found..." << std::endl;
        return nullptr;
    }

    std::cout << "loop found!!" << std::endl;
    std::cout << "relpose: " << relative_pose.block<3, 1>( 0, 3 ).transpose() << " - "
              << Eigen::Quaternionf( relative_pose.block<3, 3>( 0, 0 ) ).coeffs().transpose() << std::endl;

    if( new_keyframe->accum_distance >= 0 ) {
        last_edge_accum_distance = new_keyframe->accum_distance;
    }

    return std::make_shared<Loop>( new_keyframe, best_matched, relative_pose );
}

}  // namespace hdl_graph_slam