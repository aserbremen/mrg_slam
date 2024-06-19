// boost
#include <boost/uuid/uuid_io.hpp>
// mrg_slam
#include <mrg_slam/graph_database.hpp>
// ROS2
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mrg_slam {

GraphDatabase::GraphDatabase( rclcpp::Node::SharedPtr _node, std::shared_ptr<GraphSLAM> _graph_slam ) : graph_slam( _graph_slam )
{
    inf_calclator.reset( new InformationMatrixCalculator( _node ) );

    own_name                             = _node->get_parameter( "own_name" ).as_string();
    fix_first_node                       = _node->get_parameter( "fix_first_node" ).as_bool();
    fix_first_node_adaptive              = _node->get_parameter( "fix_first_node_adaptive" ).as_bool();
    fix_first_node_stddev_vec            = _node->get_parameter( "fix_first_node_stddev" ).as_double_array();
    max_keyframes_per_update             = _node->get_parameter( "max_keyframes_per_update" ).as_int();
    odometry_edge_robust_kernel          = _node->get_parameter( "odometry_edge_robust_kernel" ).as_string();
    odometry_edge_robust_kernel_size     = _node->get_parameter( "odometry_edge_robust_kernel_size" ).as_double();
    loop_closure_edge_robust_kernel      = _node->get_parameter( "loop_closure_edge_robust_kernel" ).as_string();
    loop_closure_edge_robust_kernel_size = _node->get_parameter( "loop_closure_edge_robust_kernel_size" ).as_double();

    // We start the odom_keyframe_counter at 1, because the first keyframe (0) is the anchor keyframe
    odom_keyframe_counter = 1;

    anchor_node     = nullptr;
    anchor_edge_g2o = nullptr;
    anchor_kf       = nullptr;
    anchor_edge_ptr = nullptr;

    prev_robot_keyframe = nullptr;
}

void
GraphDatabase::add_odom_keyframe( const builtin_interfaces::msg::Time &stamp, const Eigen::Isometry3d &odom, double accum_distance,
                                  pcl::PointCloud<PointT>::ConstPtr cloud, sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg )
{
    auto        uuid     = uuid_generator();
    std::string uuid_str = boost::uuids::to_string( uuid );

    KeyFrame::Ptr kf = std::make_shared<KeyFrame>( own_name, stamp, odom, odom_keyframe_counter, accum_distance, uuid, uuid_str, cloud,
                                                   cloud_msg );
    odom_keyframe_counter++;

    std::lock_guard<std::mutex> lock( keyframe_queue_mutex );
    keyframe_queue.push_back( kf );
}

bool
GraphDatabase::flush_keyframe_queue( const Eigen::Isometry3d &odom2map )
{
    std::lock_guard<std::mutex> lock( keyframe_queue_mutex );

    if( keyframe_queue.empty() ) {
        return false;
    }

    int num_processed = 0;
    for( int i = 0; i < std::min<int>( keyframe_queue.size(), max_keyframes_per_update ); i++ ) {
        num_processed = i;

        const auto &keyframe = keyframe_queue[i];
        // new_keyframes will be tested later for loop closure
        new_keyframes.push_back( keyframe );

        // add pose node
        Eigen::Isometry3d odom            = odom2map * keyframe->odom;
        keyframe->node                    = graph_slam->add_se3_node( odom );
        uuid_keyframe_map[keyframe->uuid] = keyframe;
        keyframe_hash[keyframe->stamp]    = keyframe;

        // first keyframe?
        if( keyframes.empty() && new_keyframes.size() == 1 ) {
            keyframe->first_keyframe = true;  // exclude point cloud of first keyframe from map, because points corresponding to
                                              // other robots have not been filtered for this keyframe

            // fix the first node
            if( fix_first_node ) {
                Eigen::MatrixXd information = Eigen::MatrixXd::Identity( 6, 6 );
                for( int i = 0; i < 6; i++ ) {
                    information( i, i ) = 1.0 / ( fix_first_node_stddev_vec[i] * fix_first_node_stddev_vec[i] );
                }
                RCLCPP_INFO_STREAM( rclcpp::get_logger( "flush_keyframe_queue" ), "fixing first node with information:\n" << information );

                anchor_node = graph_slam->add_se3_node( Eigen::Isometry3d::Identity() );
                anchor_node->setFixed( true );

                auto        uuid     = uuid_generator();
                std::string uuid_str = boost::uuids::to_string( uuid );
                anchor_kf = std::make_shared<KeyFrame>( own_name, rclcpp::Time(), Eigen::Isometry3d::Identity(), 0, -1, uuid, uuid_str,
                                                        nullptr );
                if( fix_first_node_adaptive ) {
                    // TODO if the anchor node is adaptive, handling needs to be implemented
                }
                anchor_kf->node                    = anchor_node;
                uuid_keyframe_map[anchor_kf->uuid] = anchor_kf;

                anchor_edge_g2o = graph_slam->add_se3_edge( anchor_node, keyframe->node, keyframe->node->estimate(), information );
                anchor_edge_ptr = std::make_shared<Edge>( anchor_edge_g2o, Edge::TYPE_ANCHOR, uuid_generator(), anchor_kf, anchor_kf->uuid,
                                                          keyframe, keyframe->uuid );
                edges.emplace_back( anchor_edge_ptr );
                edge_uuids.insert( anchor_edge_ptr->uuid );
            }
        }

        if( i == 0 && keyframes.empty() ) {
            prev_robot_keyframe = keyframe;
            continue;
        }

        // add edge between consecutive keyframes
        Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_robot_keyframe->odom;
        Eigen::MatrixXd information = inf_calclator->calc_information_matrix( keyframe->cloud, prev_robot_keyframe->cloud, relative_pose );
        auto            graph_edge  = graph_slam->add_se3_edge( keyframe->node, prev_robot_keyframe->node, relative_pose, information );
        Edge::Ptr       edge( new Edge( graph_edge, Edge::TYPE_ODOM, uuid_generator(), keyframe, keyframe->uuid, prev_robot_keyframe,
                                        prev_robot_keyframe->uuid ) );
        edges.emplace_back( edge );
        edge_uuids.insert( edge->uuid );
        // Add the edge to the corresponding keyframes
        RCLCPP_INFO_STREAM( rclcpp::get_logger( "flush_keyframe_queue" ),
                            "added " << edge->readable_id << " as next edge to keyframe " << prev_robot_keyframe->readable_id );
        uuid_keyframe_map[prev_robot_keyframe->uuid]->next_edge = edge;
        RCLCPP_INFO_STREAM( rclcpp::get_logger( "flush_keyframe_queue" ),
                            "added " << edge->readable_id << " as prev edge to keyframe " << keyframe->readable_id );
        keyframe->prev_edge = edge;
        graph_slam->add_robust_kernel( graph_edge, odometry_edge_robust_kernel, odometry_edge_robust_kernel_size );
        prev_robot_keyframe = keyframe;
    }

    // TODO remove read_until ?
    // std_msgs::msg::Header read_until;
    // read_until.stamp =
    //     ( rclcpp::Time( keyframe_queue[num_processed]->stamp ) + rclcpp::Duration( 10, 0 ) ).operator builtin_interfaces::msg::Time();
    // read_until.frame_id = points_topic;
    // read_until_pub->publish( read_until );
    // read_until.frame_id = "/filtered_points";
    // read_until_pub->publish( read_until );

    keyframe_queue.erase( keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1 );
    return true;
}

void
GraphDatabase::add_graph_ros( const mrg_slam_msgs::msg::GraphRos &graph_ros )
{
    std::lock_guard<std::mutex> lock( keyframe_queue_mutex );
    graph_queue.push_back( std::make_shared<mrg_slam_msgs::msg::GraphRos>( std::move( graph_ros ) ) );
}

bool
GraphDatabase::flush_graph_queue(
    std::unordered_map<std::string, std::pair<KeyFrame::ConstPtr, Eigen::Isometry3d>> &others_prev_robot_keyframes )
{
    //     std::lock_guard<std::mutex> lock( graph_queue_mutex );

    if( graph_queue.empty() || keyframes.empty() ) {
        return false;
    }

    RCLCPP_INFO_STREAM( rclcpp::get_logger( "flush_graph_queue" ), "Flusing graph, received graph msgs: " << graph_queue.size() );

    // Create unique keyframes and edges vectors to keep order of received messages
    std::vector<const mrg_slam_msgs::msg::KeyFrameRos *> unique_keyframes;
    std::vector<const mrg_slam_msgs::msg::EdgeRos *>     unique_edges;

    std::unordered_map<std::string, std::pair<boost::uuids::uuid, const geometry_msgs::msg::Pose *>> latest_robot_keyframes;

    for( const auto &graph_msg : graph_queue ) {
        for( const auto &edge_ros : graph_msg->edges ) {
            boost::uuids::uuid edge_uuid = uuid_from_string_generator( edge_ros.uuid_str );
            if( edge_uuids.find( edge_uuid ) != edge_uuids.end() ) {
                continue;
            }
            if( edge_ignore_uuids.find( edge_uuid ) != edge_ignore_uuids.end() ) {
                continue;
            }

            unique_edges.push_back( &edge_ros );
        }

        for( const auto &keyframe_ros : graph_msg->keyframes ) {
            boost::uuids::uuid keyframe_uuid = uuid_from_string_generator( keyframe_ros.uuid_str );
            if( uuid_keyframe_map.find( keyframe_uuid ) != uuid_keyframe_map.end() ) {
                continue;
            }

            unique_keyframes.push_back( &keyframe_ros );
        }

        latest_robot_keyframes[graph_msg->robot_name] = std::make_pair<boost::uuids::uuid, const geometry_msgs::msg::Pose *>(
            uuid_from_string_generator( graph_msg->latest_keyframe_uuid_str ), &graph_msg->latest_keyframe_odom );
    }

    if( unique_keyframes.empty() || unique_edges.empty() ) {
        graph_queue.clear();
        return false;
    }

    for( const auto &keyframe_ros : unique_keyframes ) {
        pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT>() );
        pcl::fromROSMsg( keyframe_ros->cloud, *cloud );
        sensor_msgs::msg::PointCloud2::SharedPtr cloud_ros = std::make_shared<sensor_msgs::msg::PointCloud2>( keyframe_ros->cloud );

        auto          uuid     = uuid_from_string_generator( keyframe_ros->uuid_str );
        KeyFrame::Ptr keyframe = std::make_shared<KeyFrame>( keyframe_ros->robot_name, keyframe_ros->stamp, Eigen::Isometry3d::Identity(),
                                                             keyframe_ros->odom_counter, keyframe_ros->accum_distance, uuid,
                                                             keyframe_ros->uuid_str, std::move( cloud ), std::move( cloud_ros ) );

        Eigen::Isometry3d pose;
        tf2::fromMsg( keyframe_ros->estimate, pose );
        keyframe->node                    = graph_slam->add_se3_node( pose );
        keyframe->first_keyframe          = keyframe_ros->first_keyframe;
        uuid_keyframe_map[keyframe->uuid] = keyframe;
        new_keyframes.push_back( keyframe );  // new_keyframes will be tested later for loop closure
                                              // don't add it to keyframe_hash, which is only used for floor_coeffs
                                              // keyframe_hash[keyframe->stamp] = keyframe;

        RCLCPP_INFO_STREAM( rclcpp::get_logger( "flush_graph_queue" ), "Adding unique keyframe: " << keyframe->readable_id );
    }

    for( const auto &edge_ros : unique_edges ) {
        auto          edge_uuid      = uuid_from_string_generator( edge_ros->uuid_str );
        auto          edge_from_uuid = uuid_from_string_generator( edge_ros->from_uuid_str );
        auto          edge_to_uuid   = uuid_from_string_generator( edge_ros->to_uuid_str );
        KeyFrame::Ptr from_keyframe;
        if( edge_ros->type == Edge::TYPE_ANCHOR ) {
            RCLCPP_INFO_STREAM( rclcpp::get_logger( "flush_graph_queue" ), "Handling anchor edge" );
            from_keyframe = anchor_kf;
        } else {
            from_keyframe = uuid_keyframe_map[edge_from_uuid];
        }
        KeyFrame::Ptr to_keyframe = uuid_keyframe_map[edge_to_uuid];

        // check if the edge is already added
        if( from_keyframe->edge_exists( *to_keyframe, rclcpp::get_logger( "flush_graph_queue" ) ) ) {
            edge_ignore_uuids.insert( edge_uuid );
            continue;
        }

        Eigen::Isometry3d relpose;
        tf2::fromMsg( edge_ros->relative_pose, relpose );
        Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> information( edge_ros->information.data() );
        auto      graph_edge = graph_slam->add_se3_edge( from_keyframe->node, to_keyframe->node, relpose, information );
        Edge::Ptr edge       = std::make_shared<Edge>( graph_edge, static_cast<Edge::Type>( edge_ros->type ), edge_uuid, from_keyframe,
                                                       from_keyframe->uuid, to_keyframe, to_keyframe->uuid );
        edges.emplace_back( edge );
        edge_uuids.insert( edge->uuid );


        RCLCPP_INFO_STREAM( rclcpp::get_logger( "flush_graph_queue" ), "Adding unique edge: " << edge->readable_id );

        // Add odometry edges to the corresponding keyframes as prev or next edge
        if( edge->type == Edge::TYPE_ODOM ) {
            // Only set the prev edge for 2nd and later keyframes
            if( from_keyframe->odom_keyframe_counter > 1 ) {
                from_keyframe->prev_edge = edge;
                RCLCPP_INFO_STREAM( rclcpp::get_logger( "flush_graph_queue" ),
                                    "Setting edge " << edge->readable_id << " as prev edge to keyframe " << from_keyframe->readable_id );
            }
            to_keyframe->next_edge = edge;
            RCLCPP_INFO_STREAM( rclcpp::get_logger( "flush_graph_queue" ),
                                "Setting edge " << edge->readable_id << " as next edge to keyframe " << to_keyframe->readable_id );
        }


        if( edge->type == Edge::TYPE_ODOM || edge->type == Edge::TYPE_ANCHOR ) {
            graph_slam->add_robust_kernel( graph_edge, odometry_edge_robust_kernel, odometry_edge_robust_kernel_size );
        } else if( edge->type == Edge::TYPE_LOOP ) {
            graph_slam->add_robust_kernel( graph_edge, loop_closure_edge_robust_kernel, loop_closure_edge_robust_kernel_size );
        }
    }

    for( const auto &latest_keyframe : latest_robot_keyframes ) {
        auto &kf = others_prev_robot_keyframes[latest_keyframe.first];
        kf.first = uuid_keyframe_map[latest_keyframe.second.first];  // pointer to keyframe
        tf2::fromMsg( *latest_keyframe.second.second, kf.second );   // odometry
    }

    graph_queue.clear();
    return true;
}


void
GraphDatabase::insert_loops( const std::vector<Loop::Ptr> &loops )
{
    std::lock_guard<std::mutex> lock( keyframe_queue_mutex );

    for( const auto &loop : loops ) {
        Eigen::Isometry3d relpose( loop->relative_pose.cast<double>() );
        Eigen::MatrixXd   information_matrix = inf_calclator->calc_information_matrix( loop->key1->cloud, loop->key2->cloud, relpose );
        auto              graph_edge         = graph_slam->add_se3_edge( loop->key1->node, loop->key2->node, relpose, information_matrix );

        auto        uuid     = uuid_generator();
        std::string uuid_str = boost::uuids::to_string( uuid );

        Edge::Ptr edge(
            new Edge( graph_edge, Edge::TYPE_LOOP, uuid_generator(), loop->key1, loop->key1->uuid, loop->key2, loop->key2->uuid ) );
        edges.emplace_back( edge );
        edge_uuids.insert( edge->uuid );
        graph_slam->add_robust_kernel( graph_edge, loop_closure_edge_robust_kernel, loop_closure_edge_robust_kernel_size );
    }

    // Add the new keyframes that have been checked for loop closures to the keyframes vector
    std::copy( new_keyframes.begin(), new_keyframes.end(), std::back_inserter( keyframes ) );
    new_keyframes.clear();
}

}  // namespace mrg_slam