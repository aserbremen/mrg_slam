// boost
#include <boost/filesystem.hpp>
#include <boost/uuid/uuid_io.hpp>
// mrg_slam
#include <mrg_slam/graph_database.hpp>
#include <mrg_slam/ros_utils.hpp>
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
    result_dir                           = _node->get_parameter( "result_dir" ).as_string();
    if( result_dir.back() == '/' ) {
        result_dir.pop_back();
    }
    // We start the odom_keyframe_counter at 1, because the first keyframe (0) is the anchor keyframe
    odom_keyframe_counter = 1;

    anchor_node     = nullptr;
    anchor_edge_g2o = nullptr;
    anchor_kf       = nullptr;
    anchor_edge_ptr = nullptr;

    prev_robot_keyframe = nullptr;

    // Generate a unique id for the graph of the current run and robot
    slam_uuid     = uuid_generator();
    slam_uuid_str = boost::uuids::to_string( slam_uuid );
}

void
GraphDatabase::add_odom_keyframe( const builtin_interfaces::msg::Time &stamp, const Eigen::Isometry3d &odom, double accum_distance,
                                  pcl::PointCloud<PointT>::ConstPtr cloud, sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg )
{
    auto        uuid     = uuid_generator();
    std::string uuid_str = boost::uuids::to_string( uuid );

    KeyFrame::Ptr kf = std::make_shared<KeyFrame>( own_name, stamp, odom, odom_keyframe_counter, accum_distance, uuid, uuid_str, slam_uuid,
                                                   slam_uuid_str, cloud, cloud_msg );
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

    auto logger = rclcpp::get_logger( "flush_keyframe_queue" );

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
                RCLCPP_INFO_STREAM( logger, "fixing first node with information:\n" << information );

                anchor_node = graph_slam->add_se3_node( Eigen::Isometry3d::Identity() );
                anchor_node->setFixed( true );

                auto        uuid_anchor_kf     = uuid_generator();
                std::string uuid_anchor_kf_str = boost::uuids::to_string( uuid_anchor_kf );

                anchor_kf = std::make_shared<KeyFrame>( own_name, rclcpp::Time(), Eigen::Isometry3d::Identity(), 0, -1, uuid_anchor_kf,
                                                        uuid_anchor_kf_str, slam_uuid, slam_uuid_str, nullptr );
                if( fix_first_node_adaptive ) {
                    // TODO if the anchor node is adaptive, handling needs to be implemented
                }
                anchor_kf->node                    = anchor_node;
                uuid_keyframe_map[anchor_kf->uuid] = anchor_kf;

                anchor_edge_g2o = graph_slam->add_se3_edge( anchor_node, keyframe->node, keyframe->node->estimate(), information );

                auto        uuid_anchor_edge     = uuid_generator();
                std::string uuid_anchor_edge_str = boost::uuids::to_string( uuid_anchor_edge );

                anchor_edge_ptr = std::make_shared<Edge>( anchor_edge_g2o, Edge::TYPE_ANCHOR, uuid_anchor_edge, uuid_anchor_edge_str,
                                                          anchor_kf, keyframe );
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
        auto            g2o_edge    = graph_slam->add_se3_edge( keyframe->node, prev_robot_keyframe->node, relative_pose, information );

        auto        uuid     = uuid_generator();
        std::string uuid_str = boost::uuids::to_string( uuid );

        Edge::Ptr edge = std::make_shared<Edge>( g2o_edge, Edge::TYPE_ODOM, uuid, uuid_str, keyframe, prev_robot_keyframe );
        edges.emplace_back( edge );
        edge_uuids.insert( edge->uuid );
        // Add the edge to the corresponding keyframes
        RCLCPP_INFO_STREAM( logger, "added " << edge->readable_id << " as next edge to keyframe " << prev_robot_keyframe->readable_id );
        uuid_keyframe_map[prev_robot_keyframe->uuid]->next_edge = edge;
        RCLCPP_INFO_STREAM( logger, "added " << edge->readable_id << " as prev edge to keyframe " << keyframe->readable_id );
        keyframe->prev_edge = edge;
        graph_slam->add_robust_kernel( g2o_edge, odometry_edge_robust_kernel, odometry_edge_robust_kernel_size );
        prev_robot_keyframe = keyframe;
    }
    keyframe_queue.erase( keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1 );

    // TODO remove read_until ?
    // std_msgs::msg::Header read_until;
    // read_until.stamp =
    //     ( rclcpp::Time( keyframe_queue[num_processed]->stamp ) + rclcpp::Duration( 10, 0 ) ).operator builtin_interfaces::msg::Time();
    // read_until.frame_id = points_topic;
    // read_until_pub->publish( read_until );
    // read_until.frame_id = "/filtered_points";
    // read_until_pub->publish( read_until );

    return true;
}

void
GraphDatabase::add_static_keyframes( const std::vector<mrg_slam_msgs::msg::KeyFrameRos> &keyframes )
{
    std::lock_guard<std::mutex> lock_static( static_keyframe_queue_mutex );


    auto logger = rclcpp::get_logger( "add_static_keyframes" );

    for( const auto &keyframe_ros : keyframes ) {
        auto uuid = uuid_from_string_generator( keyframe_ros.uuid_str );
        // Check if the static keyframe is already in the graph
        if( uuid_keyframe_map.find( uuid ) != uuid_keyframe_map.end()
            || std::find_if( static_keyframe_queue.begin(), static_keyframe_queue.end(), [uuid]( const KeyFrame::Ptr &keyframe ) {
                   return keyframe->uuid == uuid;
               } ) != static_keyframe_queue.end() ) {
            RCLCPP_INFO_STREAM( logger, "Static keyframe " << keyframe_ros.uuid_str << " already in the graph or queue" );
            continue;
        }

        pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT>() );
        pcl::fromROSMsg( keyframe_ros.cloud, *cloud );
        sensor_msgs::msg::PointCloud2::SharedPtr cloud_ros = std::make_shared<sensor_msgs::msg::PointCloud2>( keyframe_ros.cloud );

        KeyFrame::Ptr keyframe    = std::make_shared<KeyFrame>( keyframe_ros.robot_name, keyframe_ros.stamp, Eigen::Isometry3d::Identity(),
                                                                keyframe_ros.odom_counter, keyframe_ros.accum_distance, uuid,
                                                                keyframe_ros.uuid_str, slam_uuid, slam_uuid_str, std::move( cloud ),
                                                                std::move( cloud_ros ) );
        keyframe->static_keyframe = true;
        keyframe->estimate_transform = pose2isometry( keyframe_ros.estimate );

        static_keyframe_queue.push_back( keyframe );
    }
}


bool
GraphDatabase::flush_static_keyframe_queue()
{
    std::lock_guard<std::mutex> lock_static( static_keyframe_queue_mutex );

    if( static_keyframe_queue.empty() ) {
        return false;
    }

    auto logger = rclcpp::get_logger( "flush_static_keyframe_queue" );

    // Add static keyframes to the graph. Static keyframes are not connected to any other keyframes.
    // They are not added to new_keyframes, because they cannot have loop closures among themselves.
    // However, new_keyframes will also be tested for loop closure against static keyframes.
    for( const auto &static_keyframe : static_keyframe_queue ) {
        // add pose node
        RCLCPP_INFO_STREAM( logger, "Adding static keyframe: " << static_keyframe->readable_id << " with center "
                                                               << static_keyframe->estimate_transform.translation().transpose() );
        static_keyframe->node = graph_slam->add_se3_node( static_keyframe->estimate_transform );
        static_keyframe->node->setFixed( true );
        uuid_keyframe_map[static_keyframe->uuid] = static_keyframe;
        keyframe_hash[static_keyframe->stamp]    = static_keyframe;
        keyframes.push_back( static_keyframe );
    }

    static_keyframe_queue.clear();

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
    std::unordered_map<std::string, std::pair<KeyFrame::ConstPtr, Eigen::Isometry3d>> &others_prev_robot_keyframes,
    LoopManager::Ptr                                                                   loop_manager )
{
    //     std::lock_guard<std::mutex> lock( graph_queue_mutex );

    if( graph_queue.empty() || keyframes.empty() ) {
        return false;
    }

    auto logger = rclcpp::get_logger( "flush_graph_queue" );

    RCLCPP_INFO_STREAM( logger, "Flushing graph, received graph msgs: " << graph_queue.size() );

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

        auto          uuid          = uuid_from_string_generator( keyframe_ros->uuid_str );
        auto          instance_uuid = uuid_from_string_generator( keyframe_ros->slam_uuid_str );
        KeyFrame::Ptr keyframe = std::make_shared<KeyFrame>( keyframe_ros->robot_name, keyframe_ros->stamp, Eigen::Isometry3d::Identity(),
                                                             keyframe_ros->odom_counter, keyframe_ros->accum_distance, uuid,
                                                             keyframe_ros->uuid_str, instance_uuid, keyframe_ros->slam_uuid_str,
                                                             std::move( cloud ), std::move( cloud_ros ) );

        Eigen::Isometry3d pose;
        tf2::fromMsg( keyframe_ros->estimate, pose );
        keyframe->node                    = graph_slam->add_se3_node( pose );
        keyframe->first_keyframe          = keyframe_ros->first_keyframe;
        uuid_keyframe_map[keyframe->uuid] = keyframe;

        RCLCPP_INFO_STREAM( logger, "Adding unique keyframe: " << keyframe->readable_id );
        // Immediately add static keyframes to keyframes, since they are not connected to other keyframes.
        // New keyframes will be tested for loop closure against static keyframes. Static keyframes are not added to new_keyframes.
        if( keyframe_ros->static_keyframe ) {
            keyframe->static_keyframe = keyframe_ros->static_keyframe;
            keyframe->node->setFixed( true );
            keyframe->estimate_transform = pose;
            keyframes.push_back( keyframe );
            continue;
        }
        // new_keyframes will be tested later for loop closure
        // don't add it to keyframe_hash, which is only used for floor_coeffs
        // keyframe_hash[keyframe->stamp] = keyframe;
        new_keyframes.push_back( keyframe );
    }

    for( const auto &edge_ros : unique_edges ) {
        auto          edge_uuid      = uuid_from_string_generator( edge_ros->uuid_str );
        auto          edge_from_uuid = uuid_from_string_generator( edge_ros->from_uuid_str );
        auto          edge_to_uuid   = uuid_from_string_generator( edge_ros->to_uuid_str );
        KeyFrame::Ptr from_keyframe;
        if( edge_ros->type == Edge::TYPE_ANCHOR ) {
            RCLCPP_INFO_STREAM( logger, "Handling anchor edge" );
            from_keyframe = anchor_kf;
        } else {
            from_keyframe = uuid_keyframe_map[edge_from_uuid];
        }
        KeyFrame::Ptr to_keyframe = uuid_keyframe_map[edge_to_uuid];

        // check if the edge is already added
        if( from_keyframe->edge_exists( *to_keyframe, logger ) ) {
            edge_ignore_uuids.insert( edge_uuid );
            continue;
        }

        Eigen::Isometry3d relpose;
        tf2::fromMsg( edge_ros->relative_pose, relpose );
        Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> information( edge_ros->information.data() );
        auto      g2o_edge = graph_slam->add_se3_edge( from_keyframe->node, to_keyframe->node, relpose, information );
        Edge::Ptr edge     = std::make_shared<Edge>( g2o_edge, static_cast<Edge::Type>( edge_ros->type ), edge_uuid, edge_ros->uuid_str,
                                                     from_keyframe, to_keyframe );
        edges.emplace_back( edge );
        edge_uuids.insert( edge->uuid );


        RCLCPP_INFO_STREAM( logger, "Adding unique edge: " << edge->readable_id );

        // Add odometry edges to the corresponding keyframes as prev or next edge
        if( edge->type == Edge::TYPE_ODOM ) {
            // Only set the prev edge for 2nd and later keyframes
            if( from_keyframe->odom_keyframe_counter > 1 ) {
                from_keyframe->prev_edge = edge;
                RCLCPP_INFO_STREAM( logger,
                                    "Setting edge " << edge->readable_id << " as prev edge to keyframe " << from_keyframe->readable_id );
            }
            to_keyframe->next_edge = edge;
            RCLCPP_INFO_STREAM( logger, "Setting edge " << edge->readable_id << " as next edge to keyframe " << to_keyframe->readable_id );
        }


        if( edge->type == Edge::TYPE_ODOM || edge->type == Edge::TYPE_ANCHOR ) {
            graph_slam->add_robust_kernel( g2o_edge, odometry_edge_robust_kernel, odometry_edge_robust_kernel_size );
        } else if( edge->type == Edge::TYPE_LOOP ) {
            Eigen::Matrix4f relative_pose_matrix = edge->relative_pose().matrix().cast<float>();
            Loop::Ptr       loop                 = std::make_shared<Loop>( from_keyframe, to_keyframe, relative_pose_matrix );
            loop_manager->add_loop_accum_distance_check( loop );
            graph_slam->add_robust_kernel( g2o_edge, loop_closure_edge_robust_kernel, loop_closure_edge_robust_kernel_size );
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


bool
GraphDatabase::load_graph( const std::string &directory )
{
    auto logger = rclcpp::get_logger( "load_graph" );
    // lambda function to glob all files with a specific extension in a directory
    auto glob_abs_paths = []( boost::filesystem::path const &root, std::string const &ext ) -> std::vector<boost::filesystem::path> {
        std::vector<boost::filesystem::path> paths;
        if( boost::filesystem::exists( root ) && boost::filesystem::is_directory( root ) ) {
            for( auto const &entry : boost::filesystem::recursive_directory_iterator( root ) ) {
                if( boost::filesystem::is_regular_file( entry ) && entry.path().extension() == ext ) {
                    paths.emplace_back( entry.path() );
                }
            }
        }

        std::sort( paths.begin(), paths.end() );
        return paths;
    };
    // glob all the keyframe files ending on .txt and and point clouds ending on .pcd
    boost::filesystem::path keyframe_dir( directory );
    keyframe_dir /= "keyframes";
    if( !boost::filesystem::is_directory( keyframe_dir ) ) {
        RCLCPP_WARN_STREAM( logger, "Directory " << keyframe_dir << " does not exist, cannot load keyframes" );
        return false;
    }
    auto keyframe_files   = glob_abs_paths( keyframe_dir, ".txt" );
    auto pointcloud_files = glob_abs_paths( keyframe_dir, ".pcd" );
    if( keyframe_files.size() != pointcloud_files.size() ) {
        RCLCPP_WARN_STREAM( logger, "Number of keyframe files and pointcloud files do not match, cannot load graph" );
        return false;
    }
    // glob all the edge files ending on .txt
    boost::filesystem::path edge_dir = boost::filesystem::path( directory );
    edge_dir /= "edges";
    if( !boost::filesystem::is_directory( edge_dir ) ) {
        RCLCPP_WARN_STREAM( logger, "Directory " << edge_dir << " does not exist, cannot load edges" );
        return false;
    }
    auto edge_files = glob_abs_paths( edge_dir, ".txt" );

    // lambda function to load uuid_str from a file and check if the keyframe or edge is already in the graph
    auto load_string_key = []( const std::string &path, const std::string &_key ) -> std::string {
        std::ifstream ifs( path );
        std::string   line;
        while( std::getline( ifs, line ) ) {
            std::stringstream iss( line );
            std::string       key;
            iss >> key;
            if( key == _key ) {
                std::string str;
                iss >> str;
                return str;
            }
        }
        return "";
    };

    // load all the keyframes, point clouds and edges, which will be added to the pose graph in the next optimization
    std::lock_guard<std::mutex> lock( loaded_graph_mutex );
    for( size_t i = 0; i < keyframe_files.size(); i++ ) {
        std::string keyframe_file   = keyframe_files[i].string();
        std::string pointcloud_file = pointcloud_files[i].string();
        std::string uuid_str        = load_string_key( keyframe_file, "uuid_str" );
        auto        uuid            = uuid_from_string_generator( uuid_str );
        if( uuid_keyframe_map.find( uuid ) != uuid_keyframe_map.end() ) {
            RCLCPP_WARN_STREAM( logger, "Keyframe with uuid " << uuid_str << " already exists, skipping" );
            continue;
        }

        KeyFrame::Ptr keyframe = std::make_shared<KeyFrame>( keyframe_file, pointcloud_file, uuid, uuid_str );
        loaded_keyframes.emplace_back( keyframe );
    }

    for( size_t i = 0; i < edge_files.size(); i++ ) {
        std::string edge_file = edge_files[i].string();
        std::string uuid_str  = load_string_key( edge_file, "uuid_str" );
        auto        uuid      = uuid_from_string_generator( uuid_str );
        if( edge_uuids.find( uuid ) != edge_uuids.end() ) {
            RCLCPP_WARN_STREAM( logger, "Edge with uuid " << uuid_str << " already exists, skipping" );
            continue;
        }
        std::string from_uid_str = load_string_key( edge_file, "from_uuid_str" );
        std::string to_uid_str   = load_string_key( edge_file, "to_uuid_str" );
        auto        from_uuid    = uuid_from_string_generator( from_uid_str );
        auto        to_uuid      = uuid_from_string_generator( to_uid_str );

        Edge::Ptr edge = std::make_shared<Edge>( edge_file, uuid, uuid_str, from_uuid, from_uid_str, to_uuid, to_uid_str );
        loaded_edges.emplace_back( edge );
    }

    return true;
}

bool
GraphDatabase::flush_loaded_graph( LoopManager::Ptr loop_manager )
{
    std::lock_guard<std::mutex> lock( loaded_graph_mutex );

    if( loaded_keyframes.empty() && loaded_edges.empty() ) {
        return false;
    }

    auto logger = rclcpp::get_logger( "flush_loaded_graph" );

    RCLCPP_INFO_STREAM( logger, "Flushing loaded " << loaded_keyframes.size() << " keyframes and " << loaded_edges.size() << " edges" );

    for( const auto &keyframe : loaded_keyframes ) {
        keyframe->node                    = graph_slam->add_se3_node( keyframe->estimate_transform );
        uuid_keyframe_map[keyframe->uuid] = keyframe;
        if( keyframe->static_keyframe ) {
            keyframe->node->setFixed( true );
            keyframes.push_back( keyframe );
        } else {
            new_keyframes.push_back( keyframe );  // new_keyframes will be tested later for loop closure don't add it to keyframe_hash,
                                                  // which is only used for floor_coeffs keyframe_hash[keyframe->stamp] = keyframe;
        }
        RCLCPP_INFO_STREAM( logger, "Adding keyframe: " << keyframe->readable_id );
    }

    for( const auto &edge : loaded_edges ) {
        RCLCPP_INFO_STREAM( logger, "Adding edge: " << edge->readable_id );
        KeyFrame::Ptr from_keyframe;
        // TODO check if anchor keyframe is setup correctly
        if( edge->type == Edge::TYPE_ANCHOR ) {
            RCLCPP_INFO_STREAM( logger, "Handling anchor edge, setting from keyframe to anchor keyframe" );
            from_keyframe = anchor_kf;
        } else {
            from_keyframe = uuid_keyframe_map[edge->from_uuid];
        }
        KeyFrame::Ptr to_keyframe = uuid_keyframe_map[edge->to_uuid];
        RCLCPP_INFO_STREAM( logger, "Adding from keyframe: " << from_keyframe->readable_id << " to edge " << edge->readable_id );
        edge->from_keyframe = from_keyframe;
        RCLCPP_INFO_STREAM( logger, "Adding to keyframe: " << to_keyframe->readable_id << " to edge " << edge->readable_id );
        edge->to_keyframe = to_keyframe;

        if( from_keyframe->node == nullptr ) {
            RCLCPP_WARN_STREAM( logger, "from_keyframe->node is nullptr" );
        }
        if( to_keyframe->node == nullptr ) {
            RCLCPP_WARN_STREAM( logger, "to_keyframe->node is nullptr" );
        }
        auto edge_g2o = graph_slam->add_se3_edge( from_keyframe->node, to_keyframe->node, edge->relative_pose_loaded,
                                                  edge->information_loaded );

        edge->edge = edge_g2o;
        edges.emplace_back( edge );
        edge_uuids.insert( edge->uuid );

        // Add the edge to the corresponding keyframes
        if( edge->type == Edge::TYPE_ODOM ) {
            if( from_keyframe->odom_keyframe_counter > 1 ) {
                from_keyframe->prev_edge = edge;
                RCLCPP_INFO_STREAM( logger,
                                    "Setting edge " << edge->readable_id << " as prev edge to keyframe " << from_keyframe->readable_id );
            }
            to_keyframe->next_edge = edge;
            RCLCPP_INFO_STREAM( logger, "Setting edge " << edge->readable_id << " as next edge to keyframe " << to_keyframe->readable_id );
        }

        if( edge->type == Edge::TYPE_ODOM || edge->type == Edge::TYPE_ANCHOR ) {
            graph_slam->add_robust_kernel( edge_g2o, odometry_edge_robust_kernel, odometry_edge_robust_kernel_size );
        } else if( edge->type == Edge::TYPE_LOOP ) {
            Eigen::Matrix4f relative_pose_matrix = edge->relative_pose().matrix().cast<float>();
            Loop::Ptr       loop                 = std::make_shared<Loop>( from_keyframe, to_keyframe, relative_pose_matrix );
            loop_manager->add_loop_accum_distance_check( loop );
            graph_slam->add_robust_kernel( edge_g2o, loop_closure_edge_robust_kernel, loop_closure_edge_robust_kernel_size );
        }
    }

    loaded_keyframes.clear();
    loaded_edges.clear();
    return true;
}


void
GraphDatabase::insert_loops( const std::vector<Loop::Ptr> &loops )
{
    std::lock_guard<std::mutex> lock( keyframe_queue_mutex );

    for( const auto &loop : loops ) {
        Eigen::Isometry3d relpose( loop->relative_pose.cast<double>() );
        Eigen::MatrixXd   information_matrix = inf_calclator->calc_information_matrix( loop->key1->cloud, loop->key2->cloud, relpose );
        auto              g2o_edge           = graph_slam->add_se3_edge( loop->key1->node, loop->key2->node, relpose, information_matrix );

        auto        uuid     = uuid_generator();
        std::string uuid_str = boost::uuids::to_string( uuid );

        Edge::Ptr edge = std::make_shared<Edge>( g2o_edge, Edge::TYPE_LOOP, uuid, uuid_str, loop->key1, loop->key2 );
        edges.emplace_back( edge );
        edge_uuids.insert( edge->uuid );
        graph_slam->add_robust_kernel( g2o_edge, loop_closure_edge_robust_kernel, loop_closure_edge_robust_kernel_size );
    }

    // Add the new keyframes that have been checked for loop closures to the keyframes vector
    std::copy( new_keyframes.begin(), new_keyframes.end(), std::back_inserter( keyframes ) );
    new_keyframes.clear();
}


void
GraphDatabase::save_keyframe_poses()
{
    static int save_counter = 0;

    if( result_dir.empty() ) {
        return;
    }

    std::string name = own_name;
    if( own_name.empty() ) {
        name = "no_namespace";
    }

    std::ostringstream dir_format;
    dir_format << result_dir << "/" << name;
    std::string dir_path = dir_format.str();

    if( !boost::filesystem::exists( dir_path ) ) {
        boost::filesystem::create_directories( dir_path );
    }

    std::ostringstream file_format;
    file_format << dir_path << "/" << name << "_" << std::setw( 4 ) << std::setfill( '0' ) << save_counter << ".txt";

    RCLCPP_INFO_STREAM( rclcpp::get_logger( "save_keyframe_poses" ), "Saving keyframe poses to " << file_format.str() );
    std::lock_guard<std::mutex> lock( keyframe_queue_mutex );
    std::ofstream               ofs( file_format.str() );
    for( const auto &keyframe : keyframes ) {
        if( keyframe->node == nullptr ) {
            continue;
        }
        if( keyframe->robot_name != own_name ) {
            continue;
        }
        Eigen::Isometry3d  pose = keyframe->node->estimate();
        Eigen::Quaterniond q( pose.linear() );
        Eigen::Vector3d    t( pose.translation() );
        ofs << keyframe->stamp.sec << "." << std::setfill( '0' ) << std::setw( 9 ) << keyframe->stamp.nanosec << " " << t.x() << " "
            << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    save_counter++;
}

}  // namespace mrg_slam