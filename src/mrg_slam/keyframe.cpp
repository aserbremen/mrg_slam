// SPDX-License-Identifier: BSD-2-Clause

// boost
#include <boost/filesystem.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
// g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/vertex_se3.h>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
// mrg_slam
#include <mrg_slam/keyframe.hpp>
// ROS2
#include <rclcpp/logging.hpp>

namespace mrg_slam {

KeyFrame::KeyFrame( const std::string& robot_name, const builtin_interfaces::msg::Time& stamp, const Eigen::Isometry3d& odom,
                    int odom_keyframe_counter, double accum_distance, const boost::uuids::uuid& uuid, const std::string& uuid_str,
                    const pcl::PointCloud<PointT>::ConstPtr& cloud, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg ) :
    robot_name( robot_name ),
    stamp( stamp ),
    odom( odom ),
    odom_keyframe_counter( odom_keyframe_counter ),
    uuid( uuid ),
    uuid_str( uuid_str ),
    accum_distance( accum_distance ),
    first_keyframe( false ),
    static_keyframe( false ),
    cloud( cloud ),
    cloud_msg( cloud_msg ),
    node( nullptr )
{
    readable_id = robot_name.empty() ? empty_robot_name_str : robot_name;
    // use the first 8 characters of the uuid if keyframe is given by the static keyframe provider
    readable_id += odom_keyframe_counter >= 0 ? "." + std::to_string( odom_keyframe_counter ) : "." + uuid_str.substr( 0, 4 );
}

KeyFrame::KeyFrame( const std::string& keyframe_path, const std::string& pcd_path, const boost::uuids::uuid& _uuid,
                    const std::string& _uuid_str )
{
    load( keyframe_path, pcd_path, _uuid, _uuid_str );
}

KeyFrame::~KeyFrame() {}

void
KeyFrame::save( const std::string& result_path )
{
    std::ofstream ofs( result_path + ".txt" );

    if( robot_name.empty() ) {
        ofs << "robot_name " << empty_robot_name_str << "\n";
    } else {
        ofs << "robot_name " << robot_name << "\n";
    }
    std::string readable_id_write = readable_id;
    size_t      pos               = readable_id_write.find( loaded_keyframe_str );
    if( pos != std::string::npos ) {
        readable_id_write.erase( pos, loaded_keyframe_str.size() );
    }
    ofs << "readable_id " << readable_id_write << "\n";
    ofs << "stamp " << stamp.sec << " " << stamp.nanosec << "\n";

    ofs << "estimate\n";
    ofs << node->estimate().matrix() << "\n";

    ofs << "odom_counter " << odom_keyframe_counter << "\n";

    ofs << "odom\n";
    ofs << odom.matrix() << "\n";

    ofs << "accum_distance " << accum_distance << "\n";

    ofs << "first_keyframe " << first_keyframe << "\n";

    ofs << "static_keyframe " << static_keyframe << "\n";

    if( floor_coeffs ) {
        ofs << "floor_coeffs " << floor_coeffs->transpose() << "\n";
    }

    if( utm_coord ) {
        ofs << "utm_coord " << utm_coord->transpose() << "\n";
    }

    if( acceleration ) {
        ofs << "acceleration " << acceleration->transpose() << "\n";
    }

    if( orientation ) {
        ofs << "orientation " << orientation->x() << " " << orientation->y() << " " << orientation->z() << " " << orientation->w() << "\n";
    }

    if( node ) {
        ofs << "g2o_id " << node->id() << "\n";
    }

    ofs << "uuid_str " << uuid_str << "\n";

    // Save the point cloud
    pcl::io::savePCDFileBinary( result_path + ".pcd", *cloud );
}

bool
KeyFrame::load( const std::string& keyframe_path, const std::string& pcd_path, const boost::uuids::uuid& _uuid,
                const std::string& _uuid_str )
{
    auto logger = rclcpp::get_logger( "KeyFrame::load" );

    uuid     = _uuid;
    uuid_str = _uuid_str;

    std::ifstream ifs( keyframe_path );
    if( !ifs ) {
        RCLCPP_ERROR_STREAM( logger, "Failed to open " << keyframe_path );
        return false;
    }

    RCLCPP_INFO_STREAM( logger, "Loading keyframe from " << keyframe_path );

    // when loading from file, accum_distance is negative so that loaded keyframes are considered when determining loop closure candidates
    accum_distance = -1.0;

    std::string line;
    while( std::getline( ifs, line ) ) {
        std::istringstream iss( line );
        std::string        key;
        iss >> key;
        if( key == "robot_name" ) {
            iss >> robot_name;
            if( robot_name == empty_robot_name_str ) {
                robot_name = std::string();  // empty string
            }
        } else if( key == "readable_id" ) {
            iss >> readable_id;
            readable_id += loaded_keyframe_str;
        } else if( key == "stamp" ) {
            iss >> stamp.sec >> stamp.nanosec;
        } else if( key == "estimate" ) {
            auto& estimate_mat = estimate_transform.matrix();
            for( int i = 0; i < 4; ++i ) {
                std::getline( ifs, line );
                std::istringstream matrixStream( line );
                for( int j = 0; j < 4; ++j ) {
                    matrixStream >> estimate_mat( i, j );
                }
            }
            estimate_transform.matrix() = estimate_mat;
        } else if( key == "odom_counter" ) {
            iss >> odom_keyframe_counter;
        } else if( key == "odom" ) {
            auto& odom_mat = odom.matrix();
            for( int i = 0; i < 4; ++i ) {
                std::getline( ifs, line );
                std::istringstream matrixStream( line );
                for( int j = 0; j < 4; ++j ) {
                    matrixStream >> odom_mat( i, j );
                }
            }
        } else if( key == "first_keyframe" ) {
            iss >> first_keyframe;
        } else if( key == "static_keyframe" ) {
            iss >> static_keyframe;
        } else if( key == "floor_coeffs" ) {
            Eigen::Vector4d coeffs;
            ifs >> coeffs[0] >> coeffs[1] >> coeffs[2] >> coeffs[3];
            floor_coeffs = coeffs;
        } else if( key == "utm_coord" ) {
            Eigen::Vector3d coord;
            ifs >> coord[0] >> coord[1] >> coord[2];
            utm_coord = coord;
        } else if( key == "acceleration" ) {
            Eigen::Vector3d acc;
            ifs >> acc[0] >> acc[1] >> acc[2];
            acceleration = acc;
        } else if( key == "orientation" ) {
            Eigen::Quaterniond quat;
            ifs >> quat.x() >> quat.y() >> quat.z() >> quat.w();
            orientation = quat;
        }
    }

    pcl::PointCloud<PointT>::Ptr cloud_( new pcl::PointCloud<PointT>() );
    pcl::io::loadPCDFile( pcd_path, *cloud_ );
    cloud = std::move( cloud_ );

    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_( new sensor_msgs::msg::PointCloud2() );
    pcl::toROSMsg( *cloud, *cloud_msg_ );
    cloud_msg = std::move( cloud_msg_ );

    return true;
}

Eigen::Matrix<double, 6, 6>
KeyFrame::covariance( const std::shared_ptr<g2o::SparseBlockMatrixX>& marginals ) const
{
    auto hidx = node->hessianIndex();
    if( hidx >= 0 ) {
        return *marginals->block( hidx, hidx );
    } else {
        return Eigen::Matrix<double, 6, 6>::Zero();
    }
}

long
KeyFrame::id() const
{
    return node->id();
}

Eigen::Isometry3d
KeyFrame::estimate() const
{
    return node->estimate();
}

bool
KeyFrame::edge_exists( const KeyFrame& other, const rclcpp::Logger& logger ) const
{
    const auto& from_node = node;
    const auto& to_node   = other.node;
    bool        exist     = false;

    if( from_node == nullptr ) {
        RCLCPP_WARN_STREAM( logger, "from_node is nullptr!!" );
    }
    if( to_node == nullptr ) {
        RCLCPP_WARN_STREAM( logger, "to_node is nullptr!!" );
    }

    // check if both nodes are connected by an edge already
    for( const auto& node_edge : from_node->edges() ) {
        if( node_edge->vertices().size() == 2 ) {
            if( ( node_edge->vertex( 0 ) == from_node && node_edge->vertex( 1 ) == to_node )
                || ( node_edge->vertex( 0 ) == to_node && node_edge->vertex( 1 ) == from_node ) ) {
                exist = true;
                break;
            }
        }
    }

    return exist;
}

KeyFrameSnapshot::KeyFrameSnapshot( const KeyFrame::Ptr& key, const std::shared_ptr<g2o::SparseBlockMatrixX>& marginals ) :
    stamp( key->stamp ),
    id( key->id() ),
    uuid( key->uuid ),
    pose( key->node->estimate() ),
    cloud( key->cloud ),
    first_keyframe( key->first_keyframe ),
    static_keyframe( key->static_keyframe )
{
    if( marginals ) {
        covariance = key->covariance( marginals );
    } else {
        covariance = Eigen::Matrix<double, 6, 6>::Zero();
    }
}

KeyFrameSnapshot::~KeyFrameSnapshot() {}

}  // namespace mrg_slam
