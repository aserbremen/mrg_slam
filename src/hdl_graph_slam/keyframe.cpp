// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <pcl/io/pcd_io.h>

#include <boost/filesystem.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <hdl_graph_slam/keyframe.hpp>
// ROS2 migration
#include <rclcpp/logging.hpp>

namespace hdl_graph_slam {

KeyFrame::KeyFrame( const std::string& robot_name, const builtin_interfaces::msg::Time& stamp, const Eigen::Isometry3d& odom,
                    int odom_keyframe_counter, const boost::uuids::uuid& uuid, double accum_distance,
                    const pcl::PointCloud<PointT>::ConstPtr& cloud, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg ) :
    robot_name( robot_name ),
    stamp( stamp ),
    odom( odom ),
    odom_keyframe_counter( odom_keyframe_counter ),
    uuid( uuid ),
    uuid_str( boost::uuids::to_string( uuid ) ),
    accum_distance( accum_distance ),
    first_keyframe( false ),
    cloud( cloud ),
    cloud_msg( cloud_msg ),
    node( nullptr )
{
    readable_id = robot_name + "-" + std::to_string( odom_keyframe_counter );
}

KeyFrame::KeyFrame( const std::string& directory, g2o::HyperGraph* graph ) :
    stamp(),
    odom( Eigen::Isometry3d::Identity() ),
    accum_distance( -1 ),
    first_keyframe( false ),
    cloud( nullptr ),
    cloud_msg( nullptr ),
    node( nullptr )
{
    // TODO implement with new way of identifying keyframes
    load( directory, graph );
}

KeyFrame::~KeyFrame() {}


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


void
KeyFrame::save( const std::string& directory )
{
    if( !boost::filesystem::is_directory( directory ) ) {
        boost::filesystem::create_directory( directory );
    }

    std::ofstream ofs( directory + "/data" );
    ofs << "robot_name " << robot_name << "\n";
    ofs << "stamp " << stamp.sec << " " << stamp.nanosec << "\n";

    ofs << "estimate\n";
    ofs << node->estimate().matrix() << "\n";

    ofs << "odom\n";
    ofs << odom.matrix() << "\n";

    ofs << "accum_distance " << accum_distance << "\n";

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
        ofs << "orientation " << orientation->w() << " " << orientation->x() << " " << orientation->y() << " " << orientation->z() << "\n";
    }

    if( node ) {
        ofs << "id " << node->id() << "\n";
    }

    pcl::io::savePCDFileBinary( directory + "/cloud.pcd", *cloud );
}

bool
KeyFrame::load( const std::string& directory, g2o::HyperGraph* graph )
{
    std::ifstream ifs( directory + "/data" );
    if( !ifs ) {
        return false;
    }

    long                               node_id = -1;
    boost::optional<Eigen::Isometry3d> estimate;

    while( !ifs.eof() ) {
        std::string token;
        ifs >> token;

        if( token == "stamp" ) {
            ifs >> stamp.sec >> stamp.nanosec;
        } else if( token == "estimate" ) {
            Eigen::Matrix4d mat;
            for( int i = 0; i < 4; i++ ) {
                for( int j = 0; j < 4; j++ ) {
                    ifs >> mat( i, j );
                }
            }
            estimate                = Eigen::Isometry3d::Identity();
            estimate->linear()      = mat.block<3, 3>( 0, 0 );
            estimate->translation() = mat.block<3, 1>( 0, 3 );
        } else if( token == "odom" ) {
            Eigen::Matrix4d odom_mat = Eigen::Matrix4d::Identity();
            for( int i = 0; i < 4; i++ ) {
                for( int j = 0; j < 4; j++ ) {
                    ifs >> odom_mat( i, j );
                }
            }

            odom.setIdentity();
            odom.linear()      = odom_mat.block<3, 3>( 0, 0 );
            odom.translation() = odom_mat.block<3, 1>( 0, 3 );
        } else if( token == "accum_distance" ) {
            ifs >> accum_distance;
        } else if( token == "floor_coeffs" ) {
            Eigen::Vector4d coeffs;
            ifs >> coeffs[0] >> coeffs[1] >> coeffs[2] >> coeffs[3];
            floor_coeffs = coeffs;
        } else if( token == "utm_coord" ) {
            Eigen::Vector3d coord;
            ifs >> coord[0] >> coord[1] >> coord[2];
            utm_coord = coord;
        } else if( token == "acceleration" ) {
            Eigen::Vector3d acc;
            ifs >> acc[0] >> acc[1] >> acc[2];
            acceleration = acc;
        } else if( token == "orientation" ) {
            Eigen::Quaterniond quat;
            ifs >> quat.w() >> quat.x() >> quat.y() >> quat.z();
            orientation = quat;
        } else if( token == "id" ) {
            ifs >> node_id;
        }
    }

    if( node_id < 0 ) {
        RCLCPP_ERROR_STREAM( rclcpp::get_logger( "rclcpp" ), "invalid node id!!" );
        RCLCPP_ERROR_STREAM( rclcpp::get_logger( "rclcpp" ), directory );
        return false;
    }

    if( graph->vertices().find( node_id ) == graph->vertices().end() ) {
        RCLCPP_ERROR_STREAM( rclcpp::get_logger( "rclcpp" ), "vertex ID=" << node_id << " does not exist!!" );
        return false;
    }

    node = dynamic_cast<g2o::VertexSE3*>( graph->vertices()[node_id] );
    if( node == nullptr ) {
        RCLCPP_ERROR_STREAM( rclcpp::get_logger( "rclcpp" ), "failed to downcast!!" );
        return false;
    }

    if( estimate ) {
        node->setEstimate( *estimate );
    }

    pcl::PointCloud<PointT>::Ptr cloud_( new pcl::PointCloud<PointT>() );
    pcl::io::loadPCDFile( directory + "/cloud.pcd", *cloud_ );
    cloud = cloud_;

    return true;
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

/*
KeyFrameSnapshot::KeyFrameSnapshot( long id, const Eigen::Isometry3d& pose, const pcl::PointCloud<PointT>::ConstPtr& cloud,
                                    bool first_keyframe, const Eigen::MatrixXd* _covariance ) :
    id( id ), pose( pose ), first_keyframe( first_keyframe ), cloud( cloud )
{
    if( _covariance ) {
        covariance = *_covariance;
    }
}
*/

KeyFrameSnapshot::KeyFrameSnapshot( const KeyFrame::Ptr& key, const std::shared_ptr<g2o::SparseBlockMatrixX>& marginals ) :
    stamp( key->stamp ),
    id( key->id() ),
    uuid( key->uuid ),
    pose( key->node->estimate() ),
    cloud( key->cloud ),
    first_keyframe( key->first_keyframe )
{
    if( marginals ) {
        covariance = key->covariance( marginals );
    } else {
        covariance = Eigen::Matrix<double, 6, 6>::Zero();
    }
}

KeyFrameSnapshot::~KeyFrameSnapshot() {}

}  // namespace hdl_graph_slam
