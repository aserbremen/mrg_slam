// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/edge_se3.h>

#include <boost/filesystem.hpp>
#include <hdl_graph_slam/edge.hpp>


namespace hdl_graph_slam {

Edge::Edge( const g2o::EdgeSE3* edge, Type type ) : edge( edge ), type( type ), gid( 0 ), from_gid( 0 ), to_gid( 0 ) {}

Edge::Edge( const g2o::EdgeSE3* edge, Type type, GlobalId from_gid, GlobalId to_gid, const GlobalIdGenerator& gid_generator ) :
    edge( edge ), type( type ), from_gid( from_gid ), to_gid( to_gid )
{
    gid = gid_generator( id() );
}

/*
Edge::Edge(const std::string& directory, g2o::HyperGraph* graph) {
  // TODO: implement
  // load(directory, graph);
}
*/


Edge::~Edge() {}


long
Edge::id() const
{
    return edge->id();
}


const Eigen::Isometry3d&
Edge::relative_pose() const
{
    return edge->measurement();
}


const Eigen::Matrix<double, 6, 6>&
Edge::information() const
{
    return edge->information();
}


EdgeSnapshot::EdgeSnapshot( const Edge::Ptr& edge ) :
    type( edge->type ), gid( edge->gid ), from_gid( edge->from_gid ), to_gid( edge->to_gid )
{
}

EdgeSnapshot::~EdgeSnapshot()
{
    // Nothing to do here
}

}  // namespace hdl_graph_slam
