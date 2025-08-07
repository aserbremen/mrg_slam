// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <boost/format.hpp>
#include <g2o/edge_plane_identity.hpp>
#include <g2o/edge_plane_parallel.hpp>
#include <g2o/edge_plane_prior.hpp>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/robust_kernel_io.hpp>
#include <mrg_slam/graph_slam.hpp>

G2O_USE_OPTIMIZATION_LIBRARY( pcg )
G2O_USE_OPTIMIZATION_LIBRARY( cholmod )  // be aware of that cholmod brings GPL dependency
G2O_USE_OPTIMIZATION_LIBRARY( csparse )  // be aware of that csparse brings LGPL unless it is dynamically linked

namespace g2o {
G2O_REGISTER_TYPE( EDGE_SE3_PLANE, EdgeSE3Plane )
G2O_REGISTER_TYPE( EDGE_SE3_PRIORXY, EdgeSE3PriorXY )
G2O_REGISTER_TYPE( EDGE_SE3_PRIORXYZ, EdgeSE3PriorXYZ )
G2O_REGISTER_TYPE( EDGE_SE3_PRIORVEC, EdgeSE3PriorVec )
G2O_REGISTER_TYPE( EDGE_SE3_PRIORQUAT, EdgeSE3PriorQuat )
G2O_REGISTER_TYPE( EDGE_PLANE_PRIOR_NORMAL, EdgePlanePriorNormal )
G2O_REGISTER_TYPE( EDGE_PLANE_PRIOR_DISTANCE, EdgePlanePriorDistance )
G2O_REGISTER_TYPE( EDGE_PLANE_IDENTITY, EdgePlaneIdentity )
G2O_REGISTER_TYPE( EDGE_PLANE_PARALLEL, EdgePlaneParallel )
G2O_REGISTER_TYPE( EDGE_PLANE_PAERPENDICULAR, EdgePlanePerpendicular )
}  // namespace g2o

namespace mrg_slam {

/**
 * @brief constructor
 */
GraphSLAM::GraphSLAM( const std::string& solver_type )
{
    graph_.reset( new g2o::SparseOptimizer() );
    g2o::SparseOptimizer* optimizer = dynamic_cast<g2o::SparseOptimizer*>( this->graph_.get() );

    std::cout << "construct solver: " << solver_type << std::endl;
    g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_property;
    g2o::OptimizationAlgorithm*        solver = solver_factory->construct( solver_type, solver_property );
    optimizer->setAlgorithm( solver );

    if( !optimizer->solver() ) {
        std::cerr << std::endl;
        std::cerr << "error : failed to allocate solver!!" << std::endl;
        solver_factory->listSolvers( std::cerr );
        std::cerr << "-------------" << std::endl;
        std::cin.ignore( 1 );
        return;
    }
    std::cout << "done" << std::endl;

    robust_kernel_factory_ = g2o::RobustKernelFactory::instance();
}

/**
 * @brief destructor
 */
GraphSLAM::~GraphSLAM() { graph_.reset(); }

void
GraphSLAM::set_solver( const std::string& solver_type )
{
    g2o::SparseOptimizer* optimizer = dynamic_cast<g2o::SparseOptimizer*>( this->graph_.get() );

    std::cout << "construct solver: " << solver_type << std::endl;
    g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_property;
    g2o::OptimizationAlgorithm*        solver = solver_factory->construct( solver_type, solver_property );
    optimizer->setAlgorithm( solver );

    if( !optimizer->solver() ) {
        std::cerr << std::endl;
        std::cerr << "error : failed to allocate solver!!" << std::endl;
        solver_factory->listSolvers( std::cerr );
        std::cerr << "-------------" << std::endl;
        std::cin.ignore( 1 );
        return;
    }
    std::cout << "done" << std::endl;
}

int
GraphSLAM::num_vertices() const
{
    return graph_->vertices().size();
}
int
GraphSLAM::num_edges() const
{
    return graph_->edges().size();
}

g2o::VertexSE3*
GraphSLAM::add_se3_node( const Eigen::Isometry3d& pose )
{
    g2o::VertexSE3* vertex( new g2o::VertexSE3() );
    vertex->setId( static_cast<int>( graph_->vertices().size() ) );
    vertex->setEstimate( pose );
    graph_->addVertex( vertex );

    return vertex;
}

g2o::VertexPlane*
GraphSLAM::add_plane_node( const Eigen::Vector4d& plane_coeffs )
{
    g2o::VertexPlane* vertex( new g2o::VertexPlane() );
    vertex->setId( static_cast<int>( graph_->vertices().size() ) );
    vertex->setEstimate( plane_coeffs );
    graph_->addVertex( vertex );

    return vertex;
}

g2o::VertexPointXYZ*
GraphSLAM::add_point_xyz_node( const Eigen::Vector3d& xyz )
{
    g2o::VertexPointXYZ* vertex( new g2o::VertexPointXYZ() );
    vertex->setId( static_cast<int>( graph_->vertices().size() ) );
    vertex->setEstimate( xyz );
    graph_->addVertex( vertex );

    return vertex;
}

g2o::EdgeSE3*
GraphSLAM::add_se3_edge( g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose,
                         const Eigen::MatrixXd& information_matrix )
{
    g2o::EdgeSE3* edge( new g2o::EdgeSE3() );
    edge->setId( static_cast<int>( graph_->edges().size() ) );
    edge->setMeasurement( relative_pose );
    edge->setInformation( information_matrix );
    edge->vertices()[0] = v1;
    edge->vertices()[1] = v2;
    graph_->addEdge( edge );

    return edge;
}

g2o::EdgeSE3Plane*
GraphSLAM::add_se3_plane_edge( g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs,
                               const Eigen::MatrixXd& information_matrix )
{
    g2o::EdgeSE3Plane* edge( new g2o::EdgeSE3Plane() );
    edge->setId( static_cast<int>( graph_->edges().size() ) );
    edge->setMeasurement( plane_coeffs );
    edge->setInformation( information_matrix );
    edge->vertices()[0] = v_se3;
    edge->vertices()[1] = v_plane;
    graph_->addEdge( edge );

    return edge;
}

g2o::EdgeSE3PointXYZ*
GraphSLAM::add_se3_point_xyz_edge( g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz,
                                   const Eigen::MatrixXd& information_matrix )
{
    g2o::EdgeSE3PointXYZ* edge( new g2o::EdgeSE3PointXYZ() );
    edge->setId( static_cast<int>( graph_->edges().size() ) );
    edge->setMeasurement( xyz );
    edge->setInformation( information_matrix );
    edge->vertices()[0] = v_se3;
    edge->vertices()[1] = v_xyz;
    graph_->addEdge( edge );

    return edge;
}

g2o::EdgePlanePriorNormal*
GraphSLAM::add_plane_normal_prior_edge( g2o::VertexPlane* v, const Eigen::Vector3d& normal, const Eigen::MatrixXd& information_matrix )
{
    g2o::EdgePlanePriorNormal* edge( new g2o::EdgePlanePriorNormal() );
    edge->setId( static_cast<int>( graph_->edges().size() ) );
    edge->setMeasurement( normal );
    edge->setInformation( information_matrix );
    edge->vertices()[0] = v;
    graph_->addEdge( edge );

    return edge;
}

g2o::EdgePlanePriorDistance*
GraphSLAM::add_plane_distance_prior_edge( g2o::VertexPlane* v, double distance, const Eigen::MatrixXd& information_matrix )
{
    g2o::EdgePlanePriorDistance* edge( new g2o::EdgePlanePriorDistance() );
    edge->setId( static_cast<int>( graph_->edges().size() ) );
    edge->setMeasurement( distance );
    edge->setInformation( information_matrix );
    edge->vertices()[0] = v;
    graph_->addEdge( edge );

    return edge;
}

g2o::EdgeSE3PriorXY*
GraphSLAM::add_se3_prior_xy_edge( g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy, const Eigen::MatrixXd& information_matrix )
{
    g2o::EdgeSE3PriorXY* edge( new g2o::EdgeSE3PriorXY() );
    edge->setId( static_cast<int>( graph_->edges().size() ) );
    edge->setMeasurement( xy );
    edge->setInformation( information_matrix );
    edge->vertices()[0] = v_se3;
    graph_->addEdge( edge );

    return edge;
}

g2o::EdgeSE3PriorXYZ*
GraphSLAM::add_se3_prior_xyz_edge( g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix )
{
    g2o::EdgeSE3PriorXYZ* edge( new g2o::EdgeSE3PriorXYZ() );
    edge->setId( static_cast<int>( graph_->edges().size() ) );
    edge->setMeasurement( xyz );
    edge->setInformation( information_matrix );
    edge->vertices()[0] = v_se3;
    graph_->addEdge( edge );

    return edge;
}

g2o::EdgeSE3PriorVec*
GraphSLAM::add_se3_prior_vec_edge( g2o::VertexSE3* v_se3, const Eigen::Vector3d& direction, const Eigen::Vector3d& measurement,
                                   const Eigen::MatrixXd& information_matrix )
{
    Eigen::Matrix<double, 6, 1> m;
    m.head<3>() = direction;
    m.tail<3>() = measurement;

    g2o::EdgeSE3PriorVec* edge( new g2o::EdgeSE3PriorVec() );
    edge->setId( static_cast<int>( graph_->edges().size() ) );
    edge->setMeasurement( m );
    edge->setInformation( information_matrix );
    edge->vertices()[0] = v_se3;
    graph_->addEdge( edge );

    return edge;
}

g2o::EdgeSE3PriorQuat*
GraphSLAM::add_se3_prior_quat_edge( g2o::VertexSE3* v_se3, const Eigen::Quaterniond& quat, const Eigen::MatrixXd& information_matrix )
{
    g2o::EdgeSE3PriorQuat* edge( new g2o::EdgeSE3PriorQuat() );
    edge->setId( static_cast<int>( graph_->edges().size() ) );
    edge->setMeasurement( quat );
    edge->setInformation( information_matrix );
    edge->vertices()[0] = v_se3;
    graph_->addEdge( edge );

    return edge;
}

g2o::EdgePlane*
GraphSLAM::add_plane_edge( g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement,
                           const Eigen::Matrix4d& information )
{
    g2o::EdgePlane* edge( new g2o::EdgePlane() );
    edge->setId( static_cast<int>( graph_->edges().size() ) );
    edge->setMeasurement( measurement );
    edge->setInformation( information );
    edge->vertices()[0] = v_plane1;
    edge->vertices()[1] = v_plane2;
    graph_->addEdge( edge );

    return edge;
}

g2o::EdgePlaneIdentity*
GraphSLAM::add_plane_identity_edge( g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement,
                                    const Eigen::Matrix4d& information )
{
    g2o::EdgePlaneIdentity* edge( new g2o::EdgePlaneIdentity() );
    edge->setId( static_cast<int>( graph_->edges().size() ) );
    edge->setMeasurement( measurement );
    edge->setInformation( information );
    edge->vertices()[0] = v_plane1;
    edge->vertices()[1] = v_plane2;
    graph_->addEdge( edge );

    return edge;
}

g2o::EdgePlaneParallel*
GraphSLAM::add_plane_parallel_edge( g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement,
                                    const Eigen::Matrix3d& information )
{
    g2o::EdgePlaneParallel* edge( new g2o::EdgePlaneParallel() );
    edge->setId( static_cast<int>( graph_->edges().size() ) );
    edge->setMeasurement( measurement );
    edge->setInformation( information );
    edge->vertices()[0] = v_plane1;
    edge->vertices()[1] = v_plane2;
    graph_->addEdge( edge );

    return edge;
}

g2o::EdgePlanePerpendicular*
GraphSLAM::add_plane_perpendicular_edge( g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement,
                                         const Eigen::MatrixXd& information )
{
    g2o::EdgePlanePerpendicular* edge( new g2o::EdgePlanePerpendicular() );
    edge->setId( static_cast<int>( graph_->edges().size() ) );
    edge->setMeasurement( measurement );
    edge->setInformation( information );
    edge->vertices()[0] = v_plane1;
    edge->vertices()[1] = v_plane2;
    graph_->addEdge( edge );

    return edge;
}

void
GraphSLAM::add_robust_kernel( g2o::HyperGraph::Edge* edge, const std::string& kernel_type, double kernel_size )
{
    if( kernel_type == "NONE" ) {
        return;
    }

    g2o::RobustKernel* kernel = robust_kernel_factory_->construct( kernel_type );
    if( kernel == nullptr ) {
        std::cerr << "warning : invalid robust kernel type: " << kernel_type << std::endl;
        return;
    }

    kernel->setDelta( kernel_size );

    g2o::OptimizableGraph::Edge* edge_ = dynamic_cast<g2o::OptimizableGraph::Edge*>( edge );
    edge_->setRobustKernel( kernel );
}

int
GraphSLAM::optimize( int num_iterations, bool verbose )
{
    static int optimize_count = 0;

    g2o::SparseOptimizer* optimizer = dynamic_cast<g2o::SparseOptimizer*>( this->graph_.get() );

    std::cout << std::endl;
    std::cout << "--- pose graph optimization ---" << std::endl;
    std::cout << "nodes: " << optimizer->vertices().size() << "   edges: " << optimizer->edges().size() << std::endl;
    std::cout << "optimizing... " << std::flush;

    std::cout << "init" << std::endl;
    optimizer->initializeOptimization();
    optimizer->setVerbose( verbose );

    double chi2 = optimizer->chi2();

    if( save_graph_at_each_optimization_ ) {
        std::stringstream ss;
        ss << std::setw( 3 ) << std::setfill( '0' ) << optimize_count;
        std::string filename = "graph_before_optimization_" + ss.str() + ".g2o";
        optimizer->save( filename.c_str() );
    }

    std::cout << "optimize!! graph slam iteration #" << optimize_count << std::endl;
    // ROS2 migration use std::chrono::system_clock insteead of ros::Walltime::now(), see https://github.com/ros-planning/moveit2/issues/31
    auto t1         = std::chrono::system_clock::now();
    int  iterations = optimizer->optimize( num_iterations );

    if( save_graph_at_each_optimization_ ) {
        std::stringstream ss;
        ss << std::setw( 3 ) << std::setfill( '0' ) << optimize_count;
        std::string filename = "graph_after_optimization_" + ss.str() + ".g2o";
        optimizer->save( filename.c_str() );
    }

    optimize_count++;
    auto t2 = std::chrono::system_clock::now();
    std::cout << "done" << std::endl;
    std::cout << "iterations: " << iterations << " / " << num_iterations << std::endl;
    std::cout << "chi2: (before)" << chi2 << " -> (after)" << optimizer->chi2() << std::endl;
    std::chrono::duration<double> diff = t2 - t1;
    std::cout << "time: " << boost::format( "%.3f" ) % diff.count() << "[sec]" << std::endl;

    return iterations;
}

std::shared_ptr<g2o::SparseBlockMatrixX>
GraphSLAM::compute_marginals( const g2o::OptimizableGraph::VertexContainer& vertices )
{
    g2o::SparseOptimizer* optimizer = dynamic_cast<g2o::SparseOptimizer*>( this->graph_.get() );

    auto res = std::make_shared<g2o::SparseBlockMatrixX>();
    optimizer->computeMarginals( *res, vertices );
    return res;
}

std::shared_ptr<g2o::SparseBlockMatrixX>
GraphSLAM::compute_marginals()
{
    g2o::OptimizableGraph::VertexContainer vertices( graph_->vertices().size() );
    size_t                                 i = 0;
    for( auto& v : graph_->vertices() ) {
        auto vert = static_cast<g2o::OptimizableGraph::Vertex*>( v.second );
        if( vert->hessianIndex() >= 0 ) {
            vertices[i] = static_cast<g2o::OptimizableGraph::Vertex*>( vert );
            i++;
        }
    }
    vertices.resize( i );

    return compute_marginals( vertices );
}

void
GraphSLAM::save( const std::string& filename )
{
    g2o::SparseOptimizer* optimizer = dynamic_cast<g2o::SparseOptimizer*>( this->graph_.get() );

    std::ofstream ofs( filename );
    optimizer->save( ofs );

    g2o::save_robust_kernels( filename + ".kernels", optimizer );
}

bool
GraphSLAM::load( const std::string& filename )
{
    std::cout << "loading pose graph..." << std::endl;
    g2o::SparseOptimizer* optimizer = dynamic_cast<g2o::SparseOptimizer*>( this->graph_.get() );

    std::ifstream ifs( filename );
    if( !optimizer->load( ifs ) ) {
        return false;
    }

    std::cout << "nodes  : " << optimizer->vertices().size() << std::endl;
    std::cout << "edges  : " << optimizer->edges().size() << std::endl;

    if( !g2o::load_robust_kernels( filename + ".kernels", optimizer ) ) {
        return false;
    }

    return true;
}

void
GraphSLAM::set_save_graph( bool save_graph )
{
    save_graph_at_each_optimization_ = save_graph;
}

}  // namespace mrg_slam
