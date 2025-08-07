// SPDX-License-Identifier: BSD-2-Clause

#ifndef GRAPH_SLAM_HPP
#define GRAPH_SLAM_HPP

#include <g2o/core/hyper_graph.h>
#include <g2o/core/sparse_block_matrix.h>

#include <memory>
// ROS2 migration replaces ros/time.h with std::chrono usage

namespace g2o {
class VertexSE3;
class VertexPlane;
class VertexPointXYZ;
class EdgeSE3;
class EdgeSE3Plane;
class EdgeSE3PointXYZ;
class EdgeSE3PriorXY;
class EdgeSE3PriorXYZ;
class EdgeSE3PriorVec;
class EdgeSE3PriorQuat;
class EdgePlane;
class EdgePlaneIdentity;
class EdgePlaneParallel;
class EdgePlanePerpendicular;
class EdgePlanePriorNormal;
class EdgePlanePriorDistance;
class RobustKernelFactory;
}  // namespace g2o

namespace mrg_slam {

class GraphSLAM {
public:
    GraphSLAM( const std::string& solver_type = "lm_var" );
    virtual ~GraphSLAM();

    int num_vertices() const;
    int num_edges() const;

    void set_solver( const std::string& solver_type );

    /**
     * @brief add a SE3 node to the graph
     * @param pose
     * @return registered node
     */
    g2o::VertexSE3* add_se3_node( const Eigen::Isometry3d& pose );

    /**
     * @brief add a plane node to the graph
     * @param plane_coeffs
     * @return registered node
     */
    g2o::VertexPlane* add_plane_node( const Eigen::Vector4d& plane_coeffs );

    /**
     * @brief add a point_xyz node to the graph
     * @param xyz
     * @return registered node
     */
    g2o::VertexPointXYZ* add_point_xyz_node( const Eigen::Vector3d& xyz );

    /**
     * @brief add an edge between SE3 nodes
     * @param v1  node1
     * @param v2  node2
     * @param relative_pose  relative pose between node1 and node2
     * @param information_matrix  information matrix (it must be 6x6)
     * @return registered edge
     */
    g2o::EdgeSE3* add_se3_edge( g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose,
                                const Eigen::MatrixXd& information_matrix );

    /**
     * @brief add an edge between an SE3 node and a plane node
     * @param v_se3    SE3 node
     * @param v_plane  plane node
     * @param plane_coeffs  plane coefficients w.r.t. v_se3
     * @param information_matrix  information matrix (it must be 3x3)
     * @return registered edge
     */
    g2o::EdgeSE3Plane* add_se3_plane_edge( g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs,
                                           const Eigen::MatrixXd& information_matrix );

    /**
     * @brief add an edge between an SE3 node and a point_xyz node
     * @param v_se3        SE3 node
     * @param v_xyz        point_xyz node
     * @param xyz          xyz coordinate
     * @param information  information_matrix (it must be 3x3)
     * @return registered edge
     */
    g2o::EdgeSE3PointXYZ* add_se3_point_xyz_edge( g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz,
                                                  const Eigen::MatrixXd& information_matrix );

    /**
     * @brief add a prior edge to an SE3 node
     * @param v_se3
     * @param xy
     * @param information_matrix
     * @return
     */
    g2o::EdgePlanePriorNormal* add_plane_normal_prior_edge( g2o::VertexPlane* v, const Eigen::Vector3d& normal,
                                                            const Eigen::MatrixXd& information_matrix );

    g2o::EdgePlanePriorDistance* add_plane_distance_prior_edge( g2o::VertexPlane* v, double distance,
                                                                const Eigen::MatrixXd& information_matrix );

    g2o::EdgeSE3PriorXY* add_se3_prior_xy_edge( g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy,
                                                const Eigen::MatrixXd& information_matrix );

    g2o::EdgeSE3PriorXYZ* add_se3_prior_xyz_edge( g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz,
                                                  const Eigen::MatrixXd& information_matrix );

    g2o::EdgeSE3PriorQuat* add_se3_prior_quat_edge( g2o::VertexSE3* v_se3, const Eigen::Quaterniond& quat,
                                                    const Eigen::MatrixXd& information_matrix );

    g2o::EdgeSE3PriorVec* add_se3_prior_vec_edge( g2o::VertexSE3* v_se3, const Eigen::Vector3d& direction,
                                                  const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information_matrix );

    g2o::EdgePlane* add_plane_edge( g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement,
                                    const Eigen::Matrix4d& information );

    g2o::EdgePlaneIdentity* add_plane_identity_edge( g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2,
                                                     const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information );

    g2o::EdgePlaneParallel* add_plane_parallel_edge( g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2,
                                                     const Eigen::Vector3d& measurement, const Eigen::Matrix3d& information );

    g2o::EdgePlanePerpendicular* add_plane_perpendicular_edge( g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2,
                                                               const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information );

    void add_robust_kernel( g2o::HyperGraph::Edge* edge, const std::string& kernel_type, double kernel_size );

    /**
     * @brief perform graph optimization
     */
    int optimize( int num_iterations, bool verbose = false );

    /**
     * @brief computes the covariance matrices for the given vertices, access result via
     * res.block( vertex->hessianIndex(), vertex->hessianIndex() )
     */
    std::shared_ptr<g2o::SparseBlockMatrixX> compute_marginals( const g2o::OptimizableGraph::VertexContainer& vertices );

    /**
     * @brief computes the covariance matrices for all vertices, access result via
     * res.block( vertex->hessianIndex(), vertex->hessianIndex() )
     */
    std::shared_ptr<g2o::SparseBlockMatrixX> compute_marginals();

    /**
     * @brief save the pose graph to a file
     * @param filename  output filename
     */
    void save( const std::string& filename );

    /**
     * @brief load the pose graph from file
     * @param filename  output filename
     */
    bool load( const std::string& filename );

    void set_save_graph( bool save_graph );

public:
    g2o::RobustKernelFactory*        robust_kernel_factory_;
    std::unique_ptr<g2o::HyperGraph> graph_;  // g2o graph

private:
    bool save_graph_at_each_optimization_ = false;
};

}  // namespace mrg_slam

#endif  // GRAPH_SLAM_HPP
