// SPDX-License-Identifier: BSD-2-Clause

// #include <gicp_omp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pclomp/gicp_omp.h>
#include <pclomp/ndt_omp.h>

// #include <pclomp/gicp_omp.h>
// #include <pclomp/ndt_omp.h>
// #include <pclomp/gicp_omp.h>
// #include <pclomp/ndt_omp.h>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <hdl_graph_slam/registrations.hpp>
#include <iostream>

#ifdef USE_VGICP_CUDA
#    include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif

namespace hdl_graph_slam {

pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr
// select_registration_method( ros::NodeHandle& pnh )
select_registration_method( rclcpp::Node::SharedPtr node )
{
    using PointT = pcl::PointXYZI;

    // select a registration method (ICP, GICP, NDT)
    // TODO: ROS2 parameter handling, verify this
    // std::string registration_method = pnh.param<std::string>( "registration_method", "NDT_OMP" );
    std::string registration_method = node->declare_parameter<std::string>( "registration_method", "NDT_OMP" );
    if( registration_method == "FAST_GICP" ) {
        std::cout << "registration: FAST_GICP" << std::endl;
        fast_gicp::FastGICP<PointT, PointT>::Ptr gicp( new fast_gicp::FastGICP<PointT, PointT>() );
        // gicp->setNumThreads( pnh.param<int>( "reg_num_threads", 0 ) );
        // gicp->setTransformationEpsilon( pnh.param<double>( "reg_transformation_epsilon", 0.01 ) );
        // gicp->setMaximumIterations( pnh.param<int>( "reg_maximum_iterations", 64 ) );
        // gicp->setMaxCorrespondenceDistance( pnh.param<double>( "reg_max_correspondence_distance", 2.5 ) );
        // gicp->setCorrespondenceRandomness( pnh.param<int>( "reg_correspondence_randomness", 20 ) );
        // TODO: ROS2 parameter handling, is declaring the parameter sufficient?, verify
        gicp->setNumThreads( node->declare_parameter<int>( "reg_num_threads", 0 ) );
        gicp->setTransformationEpsilon( node->declare_parameter<double>( "reg_transformation_epsilon", 0.01 ) );
        gicp->setMaximumIterations( node->declare_parameter<int>( "reg_maximum_iterations", 64 ) );
        gicp->setMaxCorrespondenceDistance( node->declare_parameter<double>( "reg_max_correspondence_distance", 2.5 ) );
        gicp->setCorrespondenceRandomness( node->declare_parameter<int>( "reg_correspondence_randomness", 20 ) );
        return gicp;
    }
#ifdef USE_VGICP_CUDA
    else if( registration_method == "FAST_VGICP_CUDA" ) {
        std::cout << "registration: FAST_VGICP_CUDA" << std::endl;
        fast_gicp::FastVGICPCuda<PointT, PointT>::Ptr vgicp( new fast_gicp::FastVGICPCuda<PointT, PointT>() );
        vgicp->setResolution( node->declare_parameter<double>( "reg_resolution", 1.0 ) );
        vgicp->setTransformationEpsilon( node->declare_parameter<double>( "reg_transformation_epsilon", 0.01 ) );
        vgicp->setMaximumIterations( node->declare_parameter<int>( "reg_maximum_iterations", 64 ) );
        vgicp->setCorrespondenceRandomness( node->declare_parameter<int>( "reg_correspondence_randomness", 20 ) );
        return vgicp;
    }
#endif
    else if( registration_method == "FAST_VGICP" ) {
        std::cout << "registration: FAST_VGICP" << std::endl;
        fast_gicp::FastVGICP<PointT, PointT>::Ptr vgicp( new fast_gicp::FastVGICP<PointT, PointT>() );
        vgicp->setNumThreads( node->declare_parameter<int>( "reg_num_threads", 0 ) );
        vgicp->setResolution( node->declare_parameter<double>( "reg_resolution", 1.0 ) );
        vgicp->setTransformationEpsilon( node->declare_parameter<double>( "reg_transformation_epsilon", 0.01 ) );
        vgicp->setMaximumIterations( node->declare_parameter<int>( "reg_maximum_iterations", 64 ) );
        vgicp->setCorrespondenceRandomness( node->declare_parameter<int>( "reg_correspondence_randomness", 20 ) );
        return vgicp;
    } else if( registration_method == "ICP" ) {
        std::cout << "registration: ICP" << std::endl;
        pcl::IterativeClosestPoint<PointT, PointT>::Ptr icp( new pcl::IterativeClosestPoint<PointT, PointT>() );
        icp->setTransformationEpsilon( node->declare_parameter<double>( "reg_transformation_epsilon", 0.01 ) );
        icp->setMaximumIterations( node->declare_parameter<int>( "reg_maximum_iterations", 64 ) );
        icp->setMaxCorrespondenceDistance( node->declare_parameter<double>( "reg_max_correspondence_distance", 2.5 ) );
        icp->setUseReciprocalCorrespondences( node->declare_parameter<bool>( "reg_use_reciprocal_correspondences", false ) );
        return icp;
    } else if( registration_method.find( "GICP" ) != std::string::npos ) {
        if( registration_method.find( "OMP" ) == std::string::npos ) {
            std::cout << "registration: GICP" << std::endl;
            pcl::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp( new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>() );
            gicp->setTransformationEpsilon( node->declare_parameter<double>( "reg_transformation_epsilon", 0.01 ) );
            gicp->setMaximumIterations( node->declare_parameter<int>( "reg_maximum_iterations", 64 ) );
            gicp->setUseReciprocalCorrespondences( node->declare_parameter<bool>( "reg_use_reciprocal_correspondences", false ) );
            gicp->setMaxCorrespondenceDistance( node->declare_parameter<double>( "reg_max_correspondence_distance", 2.5 ) );
            gicp->setCorrespondenceRandomness( node->declare_parameter<int>( "reg_correspondence_randomness", 20 ) );
            gicp->setMaximumOptimizerIterations( node->declare_parameter<int>( "reg_max_optimizer_iterations", 20 ) );
            return gicp;
        } else {
            std::cout << "registration: GICP_OMP" << std::endl;
            pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(
                new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>() );
            gicp->setTransformationEpsilon( node->declare_parameter<double>( "reg_transformation_epsilon", 0.01 ) );
            gicp->setMaximumIterations( node->declare_parameter<int>( "reg_maximum_iterations", 64 ) );
            gicp->setUseReciprocalCorrespondences( node->declare_parameter<bool>( "reg_use_reciprocal_correspondences", false ) );
            gicp->setMaxCorrespondenceDistance( node->declare_parameter<double>( "reg_max_correspondence_distance", 2.5 ) );
            gicp->setCorrespondenceRandomness( node->declare_parameter<int>( "reg_correspondence_randomness", 20 ) );
            gicp->setMaximumOptimizerIterations( node->declare_parameter<int>( "reg_max_optimizer_iterations", 20 ) );
            return gicp;
        }
    } else {
        if( registration_method.find( "NDT" ) == std::string::npos ) {
            std::cerr << "warning: unknown registration type(" << registration_method << ")" << std::endl;
            std::cerr << "       : use NDT" << std::endl;
        }

        double ndt_resolution = node->declare_parameter<double>( "reg_resolution", 0.5 );
        if( registration_method.find( "OMP" ) == std::string::npos ) {
            std::cout << "registration: NDT " << ndt_resolution << std::endl;
            pcl::NormalDistributionsTransform<PointT, PointT>::Ptr ndt( new pcl::NormalDistributionsTransform<PointT, PointT>() );
            ndt->setTransformationEpsilon( node->declare_parameter<double>( "reg_transformation_epsilon", 0.01 ) );
            ndt->setMaximumIterations( node->declare_parameter<int>( "reg_maximum_iterations", 64 ) );
            ndt->setResolution( ndt_resolution );
            return ndt;
        } else {
            int         num_threads      = node->declare_parameter<int>( "reg_num_threads", 0 );
            std::string nn_search_method = node->declare_parameter<std::string>( "reg_nn_search_method", "DIRECT7" );
            std::cout << "registration: NDT_OMP " << nn_search_method << " " << ndt_resolution << " (" << num_threads << " threads)"
                      << std::endl;
            pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt( new pclomp::NormalDistributionsTransform<PointT, PointT>() );
            if( num_threads > 0 ) {
                ndt->setNumThreads( num_threads );
            }
            ndt->setTransformationEpsilon( node->declare_parameter<double>( "reg_transformation_epsilon", 0.01 ) );
            ndt->setMaximumIterations( node->declare_parameter<int>( "reg_maximum_iterations", 64 ) );
            ndt->setResolution( ndt_resolution );
            if( nn_search_method == "KDTREE" ) {
                ndt->setNeighborhoodSearchMethod( pclomp::KDTREE );
            } else if( nn_search_method == "DIRECT1" ) {
                ndt->setNeighborhoodSearchMethod( pclomp::DIRECT1 );
            } else {
                ndt->setNeighborhoodSearchMethod( pclomp::DIRECT7 );
            }
            return ndt;
        }
    }

    return nullptr;
}

}  // namespace hdl_graph_slam
