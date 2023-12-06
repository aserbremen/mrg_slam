// SPDX-License-Identifier: BSD-2-Clause

#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <iostream>
#include <mrg_slam/registrations.hpp>

// ndt and fast_gicp includes
#include <pclomp/gicp_omp.h>
#include <pclomp/ndt_omp.h>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

#ifdef USE_VGICP_CUDA
#    include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif

namespace mrg_slam {

pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr
select_registration_method( rclcpp::Node* node_raw_ptr )
{
    using PointT = pcl::PointXYZI;

    // Get all necessary parameters locally for selected registration method
    std::string registration_method             = node_raw_ptr->get_parameter( "registration_method" ).as_string();         // "FAST_GICP"
    int         reg_num_threads                 = node_raw_ptr->get_parameter( "reg_num_threads" ).as_int();                // 0
    double      reg_transformation_epsilon      = node_raw_ptr->get_parameter( "reg_transformation_epsilon" ).as_double();  // 0.01
    int         reg_maximum_iterations          = node_raw_ptr->get_parameter( "reg_maximum_iterations" ).as_int();         // 64
    double      reg_max_correspondence_distance = node_raw_ptr->get_parameter( "reg_max_correspondence_distance" ).as_double();   // 2.0
    int         reg_max_optimizer_iterations    = node_raw_ptr->get_parameter( "reg_max_optimizer_iterations" ).as_int();         // 20
    bool   reg_use_reciprocal_correspondences   = node_raw_ptr->get_parameter( "reg_use_reciprocal_correspondences" ).as_bool();  // false
    int    reg_correspondence_randomness        = node_raw_ptr->get_parameter( "reg_correspondence_randomness" ).as_int();        // 20
    double reg_resolution                       = node_raw_ptr->get_parameter( "reg_resolution" ).as_double();  // NDT_OMP 0.5, rest 1.0
    std::string reg_nn_search_method            = node_raw_ptr->get_parameter( "reg_nn_search_method" ).as_string();  // "DIRECT7"

    // select a registration method (ICP, GICP, NDT)
    if( registration_method == "FAST_GICP" ) {
        std::cout << "registration: FAST_GICP" << std::endl;
        fast_gicp::FastGICP<PointT, PointT>::Ptr gicp( new fast_gicp::FastGICP<PointT, PointT>() );
        gicp->setNumThreads( reg_num_threads );
        gicp->setTransformationEpsilon( reg_transformation_epsilon );
        gicp->setMaximumIterations( reg_maximum_iterations );
        gicp->setMaxCorrespondenceDistance( reg_max_correspondence_distance );
        gicp->setCorrespondenceRandomness( reg_correspondence_randomness );
        return gicp;
    }
#ifdef USE_VGICP_CUDA
    else if( registration_method == "FAST_VGICP_CUDA" ) {
        std::cout << "registration: FAST_VGICP_CUDA" << std::endl;
        fast_gicp::FastVGICPCuda<PointT, PointT>::Ptr vgicp( new fast_gicp::FastVGICPCuda<PointT, PointT>() );
        vgicp->setResolution( reg_resolution );
        vgicp->setTransformationEpsilon( reg_transformation_epsilon );
        vgicp->setMaximumIterations( reg_maximum_iterations );
        vgicp->setCorrespondenceRandomness( reg_correspondence_randomness );
        return vgicp;
    }
#endif
    else if( registration_method == "FAST_VGICP" ) {
        std::cout << "registration: FAST_VGICP" << std::endl;
        fast_gicp::FastVGICP<PointT, PointT>::Ptr vgicp( new fast_gicp::FastVGICP<PointT, PointT>() );
        vgicp->setNumThreads( reg_num_threads );
        vgicp->setResolution( reg_resolution );
        vgicp->setTransformationEpsilon( reg_transformation_epsilon );
        vgicp->setMaximumIterations( reg_maximum_iterations );
        vgicp->setCorrespondenceRandomness( reg_correspondence_randomness );
        return vgicp;
    } else if( registration_method == "ICP" ) {
        std::cout << "registration: ICP" << std::endl;
        pcl::IterativeClosestPoint<PointT, PointT>::Ptr icp( new pcl::IterativeClosestPoint<PointT, PointT>() );
        icp->setTransformationEpsilon( reg_transformation_epsilon );
        icp->setMaximumIterations( reg_maximum_iterations );
        icp->setMaxCorrespondenceDistance( reg_max_correspondence_distance );
        icp->setUseReciprocalCorrespondences( reg_use_reciprocal_correspondences );
        return icp;
    } else if( registration_method.find( "GICP" ) != std::string::npos ) {
        if( registration_method.find( "OMP" ) == std::string::npos ) {
            std::cout << "registration: GICP" << std::endl;
            pcl::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp( new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>() );
            gicp->setTransformationEpsilon( reg_transformation_epsilon );
            gicp->setMaximumIterations( reg_maximum_iterations );
            gicp->setUseReciprocalCorrespondences( reg_use_reciprocal_correspondences );
            gicp->setMaxCorrespondenceDistance( reg_max_correspondence_distance );
            gicp->setCorrespondenceRandomness( reg_correspondence_randomness );
            gicp->setMaximumOptimizerIterations( reg_max_optimizer_iterations );
            return gicp;
        } else {
            std::cout << "registration: GICP_OMP" << std::endl;
            pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(
                new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>() );
            gicp->setTransformationEpsilon( reg_transformation_epsilon );
            gicp->setMaximumIterations( reg_maximum_iterations );
            gicp->setUseReciprocalCorrespondences( reg_use_reciprocal_correspondences );
            gicp->setMaxCorrespondenceDistance( reg_max_correspondence_distance );
            gicp->setCorrespondenceRandomness( reg_correspondence_randomness );
            gicp->setMaximumOptimizerIterations( reg_max_optimizer_iterations );
            return gicp;
        }
    } else {
        if( registration_method.find( "NDT" ) == std::string::npos ) {
            std::cerr << "warning: unknown registration type(" << registration_method << ")" << std::endl;
            std::cerr << "       : use NDT" << std::endl;
        }

        double ndt_resolution = reg_resolution;
        if( registration_method.find( "OMP" ) == std::string::npos ) {
            std::cout << "registration: NDT " << ndt_resolution << std::endl;
            pcl::NormalDistributionsTransform<PointT, PointT>::Ptr ndt( new pcl::NormalDistributionsTransform<PointT, PointT>() );
            ndt->setTransformationEpsilon( reg_transformation_epsilon );
            ndt->setMaximumIterations( reg_maximum_iterations );
            ndt->setResolution( ndt_resolution );
            return ndt;
        } else {
            std::cout << "registration: NDT_OMP " << reg_nn_search_method << " " << ndt_resolution << " (" << reg_num_threads << " threads)"
                      << std::endl;
            pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt( new pclomp::NormalDistributionsTransform<PointT, PointT>() );
            if( reg_num_threads > 0 ) {
                ndt->setNumThreads( reg_num_threads );
            }
            ndt->setTransformationEpsilon( reg_transformation_epsilon );
            ndt->setMaximumIterations( reg_maximum_iterations );
            ndt->setResolution( ndt_resolution );
            if( reg_nn_search_method == "KDTREE" ) {
                ndt->setNeighborhoodSearchMethod( pclomp::KDTREE );
            } else if( reg_nn_search_method == "DIRECT1" ) {
                ndt->setNeighborhoodSearchMethod( pclomp::DIRECT1 );
            } else {
                ndt->setNeighborhoodSearchMethod( pclomp::DIRECT7 );
            }
            return ndt;
        }
    }

    return nullptr;
}

}  // namespace mrg_slam
