// SPDX-License-Identifier: BSD-2-Clause

#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

#include <mrg_slam/information_matrix_calculator.hpp>

namespace mrg_slam {

InformationMatrixCalculator::InformationMatrixCalculator( rclcpp::Node::SharedPtr node ) { node_ = node; }

InformationMatrixCalculator::~InformationMatrixCalculator() {}

Eigen::MatrixXd
InformationMatrixCalculator::calc_information_matrix( const pcl::PointCloud<PointT>::ConstPtr& cloud1,
                                                      const pcl::PointCloud<PointT>::ConstPtr& cloud2,
                                                      const Eigen::Isometry3d&                 relpose ) const
{
    if( node_->get_parameter( "use_const_inf_matrix" ).as_bool() ) {
        Eigen::MatrixXd inf = Eigen::MatrixXd::Identity( 6, 6 );
        inf.topLeftCorner( 3, 3 ).array() /= node_->get_parameter( "const_stddev_x" ).as_double();
        inf.bottomRightCorner( 3, 3 ).array() /= node_->get_parameter( "const_stddev_q" ).as_double();
        return inf;
    }

    double fitness_score = calc_fitness_score( cloud1, cloud2, relpose );

    double min_var_x = std::pow( node_->get_parameter( "min_stddev_x" ).as_double(), 2 );
    double max_var_x = std::pow( node_->get_parameter( "max_stddev_x" ).as_double(), 2 );
    double min_var_q = std::pow( node_->get_parameter( "min_stddev_q" ).as_double(), 2 );
    double max_var_q = std::pow( node_->get_parameter( "max_stddev_q" ).as_double(), 2 );

    double w_x = weight( node_->get_parameter( "var_gain_a" ).as_double(), node_->get_parameter( "fitness_score_thresh" ).as_double(),
                         min_var_x, max_var_x, fitness_score );
    double w_q = weight( node_->get_parameter( "var_gain_a" ).as_double(), node_->get_parameter( "fitness_score_thresh" ).as_double(),
                         min_var_q, max_var_q, fitness_score );

    Eigen::MatrixXd inf = Eigen::MatrixXd::Identity( 6, 6 );
    inf.topLeftCorner( 3, 3 ).array() /= w_x;
    inf.bottomRightCorner( 3, 3 ).array() /= w_q;
    return inf;
}

double
InformationMatrixCalculator::calc_fitness_score( const pcl::PointCloud<PointT>::ConstPtr& cloud1,
                                                 const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose,
                                                 double max_range )
{
    pcl::search::KdTree<PointT>::Ptr tree( new pcl::search::KdTree<PointT>() );
    tree->setInputCloud( cloud1 );

    double fitness_score = 0.0;

    // Transform the input dataset using the final transformation
    pcl::PointCloud<PointT> input_transformed;
    pcl::transformPointCloud( *cloud2, input_transformed, relpose.cast<float>() );

    std::vector<int>   nn_indices( 1 );
    std::vector<float> nn_dists( 1 );

    // For each point in the source dataset
    int nr = 0;
    for( size_t i = 0; i < input_transformed.points.size(); ++i ) {
        // Find its nearest neighbor in the target
        tree->nearestKSearch( input_transformed.points[i], 1, nn_indices, nn_dists );

        // Deal with occlusions (incomplete targets)
        if( nn_dists[0] <= max_range ) {
            // Add to the fitness score
            fitness_score += nn_dists[0];
            nr++;
        }
    }

    if( nr > 0 )
        return ( fitness_score / nr );
    else
        return ( std::numeric_limits<double>::max() );
}

double
InformationMatrixCalculator::weight( double a, double max_x, double min_y, double max_y, double x ) const
{
    double y = ( 1.0 - std::exp( -a * x ) ) / ( 1.0 - std::exp( -a * max_x ) );
    return min_y + ( max_y - min_y ) * y;
}

}  // namespace mrg_slam
