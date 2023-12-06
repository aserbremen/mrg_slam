// SPDX-License-Identifier: BSD-2-Clause

#ifndef INFORMATION_MATRIX_CALCULATOR_HPP
#define INFORMATION_MATRIX_CALCULATOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>

namespace mrg_slam {

class InformationMatrixCalculator {
public:
    using PointT = pcl::PointXYZI;

    InformationMatrixCalculator() {}
    InformationMatrixCalculator( rclcpp::Node::SharedPtr _node );
    ~InformationMatrixCalculator();

    static double calc_fitness_score( const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2,
                                      const Eigen::Isometry3d& relpose, double max_range = std::numeric_limits<double>::max() );

    Eigen::MatrixXd calc_information_matrix( const pcl::PointCloud<PointT>::ConstPtr& cloud1,
                                             const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose ) const;

private:
    double weight( double a, double max_x, double min_y, double max_y, double x ) const
    {
        double y = ( 1.0 - std::exp( -a * x ) ) / ( 1.0 - std::exp( -a * max_x ) );
        return min_y + ( max_y - min_y ) * y;
    }

private:
    bool   use_const_inf_matrix;
    double const_stddev_x;
    double const_stddev_q;

    double var_gain_a;
    double min_stddev_x;
    double max_stddev_x;
    double min_stddev_q;
    double max_stddev_q;
    double fitness_score_thresh;
};

}  // namespace mrg_slam

#endif  // INFORMATION_MATRIX_CALCULATOR_HPP
