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
    InformationMatrixCalculator( rclcpp::Node::SharedPtr node );
    ~InformationMatrixCalculator();

    static double calc_fitness_score( const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2,
                                      const Eigen::Isometry3d& relpose, double max_range = std::numeric_limits<double>::max() );

    Eigen::MatrixXd calc_information_matrix( const pcl::PointCloud<PointT>::ConstPtr& cloud1,
                                             const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose ) const;

private:
    double weight( double a, double max_x, double min_y, double max_y, double x ) const;

    rclcpp::Node::SharedPtr node_;
};

}  // namespace mrg_slam

#endif  // INFORMATION_MATRIX_CALCULATOR_HPP
