#include <pcl/fill_ground_plane.hpp>

namespace mrg_slam_pcl {

void
fill_ground_plane( pcl::PointCloud<PointType>::Ptr cloud, double radius, double map_resolution )
{
    pcl::SampleConsensusModelPlane<PointType>::Ptr model( new pcl::SampleConsensusModelPlane<PointType>( cloud ) );
    pcl::RandomSampleConsensus<PointType>          ransac( model );
    ransac.setDistanceThreshold( .01 );
    ransac.computeModel();

    Eigen::VectorXf c;
    ransac.getModelCoefficients( c );
    Eigen::Matrix<double, 3, 1>  normal( c[0], c[1], c[2] );
    Eigen::Hyperplane<double, 3> plane( normal, c[3] );
    double                       angle_inc = map_resolution / radius;
    for( double r = map_resolution; r <= radius; r += map_resolution ) {
        Eigen::Matrix<double, 3, 1> sample = plane.projection( Eigen::Matrix<double, 3, 1>( r, 0, 0 ) );
        for( double angle = 0; angle < 2 * M_PI; angle += angle_inc ) {
            Eigen::Matrix<double, 3, 1> rot = Eigen::AngleAxis<double>( angle, normal ).toRotationMatrix() * sample;
            PointType                   p;
            p.x = rot[0];
            p.y = rot[1];
            p.z = rot[2];
            cloud->push_back( p );
        }
    }
}

}  // namespace mrg_slam_pcl
