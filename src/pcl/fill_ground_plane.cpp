/*
Copyright (C) 2017 S. Kasperski
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are
met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this
software without specific prior written permission. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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
