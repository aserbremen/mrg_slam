// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef KKL_G2O_EDGE_SE3_RANGING_HPP
#define KKL_G2O_EDGE_SE3_RANGING_HPP

#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <Eigen/Core>

namespace g2o {

class EdgeSE3Ranging : public BaseBinaryEdge<1, double, VertexSE3, VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3Ranging() : BaseBinaryEdge<1, double, VertexSE3, VertexSE3>() {}

    virtual void computeError() override
    {
        const VertexSE3* v1 = static_cast<const VertexSE3*>( _vertices[0] );
        const VertexSE3* v2 = static_cast<const VertexSE3*>( _vertices[1] );

        _error[0] = ( v1->estimate().translation() - v2->estimate().translation() ).norm() - _measurement;
    }

    virtual void setMeasurement( const double& m ) override { _measurement = m; }

    virtual bool read( std::istream& is ) override
    {
        double meas;
        is >> meas;
        setMeasurement( meas );
        is >> information()( 0, 0 );
        return true;
    }

    virtual bool write( std::ostream& os ) const override
    {
        double meas = _measurement;
        os << meas << " " << information()( 0, 0 );
        return os.good();
    }
};

}  // namespace g2o

#endif  // KKL_G2O_EDGE_SE3_RANGING_HPP