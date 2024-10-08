/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_FEATURES_IMPL_BOUNDARY_OMP_H_
#define PCL_FEATURES_IMPL_BOUNDARY_OMP_H_

#include <cfloat>
#include <omp.h>
#include "boundary_omp.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::BoundaryEstimationOMP<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  Eigen::Vector4f u = Eigen::Vector4f::Zero (), v = Eigen::Vector4f::Zero ();

  output.is_dense = true;
  // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
  if (input_->is_dense)
  {
    // Iterating over the entire index vector
    #pragma omp parallel for \
      default(none) \
      shared(output) \
      firstprivate(nn_indices, nn_dists, u, v)
      // num_threads(threads_)
      // schedule(dynamic, chunk_size_)
    for (std::size_t idx = 0; idx < indices_->size (); ++idx)
    {
      if(this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
      {
        output.points[idx].boundary_point = std::numeric_limits<std::uint8_t>::quiet_NaN ();
        output.is_dense = false;
        continue;
      }

      // Obtain a coordinate system on the least-squares plane
      //v = normals_->points[(*indices_)[idx]].getNormalVector4fMap ().unitOrthogonal ();
      //u = normals_->points[(*indices_)[idx]].getNormalVector4fMap ().cross3 (v);
      this->getCoordinateSystemOnPlane (normals_->points[(*indices_)[idx]], u, v);

      // Estimate whether the point is lying on a boundary surface or not
      output.points[idx].boundary_point = this->isBoundaryPoint (*surface_, input_->points[(*indices_)[idx]], nn_indices, u, v, angle_threshold_);
    }
  }
  else
  {
    // Iterating over the entire index vector
    #pragma omp parallel for \
      default(none) \
      shared(output) \
      firstprivate(nn_indices, nn_dists, u, v)
      // num_threads(threads_)
      // schedule(dynamic, chunk_size_)
    for (std::size_t idx = 0; idx < indices_->size (); ++idx)
    {
      if(!isFinite((*input_)[(*indices_)[idx]]) ||
          this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
      {
        output.points[idx].boundary_point = std::numeric_limits<std::uint8_t>::quiet_NaN ();
        output.is_dense = false;
        continue;
      }

      // Obtain a coordinate system on the least-squares plane
      //v = normals_->points[(*indices_)[idx]].getNormalVector4fMap ().unitOrthogonal ();
      //u = normals_->points[(*indices_)[idx]].getNormalVector4fMap ().cross3 (v);
      this->getCoordinateSystemOnPlane (normals_->points[(*indices_)[idx]], u, v);

      // Estimate whether the point is lying on a boundary surface or not
      output.points[idx].boundary_point = this->isBoundaryPoint (*surface_, input_->points[(*indices_)[idx]], nn_indices, u, v, angle_threshold_);
    }
  }
}

// #define PCL_INSTANTIATE_BoundaryEstimation(PointInT,PointNT,PointOutT) template class PCL_EXPORTS pcl::BoundaryEstimation<PointInT, PointNT, PointOutT>;

#endif    // PCL_FEATURES_IMPL_BOUNDARY_OMP_H_

