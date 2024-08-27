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

#pragma once

#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>

#include <pcl/features/boundary.h>

namespace pcl
{
  /** \brief BoundaryEstimation estimates whether a set of points is lying on surface boundaries using an angle
    * criterion. The code makes use of the estimated surface normals at each point in the input dataset.
    *
    * Here's an example for estimating boundary points for a PointXYZ point cloud:
    * \code
    * pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    * // fill in the cloud data here
    * 
    * pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    * // estimate normals and fill in \a normals
    *
    * pcl::PointCloud<pcl::Boundary> boundaries;
    * pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
    * est.setInputCloud (cloud);
    * est.setInputNormals (normals);
    * est.setRadiusSearch (0.02);   // 2cm radius
    * est.setSearchMethod (typename pcl::search::KdTree<PointXYZ>::Ptr (new pcl::search::KdTree<PointXYZ>)
    * est.compute (boundaries);
    * \endcode
    *
    * \attention 
    * The convention for Boundary features is:
    *   - if a query point's nearest neighbors cannot be estimated, the boundary feature will be set to NaN 
    *     (not a number)
    *   - it is impossible to estimate a boundary property for a point that
    *     doesn't have finite 3D coordinates. Therefore, any point that contains
    *     NaN data on x, y, or z, will have its boundary feature property set to NaN.
    *
    * \author Radu B. Rusu
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointOutT>
  class BoundaryEstimationOMP : public BoundaryEstimation<PointInT, PointNT, PointOutT>
  {
    public:
      using Ptr = shared_ptr<BoundaryEstimationOMP<PointInT, PointNT, PointOutT> >;
      using ConstPtr = shared_ptr<const BoundaryEstimationOMP<PointInT, PointNT, PointOutT> >;

      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::tree_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      using PointCloudOut = typename Feature<PointInT, PointOutT>::PointCloudOut;

      using BoundaryEstimation<PointInT, PointNT, PointOutT>::angle_threshold_;

    public:
      /** \brief Empty constructor. 
        * The angular threshold \a angle_threshold_ is set to M_PI / 2.0
        */
      BoundaryEstimationOMP ()
      {
        feature_name_ = "BoundaryEstimationOMP";
      };

    protected:
      /** \brief Estimate whether a set of points is lying on surface boundaries using an angle criterion for all points
        * given in <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param[out] output the resultant point cloud model dataset that contains boundary point estimates
        */
      void 
      computeFeature (PointCloudOut &output) override;
  };
}

#include "boundary_omp.hpp"
