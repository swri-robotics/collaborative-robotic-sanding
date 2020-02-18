/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CRS_AREA_SELECTION_FILTER_H
#define CRS_AREA_SELECTION_FILTER_H

#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/sac_model_plane.h>

namespace crs_area_selection
{
namespace data_filtering
{
template <typename PointT>
using Cloud = typename pcl::PointCloud<PointT>;

template <typename PointT>
using CloudPtr = typename Cloud<PointT>::Ptr;

template <typename PointT>
bool planeFit(const CloudPtr<PointT> input_cloud,
              Cloud<PointT>& output_cloud,
              pcl::ModelCoefficients::Ptr plane_coefficients,
              const double threshold = 0.025);

}  // end namespace data_filtering

}  // end namespace crs_area_selection

#include "crs_area_selection/filter_impl.h"

#endif  // OPP_AREA_SELECTION_FILTER_H
