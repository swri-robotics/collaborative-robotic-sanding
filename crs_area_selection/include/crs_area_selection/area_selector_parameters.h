/*
 * Copyright 2020 Southwest Research Institute
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

#ifndef CRS_AREA_SELECTION_AREA_SELECTOR_PARAMETERS_H
#define CRS_AREA_SELECTION_AREA_SELECTOR_PARAMETERS_H

#include <limits>

namespace crs_area_selection
{
struct AreaSelectorParameters
{
  double cluster_tolerance = 0.25;
  int min_cluster_size = 100;
  int max_cluster_size = std::numeric_limits<int>::max();
  double plane_distance_threshold = 1.0;

  double normal_est_radius = 0.17;
  int region_growing_nneighbors = 10;
  double region_growing_smoothness = 5.0;
  double region_growing_curvature = 1.0;
};

}  // namespace crs_area_selection

#endif  // OPP_AREA_SELECTION_AREA_SELECTOR_PARAMETERS_H
