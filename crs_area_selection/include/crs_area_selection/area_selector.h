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

#ifndef CRS_AREA_SELECTION_AREA_SELECTOR_H
#define CRS_AREA_SELECTION_AREA_SELECTOR_H

#include <boost/optional.hpp>
#include <eigen3/Eigen/Core>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
//#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "crs_area_selection/area_selector_parameters.h"

namespace crs_area_selection
{
struct FittedPlane
{
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
};

/**
 * @brief The AreaSelector class takes a set of 3D points defining a closed polygon and attempts to identify which
 * points in a published point cloud lie within the bounds of that polygon. The class fits a plane to the input points,
 * projects the input points/polygon onto the fitted plane, extrudes a volume defined by the projected polygon to the
 * extents of the searched point cloud, and finds the points in the searched point cloud within the volume. If multiple
 * clusters of points are found, the cluster closest to the centroid of the input points is selected.
 */
class AreaSelector
{
public:
  /**
   * @brief AreaSelector class constructor
   */
  AreaSelector() {}

  /**
   * @brief Finds the points that lie within the selection boundary
   * @param roi_cloud_msg
   * @return Returns false if there are less than 3 selection boundary points or if there are no search points;
   * otherwise returns true.
   */
  std::vector<int> getRegionOfInterest(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                       const std::vector<Eigen::Vector3d>& points,
                                       const AreaSelectorParameters& params);

protected:
  boost::optional<FittedPlane> fitPlaneToPoints(const std::vector<Eigen::Vector3d>& points,
                                                const AreaSelectorParameters& params);

  //  pcl::PointCloud<pcl::PointXYZ>::Ptr projectPointsOntoPlane(const std::vector<Eigen::Vector3d>& points,
  //                                                             const FittedPlane& plane);

  //  std::vector<int> getPointsInROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
  //                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr proj_sel_points,
  //                                  const FittedPlane& plane,
  //                                  const AreaSelectorParameters& params);
};

}  // namespace crs_area_selection

#endif  // OPP_AREA_SELECTION_AREA_SELECTOR_H
