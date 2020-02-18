/*
 * Copyright 2018 Southwest Research Institute
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

#include "crs_area_selection/area_selector.h"

//#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/region_growing.h>

#include "crs_area_selection/filter.h"

namespace
{
bool determinantComparator(const std::pair<Eigen::MatrixXd, Eigen::VectorXd>& a,
                           const std::pair<Eigen::MatrixXd, Eigen::VectorXd>& b)
{
  return std::abs(a.first.determinant()) < std::abs(b.first.determinant());
}

float sqrDistance(const pcl::PointXYZ& a, const pcl::PointXYZ& b)
{
  float d = static_cast<float>(std::pow((a.x - b.x), 2) + std::pow((a.y - b.y), 2) + std::pow((a.z - b.z), 2));
  return d;
}

bool clusterComparator(const pcl::PointIndices& a,
                       const pcl::PointIndices& b,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       const pcl::PointXYZ& origin)
{
  // Add cluster indices to vector
  std::vector<pcl::PointIndices> cluster_indices;
  cluster_indices.push_back(a);
  cluster_indices.push_back(b);

  // Find the distance between the selection polygon origin and the centroid of each cluster
  std::vector<float> dist;
  for (auto cluster_it = cluster_indices.begin(); cluster_it != cluster_indices.end(); ++cluster_it)
  {
    // Get the centroid of the cluster
    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    for (auto it = cluster_it->indices.begin(); it != cluster_it->indices.end(); ++it)
    {
      centroid.add(cloud->points[static_cast<std::size_t>(*it)]);
    }
    pcl::PointXYZ centroid_pt;
    centroid.get(centroid_pt);
    dist.push_back(sqrDistance(centroid_pt, origin));
  }

  return dist.front() < dist.back();
}

}  // namespace

namespace crs_area_selection
{
boost::optional<FittedPlane> AreaSelector::fitPlaneToPoints(const std::vector<Eigen::Vector3d>& points,
                                                            const AreaSelectorParameters& params)
{
  // Create a point cloud from the selection points
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::CentroidPoint<pcl::PointXYZ> centroid;
  for (auto it = points.begin(); it != points.end(); ++it)
  {
    pcl::PointXYZ pt;
    pt.x = static_cast<float>((*it)(0));
    pt.y = static_cast<float>((*it)(1));
    pt.z = static_cast<float>((*it)(2));

    input_cloud->points.push_back(pt);
    centroid.add(pt);
  }

  pcl::PointXYZ origin;
  centroid.get(origin);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (!crs_area_selection::data_filtering::planeFit<pcl::PointXYZ>(
          input_cloud, *output_cloud, coefficients, params.plane_distance_threshold))
  {
    //    PCL_ERROR("Unable to fit points to plane model");  //Try this
    return {};
  }

  FittedPlane plane;
  plane.normal(0) = static_cast<double>(coefficients->values[0]);
  plane.normal(1) = static_cast<double>(coefficients->values[1]);
  plane.normal(2) = static_cast<double>(coefficients->values[2]);
  plane.normal.normalize();

  plane.origin(0) = static_cast<double>(origin.x);
  plane.origin(1) = static_cast<double>(origin.y);
  plane.origin(2) = static_cast<double>(origin.z);

  return boost::make_optional(plane);
}

 pcl::PointCloud<pcl::PointXYZ>::Ptr AreaSelector::projectPointsOntoPlane(const std::vector<Eigen::Vector3d>& points,
                                                                         const FittedPlane& plane)
{
  pcl::PointCloud<pcl::PointXYZ> projected_points;
  for (const Eigen::Vector3d& current_pt : points)
  {
    // Create a vector from plane origin to point
    Eigen::Vector3d vec(current_pt - plane.origin);

    // Project the vector connecting the origin and point onto the plane normal to get the distance that the point is
    // from the plane
    double dp = plane.normal.dot(vec);

    // Move the point from its current location along the plane's normal by its distance from the plane
    Eigen::Vector3d proj_pt = current_pt - dp * plane.normal;

    // Convert each point to PCL format
    pcl::PointXYZ pt;
    pt.x = static_cast<float>(proj_pt(0));
    pt.y = static_cast<float>(proj_pt(1));
    pt.z = static_cast<float>(proj_pt(2));

    projected_points.points.push_back(pt);
  }

  return projected_points.makeShared();
}

 std::vector<int> AreaSelector::getPointsInROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                              const pcl::PointCloud<pcl::PointXYZ>::Ptr proj_sel_points,
                                              const FittedPlane& plane,
                                              const AreaSelectorParameters& params)
{
  // Calculate the diagonal of search_points_ cloud
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  double half_dist = std::sqrt(static_cast<double>(sqrDistance(min_pt, max_pt))) / 2.0;

  // Extrude the convex hull by half the max distance
  pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
  pcl::PointIndicesPtr selection_indices(new pcl::PointIndices{});
  prism.setInputCloud(cloud);
  prism.setInputPlanarHull(proj_sel_points);
  prism.setHeightLimits(-half_dist, half_dist);
  prism.segment(*selection_indices);

  if (selection_indices->indices.empty())
  {
//    ROS_ERROR("No points found within selection area");
    return {};
  }

  // Pull out the points that are inside the user selected prism
  pcl::PointCloud<pcl::PointXYZ>::Ptr prism_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(selection_indices);
  extract.setNegative(false);
  extract.filter(*prism_cloud);

  // Estimate the normals for each point in the user selection
  pcl::search::Search<pcl::PointXYZ>::Ptr tree =
      boost::shared_ptr<pcl::search::Search<pcl::PointXYZ>>(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(prism_cloud);
  normal_estimator.setRadiusSearch(params.normal_est_radius);
  normal_estimator.compute(*normals);

  // Perform region growing using euclidian distance and normals to estimate connectedness
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize(params.min_cluster_size);
  reg.setMaxClusterSize(params.max_cluster_size);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(static_cast<unsigned int>(params.region_growing_nneighbors));
  reg.setInputCloud(prism_cloud);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(static_cast<float>(params.region_growing_smoothness / 180.0 * M_PI));
  reg.setCurvatureThreshold(static_cast<float>(params.region_growing_curvature));

  std::vector<pcl::PointIndices> int_indices;
  reg.extract(int_indices);

  // We need to translate the indices into the prism cloud into indices into the original cloud (via selection
//  indices)
  std::vector<pcl::PointIndices> cluster_indices; cluster_indices.reserve(int_indices.size());
  for (const auto& index_set : int_indices)
  {
    pcl::PointIndices output_set;
    output_set.header = index_set.header;
    for (const auto i : index_set.indices)
    {
      output_set.indices.push_back(selection_indices->indices[static_cast<std::size_t>(i)]);
    }
    cluster_indices.push_back(output_set);
  }

  if (cluster_indices.size() > 1)
  {
//    ROS_INFO("%lu clusters found in ROI selection; choosing closest cluster to ROI selection centroid",
//             cluster_indices.size());

    // Multiple clusters of points found, so find the cluster closest to the centroid of the selection polygon
    pcl::PointXYZ polygon_centroid;
    polygon_centroid.x = static_cast<float>(plane.origin(0));
    polygon_centroid.y = static_cast<float>(plane.origin(1));
    polygon_centroid.z = static_cast<float>(plane.origin(2));

    // Find the closest cluster
    auto closest_cluster = std::min_element(cluster_indices.begin(),
                                            cluster_indices.end(),
                                            boost::bind(&clusterComparator, _1, _2, cloud, polygon_centroid));
    return closest_cluster->indices;
  }
  else
  {
    // Either one or zero clusters were found, so just return all of the points in the extruded hull
    return selection_indices->indices;
  }
}

std::vector<int> AreaSelector::getRegionOfInterest(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                                   const std::vector<Eigen::Vector3d>& points,
                                                   const AreaSelectorParameters& params)
{
  // Check size of selection points vector
  if (points.size() < 3)
  {
    //    ROS_ERROR("Not enough points to create a closed loop");
    return {};
  }

  // Check the size of the input point cloud
  if (input_cloud->points.size() == 0)
  {
    //    ROS_ERROR("No points to search for inside the selection polygon");
    return {};
  }

  // Take selection points (points_) and fit a plane to them using RANSAC
  boost::optional<FittedPlane> plane = fitPlaneToPoints(points, params);
  if (!plane)
  {
    //    ROS_ERROR("Failed to fit plane to selection points");
    return {};
  }

    // Project the selection points onto the fitted plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_sel_points = projectPointsOntoPlane(points, plane.get());

    /* Find all of the sensor data points inside a volume whose perimeter is defined by the projected selection points
     * and whose depth is defined by the largest length of the bounding box of the input point cloud */
    std::vector<int> roi_cloud_indices = getPointsInROI(input_cloud, proj_sel_points, plane.get(), params);
    if (roi_cloud_indices.empty())
    {
//      ROS_ERROR("Unable to identify points in the region of interest");
      return {};
    }

//  std::vector<int> roi_cloud_indices;
  return roi_cloud_indices;
}

}  // namespace crs_area_selection
