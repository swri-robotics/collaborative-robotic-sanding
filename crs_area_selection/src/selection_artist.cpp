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

#include "crs_area_selection/selection_artist.h"

#include <rclcpp/rclcpp.hpp>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <pcl_conversions/pcl_conversions.h>

//#include <eigen_conversions/eigen_msg.h>
//#include <opp_msgs_serialization/serialize.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>

#include "crs_area_selection/area_selector.h"
#include "crs_area_selection/area_selector_parameters.h"

namespace crs_area_selection
{
const static double TF_LOOKUP_TIMEOUT = 5.0;
const static std::string MARKER_ARRAY_TOPIC = "roi_markers";
const static std::string CLICKED_POINT_TOPIC = "clicked_point";
const static std::string CLEAR_ROI_POINTS_SERVICE = "clear_selection_points";
const static std::string COLLECT_ROI_POINTS_SERVICE = "collect_selection_points";
const static std::string package_share_directory = ament_index_cpp::get_package_share_directory("crs_area_selection");
const static std::string area_selection_config_file = package_share_directory + "/config/"
                                                                                "area_selection_parameters.yaml";

}  // namespace crs_area_selection

namespace
{
std::vector<visualization_msgs::msg::Marker> makeVisual(const std::string& frame_id)
{
  visualization_msgs::msg::Marker points, lines;
  points.header.frame_id = lines.header.frame_id = frame_id;
  points.ns = lines.ns = "roi_selection";
  points.action = lines.action = visualization_msgs::msg::Marker::ADD;

  // Point specific properties
  points.id = 0;
  points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  points.scale.x = points.scale.y = 0.1;
  points.color.r = points.color.a = 1.0;
  points.pose.orientation.w = 1.0;

  // Line specific properties
  lines.id = 1;
  lines.type = visualization_msgs::msg::Marker::LINE_STRIP;
  lines.scale.x = lines.scale.y = 0.05;
  lines.color.r = lines.color.a = 1.0;
  lines.pose.orientation.w = 1.0;

  std::vector<visualization_msgs::msg::Marker> visuals;
  visuals.push_back(points);
  visuals.push_back(lines);
  return visuals;
}

bool pclToShapeMsg(const pcl::PolygonMesh& pcl_mesh, shape_msgs::msg::Mesh& mesh_msg)
{
  // Make sure that there are at least three points and at least one polygon
  if (pcl_mesh.cloud.height * pcl_mesh.cloud.width < 3 || pcl_mesh.polygons.size() < 1)
  {
    return false;
  }

  // Prepare the message's vectors to receive data
  // One resize now saves time later
  mesh_msg.vertices.resize(pcl_mesh.cloud.height * pcl_mesh.cloud.width);
  mesh_msg.triangles.resize(pcl_mesh.polygons.size());

  // Get the points from the pcl mesh
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(pcl_mesh.cloud, vertices);

  // Copy the coordinates inside the vertices into the new mesh
  // TODO: Maybe check for nan?
  for (std::size_t i = 0; i < vertices.size(); ++i)
  {
    mesh_msg.vertices[i].x = static_cast<double>(vertices[i]._PointXYZ::x);
    mesh_msg.vertices[i].y = static_cast<double>(vertices[i]._PointXYZ::y);
    mesh_msg.vertices[i].z = static_cast<double>(vertices[i]._PointXYZ::z);
  }

  // Copy the vertex indices (which describe each polygon) from
  // the old mesh to the new mesh
  for (std::size_t i = 0; i < pcl_mesh.polygons.size(); ++i)
  {
    // If a 'polygon' in the old mesh did not have 3 or more vertices,
    // throw an error.  (If the polygon has 4 or more, just use the first
    // 3 to make a triangle.)
    // TODO: It is possible to decompose any polygon into multiple
    // triangles, and that would prevent loss of data here.
    if (pcl_mesh.polygons[i].vertices.size() < 3)
    {
      return false;
    }
    mesh_msg.triangles[i].vertex_indices[0] = pcl_mesh.polygons[i].vertices[0];
    mesh_msg.triangles[i].vertex_indices[1] = pcl_mesh.polygons[i].vertices[1];
    mesh_msg.triangles[i].vertex_indices[2] = pcl_mesh.polygons[i].vertices[2];
  }

  return true;
}

bool pclFromShapeMsg(const shape_msgs::msg::Mesh& mesh_msg, pcl::PolygonMesh& pcl_mesh)
{
  // Make sure that there are at least three points and at least one polygon
  if (mesh_msg.vertices.size() < 3 || mesh_msg.triangles.size() < 1)
  {
    return false;
  }

  // Prepare PCL structures to receive data
  // Resizing once now saves time later
  pcl::PointCloud<pcl::PointXYZ> vertices;
  vertices.resize(mesh_msg.vertices.size());
  pcl_mesh.polygons.resize(mesh_msg.triangles.size());

  // Copy the vertex indices (which describe each polygon) from
  // the old mesh to the new mesh
  for (std::size_t i = 0; i < mesh_msg.vertices.size(); ++i)
  {
    vertices[i]._PointXYZ::x = static_cast<float>(mesh_msg.vertices[i].x);
    vertices[i]._PointXYZ::y = static_cast<float>(mesh_msg.vertices[i].y);
    vertices[i]._PointXYZ::z = static_cast<float>(mesh_msg.vertices[i].z);
  }

  // Copy the vertex indices (which describe each polygon) from
  // the old mesh to the new mesh
  for (std::size_t i = 0; i < mesh_msg.triangles.size(); ++i)
  {
    pcl_mesh.polygons[i].vertices.resize(3);
    pcl_mesh.polygons[i].vertices[0] = mesh_msg.triangles[i].vertex_indices[0];
    pcl_mesh.polygons[i].vertices[1] = mesh_msg.triangles[i].vertex_indices[1];
    pcl_mesh.polygons[i].vertices[2] = mesh_msg.triangles[i].vertex_indices[2];
  }

  // Use the filled pointcloud to populate the pcl mesh
  pcl::toPCLPointCloud2(vertices, pcl_mesh.cloud);

  return true;
}

}  // namespace

// namespace YAML
//{
// template <>
// struct convert<crs_area_selection::AreaSelectorParameters>
//{
//  static Node encode(const crs_area_selection::AreaSelectorParameters& rhs)
//  {
//    Node node;
//    node["cluster_tolerance"] = rhs.cluster_tolerance;
//    node["min_cluster_size"] = rhs.min_cluster_size;
//    node["max_cluster_size"] = rhs.max_cluster_size;
//    node["plane_distance_threshold"] = rhs.plane_distance_threshold;
//    node["normal_est_radius"] = rhs.normal_est_radius;
//    node["region_growing_nneighbors"] = rhs.region_growing_nneighbors;
//    node["region_growing_smoothness"] = rhs.region_growing_smoothness;
//    node["region_growing_curvature"] = rhs.region_growing_curvature;
//    return node;
//  }

//  static bool decode(const Node& node, crs_area_selection::AreaSelectorParameters& rhs)
//  {
//    if (node.size() != 8)
//    {
//      return false;
//    }
//    rhs.cluster_tolerance = node["cluster_tolerance"].as<decltype(rhs.cluster_tolerance)>();
//    rhs.min_cluster_size = node["min_cluster_size"].as<decltype(rhs.min_cluster_size)>();
//    rhs.max_cluster_size = node["max_cluster_size"].as<decltype(rhs.max_cluster_size)>();
//    rhs.plane_distance_threshold = node["plane_distance_threshold"].as<decltype(rhs.plane_distance_threshold)>();
//    rhs.normal_est_radius = node["normal_est_radius"].as<decltype(rhs.normal_est_radius)>();
//    rhs.region_growing_nneighbors = node["region_growing_nneighbors"].as<decltype(rhs.region_growing_nneighbors)>();
//    rhs.region_growing_smoothness = node["region_growing_smoothness"].as<decltype(rhs.region_growing_smoothness)>();
//    rhs.region_growing_curvature = node["region_growing_curvature"].as<decltype(rhs.region_growing_curvature)>();
//    return true;
//  }
//};

//}  // namespace YAML

namespace crs_area_selection
{
SelectionArtist::SelectionArtist(rclcpp::Node::SharedPtr node,
                                 const std::string& world_frame,
                                 const std::string& sensor_frame)
  : node_(node)
  , world_frame_(world_frame)
  , sensor_frame_(sensor_frame)
  , clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
  , tf_buffer_(clock_)
  , tf_listener_(tf_buffer_)
{
  marker_pub_.reset();
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(MARKER_ARRAY_TOPIC, 1);

  tf_buffer_.canTransform(sensor_frame_, world_frame_, tf2::TimePointZero, tf2::durationFromSec(TF_LOOKUP_TIMEOUT));

  if (!tf_buffer_.canTransform(
          sensor_frame_, world_frame_, tf2::TimePointZero, tf2::durationFromSec(TF_LOOKUP_TIMEOUT)))
  {
    RCLCPP_ERROR(
        node_->get_logger(), "Transform lookup from %s to %s timed out", sensor_frame_.c_str(), world_frame_.c_str());
    throw std::runtime_error("Transform lookup timed out");
  }
  auto clear_roi_cb = std::bind(&SelectionArtist::clearROIPointsCb, this, std::placeholders::_1, std::placeholders::_2);
  clear_roi_points_srv_ = node_->create_service<std_srvs::srv::Trigger>(CLEAR_ROI_POINTS_SERVICE, clear_roi_cb);

  auto collect_roi_cb =
      std::bind(&SelectionArtist::collectROIPointsCb, this, std::placeholders::_1, std::placeholders::_2);
  collect_roi_points_srv_ =
      node_->create_service<crs_msgs::srv::GetROISelection>(COLLECT_ROI_POINTS_SERVICE, collect_roi_cb);

  // Initialize subscribers and callbacks
  auto drawn_points_cb = std::bind(&SelectionArtist::addSelectionPoint, this, std::placeholders::_1);
  drawn_points_sub_ =
      node->create_subscription<geometry_msgs::msg::PointStamped>(CLICKED_POINT_TOPIC, 1, drawn_points_cb);

  marker_array_.markers = makeVisual(sensor_frame);
}

void SelectionArtist::clearROIPointsCb(const std_srvs::srv::Trigger::Request::SharedPtr req,
                                       std_srvs::srv::Trigger::Response::SharedPtr res)
{
  (void)req;  // To suppress warnings, tell the compiler we will not use this parameter

  for (auto it = marker_array_.markers.begin(); it != marker_array_.markers.end(); ++it)
  {
    it->points.clear();
  }

  marker_pub_->publish(marker_array_);
  res->success = true;
  res->message = "Selection cleared";
}

bool SelectionArtist::collectROIMesh(const shape_msgs::msg::Mesh& mesh_msg,
                                     shape_msgs::msg::Mesh& submesh_msg,
                                     std::string& message)
{
  pcl::PolygonMesh mesh;
  pclFromShapeMsg(mesh_msg, mesh);
  pcl::PointCloud<pcl::PointXYZ> mesh_cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, mesh_cloud);
  crs_msgs::srv::GetROISelection::Request::SharedPtr req;
  pcl::toROSMsg(mesh_cloud, req->input_cloud);

  crs_msgs::srv::GetROISelection::Response::SharedPtr res;
  collectROIPointsCb(req, res);
  if (!res->success)
  {
    submesh_msg = mesh_msg;
    message = res->message;
    return false;
  }

  pcl::PolygonMesh submesh;
  filterMesh(mesh, res->cloud_indices, submesh);
  pclToShapeMsg(submesh, submesh_msg);

  return true;
}

void SelectionArtist::collectROIPointsCb(crs_msgs::srv::GetROISelection::Request::SharedPtr req,
                                         crs_msgs::srv::GetROISelection::Response::SharedPtr res)
{
  auto points_it = std::find_if(marker_array_.markers.begin(),
                                marker_array_.markers.end(),
                                [](const visualization_msgs::msg::Marker& marker) { return marker.id == 0; });

  // Convert the selection points to Eigen vectors
  std::vector<Eigen::Vector3d> points;
  for (const geometry_msgs::msg::Point& pt : points_it->points)
  {
    Eigen::Vector3d e;
    tf2::fromMsg(pt, e);
    points.push_back(std::move(e));
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(req->input_cloud, *cloud);

  AreaSelectorParameters params;
  // Fix this in order to change the parameters via yaml
  //  bool success = opp_msgs_serialization::deserialize(area_selection_config_file, params);
  //  if (!success)
  //  {
  //    ROS_ERROR_STREAM("Could not load area selection config from: " << area_selection_config_file);
  //    return false;
  //  }
  AreaSelector sel;
  res->cloud_indices = sel.getRegionOfInterest(cloud, points, params);

  if (!res->cloud_indices.empty())
  {
    res->success = true;
    res->message = "Selection complete";
  }
  else
  {
    res->success = false;
    res->message = "Unable to identify points within selection boundary";
  }
}

bool SelectionArtist::transformPoint(const geometry_msgs::msg::PointStamped::ConstSharedPtr pt_stamped,
                                     geometry_msgs::msg::Point& transformed_pt)
{
  RCLCPP_INFO(node_->get_logger(), pt_stamped->header.frame_id);

  // Get the current transform from the world frame to the frame of the sensor data
  geometry_msgs::msg::TransformStamped frame;
  try
  {
    frame = tf_buffer_.lookupTransform(sensor_frame_, world_frame_, tf2::TimePointZero);
  }
  catch (tf2::TransformException ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    return false;
  }

  Eigen::Isometry3d transform = tf2::transformToEigen(frame);

  // Transform the current selection point
  Eigen::Vector3d pt_vec_sensor;
  Eigen::Vector3d pt_vec_ff(pt_stamped->point.x, pt_stamped->point.y, pt_stamped->point.z);
  pt_vec_sensor = transform * pt_vec_ff;

  transformed_pt.x = pt_vec_sensor(0);
  transformed_pt.y = pt_vec_sensor(1);
  transformed_pt.z = pt_vec_sensor(2);

  return true;
}

void SelectionArtist::addSelectionPoint(const geometry_msgs::msg::PointStamped::ConstSharedPtr pt_stamped)
{
  geometry_msgs::msg::Point pt;
  if (!transformPoint(pt_stamped, pt))
  {
    return;
  }

  // Get the iterator to the points and lines markers in the interactive marker
  std::vector<visualization_msgs::msg::Marker>::iterator points_it;
  points_it = std::find_if(marker_array_.markers.begin(),
                           marker_array_.markers.end(),
                           [](const visualization_msgs::msg::Marker& marker) { return marker.id == 0; });

  std::vector<visualization_msgs::msg::Marker>::iterator lines_it;
  lines_it = std::find_if(marker_array_.markers.begin(),
                          marker_array_.markers.end(),
                          [](const visualization_msgs::msg::Marker& marker) { return marker.id == 1; });

  // Add new point to the points marker
  if (points_it == marker_array_.markers.end() || lines_it == marker_array_.markers.end())
  {
    RCLCPP_ERROR(node_->get_logger(), "Unable to find line or point marker");
    return;
  }
  else
  {
    points_it->points.push_back(pt);

    // Add the point to the front and back of the lines' points array if it is the first entry
    // Lines connect adjacent points, so first point must be entered twice to close the polygon
    if (lines_it->points.empty())
    {
      lines_it->points.push_back(pt);
      lines_it->points.push_back(pt);
    }
    // Insert the new point in the second to last position if points already exist in the array
    else
    {
      const auto it = lines_it->points.end() - 1;
      lines_it->points.insert(it, pt);
    }
  }

  marker_pub_->publish(marker_array_);
}

void SelectionArtist::filterMesh(const pcl::PolygonMesh& input_mesh,
                                 const std::vector<int>& inlying_indices,
                                 pcl::PolygonMesh& output_mesh)
{
  // mark inlying points as true and outlying points as false
  std::vector<bool> whitelist(input_mesh.cloud.width * input_mesh.cloud.height, false);
  for (std::size_t i = 0; i < inlying_indices.size(); ++i)
  {
    whitelist[static_cast<std::size_t>(inlying_indices[i])] = true;
  }

  // All points marked 'false' in the whitelist will be removed. If
  // all of the polygon's points are marked 'true', that polygon
  // should be included.
  pcl::PolygonMesh intermediate_mesh;
  intermediate_mesh.cloud = input_mesh.cloud;
  for (std::size_t i = 0; i < input_mesh.polygons.size(); ++i)
  {
    if (whitelist[input_mesh.polygons[i].vertices[0]] && whitelist[input_mesh.polygons[i].vertices[1]] &&
        whitelist[input_mesh.polygons[i].vertices[2]])
    {
      intermediate_mesh.polygons.push_back(input_mesh.polygons[i]);
    }
  }

  // Remove unused points and save the result to the output mesh
  pcl::surface::SimplificationRemoveUnusedVertices simplifier;
  simplifier.simplify(intermediate_mesh, output_mesh);

  return;
}

}  // namespace crs_area_selection
