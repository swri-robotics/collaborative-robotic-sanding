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

#ifndef CRS_AREA_SELECTION_SELECTION_AREA_ARTIST_H
#define CRS_AREA_SELECTION_SELECTION_AREA_ARTIST_H

#include <pcl/PolygonMesh.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <crs_msgs/srv/get_roi_selection.hpp>

namespace crs_area_selection
{
/**
 * @brief The SelectionArtist class uses the Publish Points plugin in Rviz to allow the user to drop points on a mesh or
 * geometry primitive, creating a closed polygon region-of-interest (ROI). The ROI is displayed using an Interactive
 * Marker, which enables the user to reset the ROI or display the points within the selection polygon by means of a
 * drop-down menu. This class also contains a ROS service server to output the last selection points.
 */
class SelectionArtist
{
public:
  /**
   * @brief SelectionArtist is the class constructor which initializes ROS communication objects and private
   * variables.The 'world_frame' argument is the highest-level fixed frame (i.e. "map", "odom", or "world"). The
   * "sensor_frame" argument is the aggregated data frame (typically the base frame of the kinematic chain, i.e
   * rail_base_link or robot_base_link)
   * @param nh
   * @param world_frame
   * @param sensor_frame
   */
  SelectionArtist(rclcpp::Node::SharedPtr node, const std::string& world_frame, const std::string& sensor_frame);

  void clearROIPointsCb(const std_srvs::srv::Trigger::Request::SharedPtr req,
                        std_srvs::srv::Trigger::Response::SharedPtr res);

  bool collectROIMesh(const shape_msgs::msg::Mesh& mesh_msg, shape_msgs::msg::Mesh& submesh_msg, std::string& message);

protected:
  /** @brief Unimplemented */
  void getSensorData(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  void addSelectionPoint(const geometry_msgs::msg::PointStamped::ConstSharedPtr pt_stamped);

  bool transformPoint(const geometry_msgs::msg::PointStamped::ConstSharedPtr pt_stamped,
                      geometry_msgs::msg::Point& transformed_pt);

  void collectROIPointsCb(crs_msgs::srv::GetROISelection::Request::SharedPtr req,
                          crs_msgs::srv::GetROISelection::Response::SharedPtr res);

  void filterMesh(const pcl::PolygonMesh& input_mesh,
                  const std::vector<int>& inlying_indices,
                  pcl::PolygonMesh& output_mesh);

  rclcpp::Node::SharedPtr node_;

  std::string world_frame_;

  std::string sensor_frame_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr drawn_points_sub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_roi_points_srv_;

  rclcpp::Service<crs_msgs::srv::GetROISelection>::SharedPtr collect_roi_points_srv_;

  rclcpp::Clock::SharedPtr clock_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  visualization_msgs::msg::MarkerArray marker_array_;
};

}  // namespace crs_area_selection

#endif  // OPP_AREA_SELECTION_SELECTION_AREA_ARTIST_H
