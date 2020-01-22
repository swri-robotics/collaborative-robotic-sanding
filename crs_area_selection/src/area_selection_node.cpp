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

#include <rclcpp/rclcpp.hpp>

#include "crs_area_selection/area_selector.h"
#include "crs_area_selection/selection_artist.h"

int main(int argc, char** argv)
{
  // Set up ROS.
  rclcpp::init(argc, argv);
//  rclcpp::sleep_for(std::chrono::seconds(3));
  auto node = std::make_shared<rclcpp::Node>("roi_selection_node");

  std::string world_frame = "world";
  std::string sensor_data_frame = "sensor_data_frame";
  RCLCPP_WARN(node->get_logger(), "World frame and sensor_data_frame are not being set from parameters");

//  // Get ROS parameters
//  std::string world_frame;
//  if (!pnh.getParam("world_frame", world_frame))
//  {
//    ROS_FATAL("'world_frame' parameter must be set");
//    return 1;
//  }

//  std::string sensor_data_frame;
//  if (!pnh.getParam("sensor_data_frame", sensor_data_frame))
//  {
//    ROS_FATAL("'sensor_data_frame' parameter must be set");
//    return 1;
//  }

  // Set up the selection artist
  crs_area_selection::SelectionArtist artist(node, world_frame, sensor_data_frame);

  // This used an aysnc spinner for some reason.
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
