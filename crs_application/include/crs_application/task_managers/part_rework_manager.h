/*
 * @author ros-industrial
 * @file part_rework_manager.h
 * @date Jan 16, 2020
 * @copyright Copyright (c) 2020, Southwest Research Institute
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef INCLUDE_CRS_APPLICATION_TASK_MANAGERS_PART_REWORK_MANAGER_H_
#define INCLUDE_CRS_APPLICATION_TASK_MANAGERS_PART_REWORK_MANAGER_H_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "crs_application/common/common.h"
#include "crs_application/common/datatypes.h"
#include "crs_application/common/config.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <visualization_msgs/msg/marker_array.hpp>

#include <crs_msgs/srv/call_freespace_motion.hpp>

#include <region_detection_msgs/srv/crop_data.hpp>
#include <region_detection_msgs/srv/detect_regions.hpp>
#include <region_detection_msgs/srv/get_selected_regions.hpp>
#include <region_detection_msgs/srv/show_selectable_regions.hpp>

namespace crs_application
{
namespace task_managers
{
class PartReworkManager
{
public:
  PartReworkManager(std::shared_ptr<rclcpp::Node> node);
  virtual ~PartReworkManager();

  // initialization and configuration
  common::ActionResult init();
  common::ActionResult configure(const config::PartReworkConfig& config);
  common::ActionResult setInput(const datatypes::ProcessToolpathData& input);

  // Process Actions
  common::ActionResult reset();
  common::ActionResult moveRobot();
  common::ActionResult acquireScan();
  common::ActionResult doneScanning();
  common::ActionResult detectRegions();
  common::ActionResult showRegions();
  common::ActionResult trimToolpaths();
  common::ActionResult showPreview();
  common::ActionResult hidePreview();

  // Results
  const std::vector<datatypes::ProcessToolpathData>& getResult() { return result_; }

protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> private_node_;
  rclcpp::executors::MultiThreadedExecutor pnode_executor_;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr scan_poses_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cropped_toolpath_markers_pub_;

  // service clients
  rclcpp::Client<crs_msgs::srv::CallFreespaceMotion>::SharedPtr call_freespace_motion_client_;
  rclcpp::Client<region_detection_msgs::srv::DetectRegions>::SharedPtr detect_regions_client_;
  rclcpp::Client<region_detection_msgs::srv::ShowSelectableRegions>::SharedPtr show_selectable_regions_client_;
  rclcpp::Client<region_detection_msgs::srv::GetSelectedRegions>::SharedPtr get_selected_regions_client_;
  rclcpp::Client<region_detection_msgs::srv::CropData>::SharedPtr crop_toolpaths_client_;

  // timers
  rclcpp::TimerBase::SharedPtr scan_poses_pub_timer_;
  rclcpp::TimerBase::SharedPtr preview_markers_publish_timer_;

  // config parameters
  std::shared_ptr<config::PartReworkConfig> config_;
  std::vector<geometry_msgs::msg::Transform> scan_poses_;

  // inputs and outputs
  std::shared_ptr<datatypes::ProcessToolpathData> input_ = nullptr;
  std::vector<datatypes::ProcessToolpathData> result_;

  // process data
  bool proceed_next_scan; /** @brief will be set to false if previous robot move failed*/
  region_detection_msgs::srv::DetectRegions::Request::Ptr scan_data_;
  region_detection_msgs::srv::DetectRegions::Response detected_regions_results_;
  int scan_index_;
};

} /* namespace task_managers */
} /* namespace crs_application */

#endif /* INCLUDE_CRS_APPLICATION_TASK_MANAGERS_PART_REWORK_MANAGER_H_ */
