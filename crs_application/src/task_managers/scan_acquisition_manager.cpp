/*
 * @author Jorge Nicho
 * @file scan_acquisition_manager.cpp
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

#include "crs_application/task_managers/scan_acquisition_manager.h"

namespace crs_application
{
namespace task_managers
{

ScanAcquisitionManager::ScanAcquisitionManager(std::shared_ptr<rclcpp::Node> node):
    node_(node),
    scan_positions_(std::vector<geometry_msgs::msg::Transform>()),
    framos_frame_id_(""),
    max_time_since_last_point_cloud_(0.1),
    point_clouds_(std::vector<sensor_msgs::msg::PointCloud2>()),
    scan_index_(0)
{

}

ScanAcquisitionManager::~ScanAcquisitionManager()
{

}

common::ActionResult ScanAcquisitionManager::init()
{
  // parameters
  //scan_positions_ = node_->declare_parameter("scan_positions", std::vector<geometry_msgs::msg::Transform>());
  framos_frame_id_ = node_->declare_parameter("framos_frame_id", "eoat_link");
  max_time_since_last_point_cloud_ = node_->declare_parameter("max_time_since_last_point_cloud", 0.1);

  // subscribers
  point_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>("/point_cloud", 1,
    std::bind(&ScanAcquisitionManager::handlePointCloud, this, std::placeholders::_1));

  // service client
  // todo(ayoungs): wait on service?
  call_freespace_motion_client_ = node_->create_client<crs_msgs::srv::CallFreespaceMotion>("test_plan");

  return true;
}

common::ActionResult ScanAcquisitionManager::configure(const ScanAcquisitionConfig& config)
{
  if (scan_positions_.size() == 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "No scan positions provided.");
    return false;
  }
  else
  {
    return true;
  }
}

common::ActionResult ScanAcquisitionManager::verify()
{
  RCLCPP_WARN(node_->get_logger(),"%s not implemented yet",__PRETTY_FUNCTION__);
  return true;
}

common::ActionResult ScanAcquisitionManager::moveRobot()
{
  auto freespace_motion_request = std::make_shared<crs_msgs::srv::CallFreespaceMotion::Request>();
  freespace_motion_request->target_link = framos_frame_id_;
  freespace_motion_request->goal_pose = scan_positions_.at(scan_index_);
  freespace_motion_request->execute = true;

  auto result_future = call_freespace_motion_client_->async_send_request(freespace_motion_request);
  if (rclcpp::spin_until_future_complete(node_, result_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Call Freespace Motion service call failed");
    // todo(ayoungs): do I need to reset the scan_index_ here and clear the point clouds?
    return false;
  }
  auto result = result_future.get();

  if (result->success)
  {
    scan_index_++;
    return true;
  }
  else
  {
    // todo(ayoungs): do I need to reset the scan_index_ here and clear the point clouds?
    RCLCPP_ERROR(node_->get_logger(), result->message);
    return false;
  }
}

common::ActionResult ScanAcquisitionManager::capture()
{
  // todo(ayoungs): transform
  // todo(ayoungs): is there a wait for topic call similar to ROS1?
  if (node_->now() - curr_point_cloud_.header.stamp <= rclcpp::Duration(max_time_since_last_point_cloud_))
  {
    point_clouds_.push_back(curr_point_cloud_);
    return true;
  }
  else
  {
    // todo(ayoungs): do I need to reset the scan_index_ here and clear the point clouds?
    return false;
  }
}

common::ActionResult ScanAcquisitionManager::checkQueue()
{
  if (scan_index_ < scan_positions_.size())
  {
    return false;
  }
  else
  {
    // save off results and reset the point clouds
    result_.point_clouds = point_clouds_;
    scan_index_ = 0;
    point_clouds_.clear();
    return true;
  }
}

void ScanAcquisitionManager::handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  curr_point_cloud_ = *msg;
}

} // end of namespace task_managers
} // end of namespace crs_application

