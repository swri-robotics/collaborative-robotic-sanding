/*
 * @author ros-industrial
 * @file part_registration.cpp
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

#include "crs_application/task_managers/part_registration_manager.h"

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace crs_application
{
namespace task_managers
{
PartRegistrationManager::PartRegistrationManager(std::shared_ptr<rclcpp::Node> node) : node_(node) {}

PartRegistrationManager::~PartRegistrationManager() {}

common::ActionResult PartRegistrationManager::init()
{
  // todo(ayoungs): wait on service?
  load_part_client_ = node_->create_client<crs_msgs::srv::LoadPart>("/crs/load_part");
  localize_to_part_client_ = node_->create_client<crs_msgs::srv::LocalizeToPart>("/crs/localize_to_part");

  return true;
}

common::ActionResult PartRegistrationManager::configure(const PartRegistrationConfig& config)
{
  auto load_part_request = std::make_shared<crs_msgs::srv::LoadPart::Request>();
  load_part_request->path_to_part = "/home/ayoungs/workspaces/crs/src/collaborative-robotic-sanding/crs_support/meshes/Parts/visual/part1_ch.stl";

  auto result_future = load_part_client_->async_send_request(load_part_request);
    RCLCPP_ERROR(node_->get_logger(), "Load Part test");
  if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Load Part service call failed");
    return false;
  }
  auto result = result_future.get();

  if (!result->success)
  {
    RCLCPP_ERROR(node_->get_logger(), "Load Part service call failed");
    return false;
  }

    RCLCPP_ERROR(node_->get_logger(), "Load Part success");
  return true;
}

common::ActionResult PartRegistrationManager::showPreview()
{
  RCLCPP_WARN(node_->get_logger(), "%s not implemented yet", __PRETTY_FUNCTION__);
  return true;
}

common::ActionResult PartRegistrationManager::setInput(const datatypes::ScanAcquisitionResult& input)
{
  input_ = std::make_shared<datatypes::ScanAcquisitionResult>(input);
  return true;
}

common::ActionResult PartRegistrationManager::computeTransform()
{
  RCLCPP_INFO(node_->get_logger(), "Test 1");
  // todo(ayoungs) get vector of point clouds from scan_acquisition_manager
  auto localize_to_part_request = std::make_shared<crs_msgs::srv::LocalizeToPart::Request>();
  RCLCPP_INFO(node_->get_logger(), "Test 2");
  localize_to_part_request->point_clouds = std::vector<sensor_msgs::msg::PointCloud2>();
  RCLCPP_INFO(node_->get_logger(), "Test 3");

  auto localize_result_future = localize_to_part_client_->async_send_request(localize_to_part_request);
  RCLCPP_INFO(node_->get_logger(), "Test 4");
  if (rclcpp::spin_until_future_complete(node_, localize_result_future) != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node_->get_logger(), "Test 5");
    RCLCPP_ERROR(node_->get_logger(), "Localize to Part service call failed");
    return false;
  }
  auto localize_result = localize_result_future.get();
  // todo(ayoungs) save off transform
    RCLCPP_INFO(node_->get_logger(), "Test 6");
  return true;
}

common::ActionResult PartRegistrationManager::applyTransform()
{
  RCLCPP_WARN(node_->get_logger(), "%s not implemented yet", __PRETTY_FUNCTION__);
  return true;
}

} /* namespace task_managers */
} /* namespace crs_application */
