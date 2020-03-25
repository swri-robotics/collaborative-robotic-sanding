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

static const double WAIT_FOR_SERVICE_PERIOD = 10.0;
static const std::string MANAGER_NAME = "PartRegistrationManager";

namespace crs_application
{
namespace task_managers
{
PartRegistrationManager::PartRegistrationManager(std::shared_ptr<rclcpp::Node> node)
: node_(node)
, pnode_(std::make_shared<rclcpp::Node>(std::string(node_->get_name()) + "_exec"))
{
}

PartRegistrationManager::~PartRegistrationManager() {}

common::ActionResult PartRegistrationManager::init()
{
  common::ActionResult res;

  load_part_client_ = pnode_->create_client<crs_msgs::srv::LoadPart>("/load_part");
  localize_to_part_client_ = pnode_->create_client<crs_msgs::srv::LocalizeToPart>("/localize_to_part");

  // waiting for services
  std::vector<rclcpp::ClientBase*> srv_clients = { load_part_client_.get(), localize_to_part_client_.get() };
  if (!std::all_of(srv_clients.begin(), srv_clients.end(), [this, &res](rclcpp::ClientBase* c) {
        if (!c->wait_for_service(std::chrono::duration<double>(WAIT_FOR_SERVICE_PERIOD)))
        {
          res.succeeded = false;
          res.err_msg = boost::str(boost::format("service '%s' was not found") % c->get_service_name());
          return res;
        }
        res.succeeded = true;
        return res;
      }))
  {
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg);
  }

  res.succeeded = true;
  return res;
}

common::ActionResult PartRegistrationManager::configure(const PartRegistrationConfig& config)
{
  common::ActionResult res;
  auto load_part_request = std::make_shared<crs_msgs::srv::LoadPart::Request>();

  // todo(ayoungs): once there is a user config, this should come from the config
  load_part_request->path_to_part = "/home/ayoungs/workspaces/crs/src/collaborative-robotic-sanding/crs_support/meshes/Parts/visual/part1_ch.stl";

  auto result_future = load_part_client_->async_send_request(load_part_request);
  if (rclcpp::spin_until_future_complete(pnode_, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Load Part service call failed");
    res.succeeded = false;
    res.err_msg = "Load Part service call failed.";
    return false;
  }
  auto result = result_future.get();

  if (!result->success)
  {
    RCLCPP_ERROR(node_->get_logger(), "Load Part service call failed");
    res.succeeded = false;
    res.err_msg = "Load Part service call failed.";
    return res;
  }

  RCLCPP_INFO(node_->get_logger(), "Load part succeeded");
  res.succeeded = true;
  return res;
}

common::ActionResult PartRegistrationManager::showPreview()
{
  RCLCPP_WARN(node_->get_logger(), "%s not implemented yet", __PRETTY_FUNCTION__);
  return true;
}

common::ActionResult PartRegistrationManager::setInput(const datatypes::ScanAcquisitionResult& input)
{
  input_ = std::make_shared<datatypes::ScanAcquisitionResult>(input);
  RCLCPP_ERROR(node_->get_logger(), "%d point clouds", input_->point_clouds.size());
  return true;
}

common::ActionResult PartRegistrationManager::computeTransform()
{
  common::ActionResult res;
  //todo(ayoungs): delete this after user configuration for loading parts works
  PartRegistrationConfig config;
  configure(config);

  auto localize_to_part_request = std::make_shared<crs_msgs::srv::LocalizeToPart::Request>();
  localize_to_part_request->point_clouds = input_->point_clouds;

  auto localize_result_future = localize_to_part_client_->async_send_request(localize_to_part_request);
  if (rclcpp::spin_until_future_complete(pnode_, localize_result_future) != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Localize to Part service call failed");
    return false;
  }
  auto localize_result = localize_result_future.get();

  if (localize_result->success == false)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to localize part: %s", localize_result->error.c_str());
    res.succeeded = false;
    res.err_msg = "Failed to localize part.";
    return res;
  }
  // todo(ayoungs) save off transform
  res.succeeded = true;
  return res;
}

common::ActionResult PartRegistrationManager::applyTransform()
{
  RCLCPP_WARN(node_->get_logger(), "%s not implemented yet", __PRETTY_FUNCTION__);
  return true;
}

} /* namespace task_managers */
} /* namespace crs_application */
