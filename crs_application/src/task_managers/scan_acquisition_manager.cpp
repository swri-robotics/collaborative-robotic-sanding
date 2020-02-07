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

#include <boost/format.hpp>
#include "crs_application/task_managers/scan_acquisition_manager.h"

static const double WAIT_FOR_SERVICE_PERIOD = 10.0;
static const double WAIT_MESSAGE_TIMEOUT = 2.0;
static const std::string POINT_CLOUD_TOPIC = "point_cloud";
static const std::string FREESPACE_MOTION_PLAN_SERVICE = "test_plan";
static const std::string MANAGER_NAME = "ScanAcquisitionManager";

namespace crs_application
{
namespace task_managers
{
ScanAcquisitionManager::ScanAcquisitionManager(std::shared_ptr<rclcpp::Node> node) : node_(node) {}

ScanAcquisitionManager::ScanAcquisitionManager(std::shared_ptr<rclcpp::Node> node):
    node_(node),
    scan_positions_(std::vector<geometry_msgs::msg::Transform>()),
    camera_frame_id_(""),
    max_time_since_last_point_cloud_(0.1),
    point_clouds_(std::vector<sensor_msgs::msg::PointCloud2>()),
    scan_index_(0)
{

common::ActionResult ScanAcquisitionManager::init()
{
  // parameters
  camera_frame_id_ = node_->declare_parameter("camera_frame_id", "eoat_link");
  max_time_since_last_point_cloud_ = node_->declare_parameter("max_time_since_last_point_cloud", 0.1);
  pre_acquisition_pause_ = node_->declare_parameter("pre_acquisition_pause", 1.0);

  // subscribers
  point_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(POINT_CLOUD_TOPIC, 1,
    std::bind(&ScanAcquisitionManager::handlePointCloud, this, std::placeholders::_1));

  // service client
  call_freespace_motion_client_ = node_->create_client<crs_msgs::srv::CallFreespaceMotion>(
      FREESPACE_MOTION_PLAN_SERVICE);

  // waiting for services
  common::ActionResult res;
  std::vector<rclcpp::ClientBase*> srv_clients = {call_freespace_motion_client_.get()};
  if(!std::all_of(srv_clients.begin(), srv_clients.end(),[this, &res](rclcpp::ClientBase* c){
    if(!c->wait_for_service(std::chrono::duration<double>(WAIT_FOR_SERVICE_PERIOD)))
    {
      res.succeeded = false;
      res.err_msg = boost::str(boost::format("service '%s' was not found") % c->get_service_name());
      return false;
    }
    return true;
  }))
  {
    RCLCPP_ERROR(node_->get_logger(),"%s %s",MANAGER_NAME.c_str(),res.err_msg);
  }

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
  common::ActionResult res = checkPreReqs();
  if(!res)
  {
    return res;
  }

  // resetting variables
  scan_index_ = 0;
  point_clouds_.clear();
  return true;
}

common::ActionResult ScanAcquisitionManager::moveRobot()
{
  auto freespace_motion_request = std::make_shared<crs_msgs::srv::CallFreespaceMotion::Request>();
  freespace_motion_request->target_link = camera_frame_id_;
  freespace_motion_request->goal_pose = scan_positions_.at(scan_index_);
  freespace_motion_request->execute = true;

  auto result_future = call_freespace_motion_client_->async_send_request(freespace_motion_request);
  if (rclcpp::spin_until_future_complete(node_, result_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s Call Freespace Motion service call failed",MANAGER_NAME.c_str());
    return false;
  }
  auto result = result_future.get();

  if (result->success)
  {
    return true;
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "%s %s",MANAGER_NAME.c_str(), result->message.c_str());
    return false;
  }
}

common::ActionResult ScanAcquisitionManager::capture()
{
  // sleeping first
  std::chrono::duration<double> sleep_dur(WAIT_MESSAGE_TIMEOUT);
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(sleep_dur));

  auto msg = common::waitForMessage<sensor_msgs::msg::PointCloud2>(node_,POINT_CLOUD_TOPIC,
                                                                   WAIT_MESSAGE_TIMEOUT);
  if(!msg)
  {
    common::ActionResult res;
    res.succeeded =false;
    res.err_msg = "Failed to get point cloud message";
    return res;
  }
  curr_point_cloud_ = *msg;

  // TODO(ayoungs): transform point cloud

  // TODO asses if the logic below is still needed
  if (node_->now() - curr_point_cloud_.header.stamp <= rclcpp::Duration(max_time_since_last_point_cloud_))
  {
    point_clouds_.push_back(curr_point_cloud_);
    return true;
  }
  else
  {
    return false;
  }
}

common::ActionResult ScanAcquisitionManager::checkQueue()
{
  scan_index_++;
  if (scan_index_ < scan_positions_.size()-1)
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

common::ActionResult ScanAcquisitionManager::checkPreReqs()
{
  common::ActionResult res;
  if(scan_positions_.empty())
  {
    res.succeeded =false;
    res.err_msg = "No scan positions available, cannot proceed";
    RCLCPP_ERROR(node_->get_logger(),"%s %s",MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }

  if(camera_frame_id_.empty())
  {
    res.succeeded =false;
    res.err_msg = "No camera frame has been specified, cannot proceed";
    RCLCPP_ERROR(node_->get_logger(),"%s %s",MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }
  return true;
}

void ScanAcquisitionManager::handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  curr_point_cloud_ = *msg;
}

} // end of namespace task_managers
} // end of namespace crs_application
