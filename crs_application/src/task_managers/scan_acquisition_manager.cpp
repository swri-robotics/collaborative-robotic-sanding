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
#include <Eigen/Core>
#include "crs_application/common/common.h"
#include "crs_application/task_managers/scan_acquisition_manager.h"

static const double WAIT_FOR_SERVICE_PERIOD = 10.0;
static const double WAIT_MESSAGE_TIMEOUT = 2.0;
static const double WAIT_ROBOT_STOP = 2.0;
static const double WAIT_MOTION_COMPLETION = 30.0;
static const std::size_t POSES_ARRAY_SIZE = 6;
static const std::string POINT_CLOUD_TOPIC = "camera/pointcloud";
static const std::string FREESPACE_MOTION_PLAN_SERVICE = "plan_freespace_motion";
static const std::string MANAGER_NAME = "ScanAcquisitionManager";
static const std::string SCAN_POSES_TOPIC = "scan_poses";
static const std::string DEFAULT_WORLD_FRAME_ID = "world";

namespace crs_application
{
namespace task_managers
{
ScanAcquisitionManager::ScanAcquisitionManager(std::shared_ptr<rclcpp::Node> node)
  : node_(node)
  , scan_poses_(std::vector<geometry_msgs::msg::Transform>())
  , tool_frame_("")
  , max_time_since_last_point_cloud_(0.1)
  , scan_index_(0)
  , private_node_(std::make_shared<rclcpp::Node>(MANAGER_NAME + "_private"))
  , tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
  , tf_listener_(tf_buffer_)
{
}

ScanAcquisitionManager::~ScanAcquisitionManager() {}
common::ActionResult ScanAcquisitionManager::init()
{
  using namespace std::chrono_literals;

  // parameters
  tool_frame_ = node_->declare_parameter("camera_frame_id", "eoat_link");
  max_time_since_last_point_cloud_ = node_->declare_parameter("max_time_since_last_point_cloud", 0.1);
  pre_acquisition_pause_ = node_->declare_parameter("pre_acquisition_pause", 1.0);

  // subscribers
  point_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      POINT_CLOUD_TOPIC, 1, std::bind(&ScanAcquisitionManager::handlePointCloud, this, std::placeholders::_1));

  // publishers
  scan_poses_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(SCAN_POSES_TOPIC, rclcpp::QoS(1));

  // service client
  call_freespace_motion_client_ =
      node_->create_client<crs_msgs::srv::CallFreespaceMotion>(FREESPACE_MOTION_PLAN_SERVICE);

  // waiting for services
  common::ActionResult res;
  std::vector<rclcpp::ClientBase*> srv_clients = { call_freespace_motion_client_.get() };
  RCLCPP_INFO(node_->get_logger(), "%s waiting for services", MANAGER_NAME.c_str());
  if (!std::all_of(srv_clients.begin(), srv_clients.end(), [this, &res](rclcpp::ClientBase* c) {
        if (!c->wait_for_service(std::chrono::duration<double>(WAIT_FOR_SERVICE_PERIOD)))
        {
          res.succeeded = false;
          res.err_msg = boost::str(boost::format("service '%s' was not found") % c->get_service_name());
          return false;
        }
        return true;
      }))
  {
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
  }

  return true;
}

common::ActionResult ScanAcquisitionManager::configure(const config::ScanAcquisitionConfig& config)
{
  using namespace std::chrono_literals;
  using namespace Eigen;

  common::ActionResult res;
  if (config.scan_poses.empty())
  {
    res.err_msg = "no scan poses were found in configuration";
    res.succeeded = false;
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }

  scan_poses_.clear();
  for (std::size_t i = 0; i < config.scan_poses.size(); i++)
  {
    std::array<double, 6> tvals;
    std::copy(config.scan_poses[i].begin(), config.scan_poses[i].end(), tvals.begin());
    geometry_msgs::msg::Transform tf = common::toTransformMsg(tvals);
    scan_poses_.push_back(tf);
  }

  // publish scan poses
  scan_poses_pub_timer_ = node_->create_wall_timer(10ms, [this]() -> void {
    geometry_msgs::msg::PoseArray poses;
    poses.header.frame_id = DEFAULT_WORLD_FRAME_ID;
    for (std::size_t i = 0; i < scan_poses_.size(); i++)
    {
      geometry_msgs::msg::Pose p;
      p.position.x = scan_poses_[i].translation.x;
      p.position.y = scan_poses_[i].translation.y;
      p.position.z = scan_poses_[i].translation.z;
      p.orientation = scan_poses_[i].rotation;
      poses.poses.push_back(p);
    }
    scan_poses_pub_->publish(poses);
  });

  tool_frame_ = config.tool_frame;

  RCLCPP_INFO(node_->get_logger(), "%s got %lu scan poses", MANAGER_NAME.c_str(), scan_poses_.size());
  return true;
}

common::ActionResult ScanAcquisitionManager::verify()
{
  common::ActionResult res = checkPreReqs();
  if (!res)
  {
    return res;
  }

  // resetting variables
  scan_index_ = 0;
  current_data_.point_clouds.clear();
  current_data_.transforms.clear();
  return true;
}

common::ActionResult ScanAcquisitionManager::moveRobot()
{
  // check service
  if (!call_freespace_motion_client_->service_is_ready())
  {
    RCLCPP_ERROR(node_->get_logger(), "%s Freespace Motion is not ready`", MANAGER_NAME.c_str());
    return false;
  }

  auto freespace_motion_request = std::make_shared<crs_msgs::srv::CallFreespaceMotion::Request>();
  freespace_motion_request->target_link = tool_frame_;
  freespace_motion_request->goal_pose = scan_poses_.at(scan_index_);
  freespace_motion_request->execute = true;

  auto result_future = call_freespace_motion_client_->async_send_request(freespace_motion_request);

  std::future_status status = result_future.wait_for(std::chrono::duration<double>(WAIT_MOTION_COMPLETION));
  if (status != std::future_status::ready)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s Call Freespace Motion service call timedout", MANAGER_NAME.c_str());
    return false;
  }
  auto result = result_future.get();

  if (result->success)
  {
    // todo(ayoungs): wait for robot to finish moving, for now
    std::chrono::duration<double> sleep_dur(WAIT_ROBOT_STOP);
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::seconds>(sleep_dur));
    return true;
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), result->message.c_str());
    return false;
  }
}

common::ActionResult ScanAcquisitionManager::capture()
{
  // sleeping first
  // TODO(ayoungs): transform point cloud

  // TODO asses if the logic below is still needed
  if (node_->now() - curr_point_cloud_.header.stamp >= rclcpp::Duration(max_time_since_last_point_cloud_))
  {
    auto captured_cloud = curr_point_cloud_;
    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform =
          tf_buffer_.lookupTransform(DEFAULT_WORLD_FRAME_ID, captured_cloud.header.frame_id, tf2::TimePointZero);
    }
    catch (tf2::TransformException ex)
    {
      std::string error_msg = "Failed to get transform from '" + captured_cloud.header.frame_id + "' to '" +
                              DEFAULT_WORLD_FRAME_ID + "' frame";

      RCLCPP_ERROR(node_->get_logger(), "Cloud Frame error: %s: ", ex.what(), error_msg.c_str());
      return false;
    }
    current_data_.point_clouds.push_back(captured_cloud);
    current_data_.transforms.push_back(transform);
    return true;
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get scan");
    return false;
  }
}

common::ActionResult ScanAcquisitionManager::checkQueue()
{
  scan_index_++;
  if (scan_index_ < scan_poses_.size())
  {
    return false;
  }
  else
  {
    // save off results and reset the point clouds
    result_ = current_data_;
    scan_index_ = 0;
    current_data_.point_clouds.clear();
    current_data_.transforms.clear();
    return true;
  }
}

common::ActionResult ScanAcquisitionManager::checkPreReqs()
{
  common::ActionResult res;

  if (scan_poses_.empty())
  {
    res.succeeded = false;
    res.err_msg = "No scan positions available, cannot proceed";
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }

  if (tool_frame_.empty())
  {
    res.succeeded = false;
    res.err_msg = "No camera frame has been specified, cannot proceed";
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }
  return true;
}

void ScanAcquisitionManager::handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  curr_point_cloud_ = *msg;
}

}  // end of namespace task_managers
}  // end of namespace crs_application
