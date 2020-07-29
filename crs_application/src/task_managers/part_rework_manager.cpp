/*
 * @author ros-industrial
 * @file part_rework_manager.cpp
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

#include <crs_motion_planning/path_processing_utils.h>

#include "crs_application/task_managers/part_rework_manager.h"

static const double WAIT_FOR_MSG_TIMEOUT = 5.0;
static const double WAIT_ROBOT_STOP = 2.0;
static const double WAIT_MOTION_COMPLETION = 30.0;
static const double WAIT_FOR_SERVICE_PERIOD = 10.0;
static const double WAIT_SERVICE_COMPLETION_TIMEOUT = 10.0;

static const std::string FREESPACE_MOTION_PLAN_SERVICE = "plan_freespace_motion";
static const std::string CROP_TOOLPATH_SERVICE = "crop_toolpaths";
static const std::string MANAGER_NAME = "PartReworkManager";
static const std::string SCAN_POSES_TOPIC = "rework_scan_poses";
static const std::string CROPPED_TOOLPATH_MARKER_TOPIC = "cropped_toolpaths";
static const std::string DEFAULT_WORLD_FRAME_ID = "world";
static const std::string POINT_CLOUD_TOPIC = "camera/pointcloud";
static const std::string IMAGE_TOPIC = "camera/image";
static const std::string MARKER_NS_TOOLPATH = "toolpath";

namespace crs_application
{
namespace task_managers
{
PartReworkManager::PartReworkManager(std::shared_ptr<rclcpp::Node> node)
  : node_(node)
  , scan_poses_(std::vector<geometry_msgs::msg::Transform>())
  , process_data_(std::make_shared<crs_msgs::srv::CropToolpaths::Request>())
  , scan_index_(0)
  , config_(nullptr)
  , private_node_(std::make_shared<rclcpp::Node>(MANAGER_NAME + "_private"))
  , tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
  , tf_listener_(tf_buffer_)
{
  // spinning the node asynchronously
  pnode_executor_.add_node(private_node_);
  std::thread([this]() { pnode_executor_.spin(); }).detach();
}

PartReworkManager::~PartReworkManager() { pnode_executor_.cancel(); }
common::ActionResult PartReworkManager::init()
{
  // publishers
  scan_poses_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(SCAN_POSES_TOPIC, rclcpp::QoS(1));
  cropped_toolpath_markers_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(CROPPED_TOOLPATH_MARKER_TOPIC, rclcpp::QoS(1));

  // service clients
  call_freespace_motion_client_ =
      node_->create_client<crs_msgs::srv::CallFreespaceMotion>(FREESPACE_MOTION_PLAN_SERVICE);
  crop_toolpaths_client_ = node_->create_client<crs_msgs::srv::CropToolpaths>(CROP_TOOLPATH_SERVICE);

  // waiting for services
  common::ActionResult res;
  std::vector<rclcpp::ClientBase*> srv_clients = { call_freespace_motion_client_.get(), crop_toolpaths_client_.get() };
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
    return false;
  }
  return true;
}

common::ActionResult PartReworkManager::configure(const config::PartReworkConfig& config)
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

  RCLCPP_INFO(node_->get_logger(), "%s got %lu scan poses", MANAGER_NAME.c_str(), scan_poses_.size());

  // saving configuration
  config_ = std::make_shared<config::PartReworkConfig>(config);
  return true;
}

common::ActionResult PartReworkManager::moveRobot()
{
  // check service
  if (!call_freespace_motion_client_->service_is_ready())
  {
    RCLCPP_ERROR(node_->get_logger(), "%s Freespace Motion is not ready`", MANAGER_NAME.c_str());
    return false;
  }

  auto freespace_motion_request = std::make_shared<crs_msgs::srv::CallFreespaceMotion::Request>();
  freespace_motion_request->target_link = config_->tool_frame;
  freespace_motion_request->goal_pose = scan_poses_.at(scan_index_);
  freespace_motion_request->execute = true;
  auto result_future = call_freespace_motion_client_->async_send_request(freespace_motion_request);

  // advancing scan index
  scan_index_++;
  proceed_next_scan = true;

  // waiting for result
  std::future_status status = result_future.wait_for(std::chrono::duration<double>(WAIT_MOTION_COMPLETION));
  if (status != std::future_status::ready)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s Call Freespace Motion service call timed out", MANAGER_NAME.c_str());
    return false;
  }
  auto result = result_future.get();

  if (!result->success)
  {
    common::ActionResult res;
    if (config_->skip_on_failure)
    {
      proceed_next_scan = false;
      res = true;
      res.err_msg = boost::str(boost::format("%s failure during robot move, skipping next scan") % MANAGER_NAME);
      RCLCPP_WARN_STREAM(node_->get_logger(), res.err_msg);
    }
    else
    {
      res = false;
      res.err_msg = boost::str(boost::format("%s %s") % MANAGER_NAME % result->message);
      RCLCPP_ERROR_STREAM(node_->get_logger(), res.err_msg);
    }
    return res;
  }

  std::chrono::duration<double> sleep_dur(WAIT_ROBOT_STOP);
  RCLCPP_INFO(node_->get_logger(), "Waiting %f seconds for robot to fully stop", WAIT_ROBOT_STOP);
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::seconds>(sleep_dur));
  return true;
}

common::ActionResult PartReworkManager::trimToolpaths()
{
  using namespace crs_msgs;

  // check if input has been set
  if (!input_)
  {
    common::ActionResult res = false;
    res.err_msg = "Input trajectory for part rework has not been set";
    RCLCPP_INFO_STREAM(node_->get_logger(), res.err_msg);
    return res;
  }

  crs_msgs::msg::ToolProcessPath process_path;
  for (const geometry_msgs::msg::PoseArray& raster : input_->rasters)
  {
    process_path.rasters.push_back(raster);
  }
  process_data_->toolpaths.clear();
  process_data_->toolpaths.push_back(process_path);

  // clearing results from previous actions
  result_ = datatypes::ProcessToolpathData();

  // calling crop service
  std::shared_future<crs_msgs::srv::CropToolpaths::Response::SharedPtr> result_future =
      crop_toolpaths_client_->async_send_request(process_data_);

  common::ActionResult res;
  if (result_future.wait_for(std::chrono::duration<double>(WAIT_SERVICE_COMPLETION_TIMEOUT)) !=
      std::future_status::ready)
  {
    res.succeeded = false;
    res.err_msg = boost::str(boost::format("%s process planning service error or timeout") % MANAGER_NAME);
    RCLCPP_ERROR_STREAM(node_->get_logger(), res.err_msg);
    return res;
  }

  // saving results
  crs_msgs::srv::CropToolpaths::Response::SharedPtr srv_response = result_future.get();
  if (srv_response->succeeded)
  {
    res.succeeded = false;
    res.err_msg = boost::str(boost::format("%s crop service failed to complete request") % MANAGER_NAME);
    RCLCPP_ERROR_STREAM(node_->get_logger(), res.err_msg);
    return res;
  }
  result_.rasters = srv_response->cropped_toolpaths.front().rasters;
  RCLCPP_INFO(node_->get_logger(), "%s successfully cropped toolpaths", MANAGER_NAME.c_str());

  return true;
}

common::ActionResult PartReworkManager::showPreview()
{
  using namespace std::chrono_literals;

  visualization_msgs::msg::MarkerArray markers =
      crs_motion_planning::convertToDottedLineMarker(result_.rasters, DEFAULT_WORLD_FRAME_ID, MARKER_NS_TOOLPATH);

  // setting up timers
  if (preview_markers_publish_timer_)
  {
    preview_markers_publish_timer_->cancel();
  }
  preview_markers_publish_timer_ =
      node_->create_wall_timer(500ms, [this, markers]() -> void { cropped_toolpath_markers_pub_->publish(markers); });
  return true;
}

common::ActionResult PartReworkManager::hidePreview()
{
  using namespace visualization_msgs;
  using namespace std::chrono_literals;

  // canceling timers
  if (preview_markers_publish_timer_)
  {
    preview_markers_publish_timer_->cancel();
  }

  if (scan_poses_pub_timer_)
  {
    scan_poses_pub_timer_->cancel();
  }

  // wait for timers to cancel
  rclcpp::sleep_for(10ms);

  // create delete all marker
  msg::MarkerArray markers;
  msg::Marker m;
  m.action = m.DELETEALL;
  markers.markers.push_back(m);

  // publishing now
  cropped_toolpath_markers_pub_->publish(markers);
  return true;
}

common::ActionResult PartReworkManager::setInput(const datatypes::ProcessToolpathData& input)
{
  input_ = std::make_shared<datatypes::ProcessToolpathData>(input);
  return true;
}

common::ActionResult PartReworkManager::reset()
{
  scan_index_ = 0;
  process_data_ = std::make_shared<crs_msgs::srv::CropToolpaths::Request>();
  return true;
}

common::ActionResult PartReworkManager::acquireScan()
{
  if (!proceed_next_scan)
  {
    common::ActionResult res = true;
    res.err_msg =
        boost::str(boost::format("%s skipping scan since previous robot moved failed to complete") % MANAGER_NAME);
    RCLCPP_WARN_STREAM(node_->get_logger(), res.err_msg);
    return res;
  }

  // grabbing point cloud and image
  sensor_msgs::msg::PointCloud2::Ptr cloud_msg = common::waitForMessage<sensor_msgs::msg::PointCloud2>(
      private_node_, POINT_CLOUD_TOPIC, false, WAIT_FOR_MSG_TIMEOUT);
  sensor_msgs::msg::Image::Ptr image_msg =
      common::waitForMessage<sensor_msgs::msg::Image>(private_node_, IMAGE_TOPIC, false, WAIT_FOR_MSG_TIMEOUT);

  if (!cloud_msg || !image_msg)
  {
    common::ActionResult res = false;
    res.err_msg = boost::str(boost::format("%s failed to get valid scan images or point clouds") % MANAGER_NAME);
    RCLCPP_ERROR_STREAM(node_->get_logger(), res.err_msg);
    return res;
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Acquired scan data");
  process_data_->clouds.push_back(*cloud_msg);
  process_data_->images.push_back(*image_msg);

  geometry_msgs::msg::TransformStamped transform;
  std::string scan_frame_id = cloud_msg->header.frame_id;
  try
  {
    transform = tf_buffer_.lookupTransform(DEFAULT_WORLD_FRAME_ID, scan_frame_id, tf2::TimePointZero);
    process_data_->transforms.push_back(transform);
  }
  catch (tf2::TransformException& ex)
  {
    std::string error_msg =
        "Failed to get transform from '" + scan_frame_id + "' to '" + DEFAULT_WORLD_FRAME_ID + "' frame";

    RCLCPP_ERROR(node_->get_logger(), "Cloud Frame error: %s: ", ex.what(), error_msg.c_str());
    return common::ActionResult(false, error_msg);
  }
  return true;
}

common::ActionResult PartReworkManager::doneScanning()
{
  if (scan_index_ >= scan_poses_.size())
  {
    return true;
  }
  return false;
}

} /* namespace task_managers */
} /* namespace crs_application */
