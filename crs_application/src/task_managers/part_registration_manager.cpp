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

#include <boost/format.hpp>

#include "crs_application/task_managers/part_registration_manager.h"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_eigen/tf2_eigen.h>
#include <crs_motion_planning/path_processing_utils.h>

static const double WAIT_SERVICE_DURATION = 50.0;            // secs
static const double WAIT_SERVICE_COMPLETION_TIMEOUT = 2.0;  // secs

static const std::string PREVIEW_TOPIC = "part_registration_preview";
static const std::string LOAD_PART_SERVICE = "load_part";
static const std::string LOAD_PART_TESSERACT_SERVICE = "load_part_tesseract_env";
static const std::string REMOVE_PART_SERVICE = "remove_part_tesseract_env";
static const std::string LOCALIZE_TO_PART_SERVICE = "localize_to_part";

static const std::string MANAGER_NAME = "PartRegistrationManager";
static const std::string MARKER_NS_PART = "part";
static const std::string MARKER_NS_TOOLPATH = "toolpath";
static const std::string PART_FRAME_ID = "part";

namespace crs_application
{
namespace task_managers
{
PartRegistrationManager::PartRegistrationManager(std::shared_ptr<rclcpp::Node> node)
  : node_(node), tf_broadcaster_(node)
{
}

PartRegistrationManager::~PartRegistrationManager() {}
common::ActionResult PartRegistrationManager::init()
{
  // setting up publishers
  preview_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(PREVIEW_TOPIC, rclcpp::QoS(1));

  // setting up service clients
  load_part_client_ = node_->create_client<crs_msgs::srv::LoadPart>(LOAD_PART_SERVICE);
  load_part_tesseract_client_ = node_->create_client<crs_msgs::srv::LoadPart>(LOAD_PART_TESSERACT_SERVICE);
  remove_part_tesseract_client_ = node_->create_client<std_srvs::srv::Trigger>(REMOVE_PART_SERVICE);
  localize_to_part_client_ = node_->create_client<crs_msgs::srv::LocalizeToPart>(LOCALIZE_TO_PART_SERVICE);

  // object spawner
  obj_spawner_ = std::make_shared<common::SimulationObjectSpawner>(node_);

  // wait on service
  using BoolClientT = std::pair<bool, rclcpp::ClientBase*>;
  std::vector<BoolClientT> srv_clients = { std::make_pair(true, load_part_client_.get()),
                                           std::make_pair(true, localize_to_part_client_.get()) };
  if (!std::all_of(srv_clients.begin(), srv_clients.end(), [this](BoolClientT& bc) {
        bool required = bc.first;
        auto* c = bc.second;
        if (c->wait_for_service(std::chrono::duration<float>(WAIT_SERVICE_DURATION)))
        {
          return true;
        }

        if (required)
        {
          RCLCPP_ERROR(node_->get_logger(), "Failed to find required service %s", c->get_service_name());
          return false;
        }
        else
        {
          RCLCPP_WARN(node_->get_logger(), "Failed to find optional service %s", c->get_service_name());
        }
        return true;
      }))
  {
    RCLCPP_WARN(node_->get_logger(), "%s: One or more required services were not found", MANAGER_NAME.c_str());
    return false;
  }

  return true;
}

common::ActionResult PartRegistrationManager::configure(const config::PartRegistrationConfig& config)
{
  // saving config
  config_ = std::make_shared<config::PartRegistrationConfig>(config);

  // setting the transform
  std::array<double, 6> tvals;
  std::copy(config_->seed_pose.begin(), config_->seed_pose.end(), tvals.begin());
  part_transform_.transform = common::toTransformMsg(tvals);
  part_transform_.child_frame_id = PART_FRAME_ID;
  part_transform_.header.frame_id = config_->target_frame_id;

  // optionally load part into simulator if its running
  std::copy(config_->simulation_pose.begin(), config_->simulation_pose.end(), tvals.begin());
  obj_spawner_->remove(PART_FRAME_ID);
  obj_spawner_->spawn(PART_FRAME_ID, config_->target_frame_id, config_->part_file, tvals);

  // preparing the request
  auto load_part_request = std::make_shared<crs_msgs::srv::LoadPart::Request>();
  load_part_request->path_to_part = config_->part_file;
  load_part_request->part_origin = tf2::toMsg(tf2::transformToEigen(part_transform_.transform));

  // calling load part service
  auto result_future = load_part_client_->async_send_request(load_part_request);
  std::chrono::nanoseconds dur_timeout =
      rclcpp::Duration::from_seconds(WAIT_SERVICE_COMPLETION_TIMEOUT).to_chrono<std::chrono::nanoseconds>();
  std::future_status status = result_future.wait_for(dur_timeout);
  if (status != std::future_status::ready)
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

  applyTransform();
  showPreview();

  RCLCPP_INFO(node_->get_logger(), "Load part succeeded");
  return true;
}

common::ActionResult PartRegistrationManager::hidePreview()
{
  using namespace visualization_msgs;
  msg::MarkerArray markers;
  msg::Marker m;
  m.action = m.DELETEALL;
  markers.markers.push_back(m);

  // publishing now
  preview_markers_pub_->publish(markers);
  return true;
}

common::ActionResult PartRegistrationManager::showPreview()
{
  using namespace std::chrono_literals;
  using namespace visualization_msgs;

  common::ActionResult res;
  if (result_.rasters.empty())
  {
    res.err_msg = "No rasters to preview";
    res.succeeded = false;
    RCLCPP_ERROR_STREAM(node_->get_logger(), MANAGER_NAME << ": " << res.err_msg);
    return res;
  }

  hidePreview();

  // creating markers
  msg::Marker part_marker =
      crs_motion_planning::meshToMarker(config_->part_file, MARKER_NS_PART, config_->target_frame_id);
  part_marker.pose = tf2::toMsg(tf2::transformToEigen(part_transform_.transform));

  msg::MarkerArray markers =
      crs_motion_planning::convertToDottedLineMarker(result_.rasters, config_->target_frame_id, MARKER_NS_TOOLPATH);
  markers.markers.push_back(part_marker);

  // publishing now
  preview_markers_pub_->publish(markers);

  // setting up timers
  if (publish_timer_)
  {
    publish_timer_->cancel();
  }

  publish_timer_ = node_->create_wall_timer(500ms, [this, markers]() -> void {
    tf_broadcaster_.sendTransform(part_transform_);
    preview_markers_pub_->publish(markers);
  });

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
  auto remove_part_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto remove_result_future = remove_part_tesseract_client_->async_send_request(remove_part_request);

  common::ActionResult res;
  if (!config_)
  {
    res.err_msg = "configuration has not been set";
    res.succeeded = false;
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }

  auto localize_to_part_request = std::make_shared<crs_msgs::srv::LocalizeToPart::Request>();
  localize_to_part_request->point_clouds = input_->point_clouds;
  localize_to_part_request->transforms = input_->transforms;
  localize_to_part_request->frame = config_->target_frame_id;

  auto localize_result_future = localize_to_part_client_->async_send_request(localize_to_part_request);
  std::future_status status = localize_result_future.wait_for(std::chrono::seconds(30));
  if (status != std::future_status::ready)
  {
    RCLCPP_ERROR(node_->get_logger(), "Localize to Part service call failed");
    return false;
  }
  auto localize_result = localize_result_future.get();

  if (localize_result->success == false)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to localize part: %s", localize_result->error.c_str());
    return false;
  }
  part_transform_ = localize_result->transform;
  part_transform_.child_frame_id = PART_FRAME_ID;
  part_transform_.header.frame_id = config_->target_frame_id;
  RCLCPP_INFO_STREAM(node_->get_logger(), MANAGER_NAME << " Saved transform");

  return true;
}

common::ActionResult PartRegistrationManager::applyTransform()
{
  std::vector<geometry_msgs::msg::PoseArray> raster_strips;
  crs_motion_planning::parsePathFromFile(config_->toolpath_file, config_->target_frame_id, raster_strips);

  auto apply_transform = [](const geometry_msgs::msg::Pose& p,
                            const geometry_msgs::msg::Transform& t) -> geometry_msgs::msg::Pose {
    using namespace Eigen;
    Isometry3d t_eig = tf2::transformToEigen(t);
    Isometry3d p_eig;
    tf2::fromMsg(p, p_eig);
    return tf2::toMsg(t_eig * p_eig);
  };
  for (auto& poses : raster_strips)
  {
    for (std::size_t i = 0; i < poses.poses.size(); i++)
    {
      poses.poses[i] = apply_transform(poses.poses[i], part_transform_.transform);
    }
  }
  result_.rasters = raster_strips;
  RCLCPP_INFO_STREAM(node_->get_logger(), MANAGER_NAME << " Transformed raster strips and saved them");

  auto load_part_request = std::make_shared<crs_msgs::srv::LoadPart::Request>();
  load_part_request->path_to_part = config_->part_file;
  geometry_msgs::msg::Pose part_origin;
  part_origin.position.x = part_transform_.transform.translation.x;
  part_origin.position.y = part_transform_.transform.translation.y;
  part_origin.position.z = part_transform_.transform.translation.z;
  part_origin.orientation = part_transform_.transform.rotation;
  load_part_request->part_origin = part_origin;

  // calling load part service
  auto result_future = load_part_tesseract_client_->async_send_request(load_part_request);

  return true;
}

} /* namespace task_managers */
} /* namespace crs_application */
