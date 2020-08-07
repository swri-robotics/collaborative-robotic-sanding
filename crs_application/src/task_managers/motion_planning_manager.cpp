/*
 * @author ros-industrial
 * @file motion_planning_manager.cpp
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

#include <tf2_eigen/tf2_eigen.h>
#include <boost/optional.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "crs_application/task_managers/motion_planning_manager.h"

static const double WAIT_SERVICE_DURATION = 2.0;           // secs
static const double WAIT_MOTION_PLANNING_PERIOD = 1200.0;  // secs
static const double WAIT_JOINT_STATE_TIMEOUT = 2.0;
static const double MAX_JOINT_TOLERANCE = (M_PI / 180.0) * 1.0;
static const double MIN_PREVIEW_STEP_TIME = 0.05;  // secs
static const std::string CURRENT_JOINT_STATE_TOPIC = "joint_states";
static const std::string PREVIEW_NAME_PREFIX = "preview/";
static const std::string PREVIEW_JOINT_STATE_TOPIC = "preview/input_joints";
static const std::string CALL_FREESPACE_MOTION_SERVICE = "plan_freespace_motion";
static const std::string PLAN_PROCESS_MOTIONS_SERVICE = "plan_process_motion";
static const std::string MANAGER_NAME = "MotionPlanningManager";

static Eigen::Vector3d toEigen(const geometry_msgs::msg::Point& p_msg)
{
  return Eigen::Vector3d(p_msg.x, p_msg.y, p_msg.z);
}

namespace crs_application
{
namespace task_managers
{
MotionPlanningManager::MotionPlanningManager(std::shared_ptr<rclcpp::Node> node) : node_(node) {}
MotionPlanningManager::~MotionPlanningManager() {}
common::ActionResult MotionPlanningManager::init()
{
  using namespace crs_msgs;

  js_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(PREVIEW_JOINT_STATE_TOPIC, 10);

  call_freespace_planning_client_ = node_->create_client<srv::CallFreespaceMotion>(CALL_FREESPACE_MOTION_SERVICE);
  process_motion_planning_client_ =
      node_->create_client<crs_msgs::srv::PlanProcessMotions>(PLAN_PROCESS_MOTIONS_SERVICE);

  // checking clients
  std::vector<rclcpp::ClientBase*> clients = { call_freespace_planning_client_.get(),
                                               process_motion_planning_client_.get() };
  if (!std::all_of(clients.begin(), clients.end(), [this](rclcpp::ClientBase* c) {
        bool found = c->wait_for_service(std::chrono::duration<float>(WAIT_SERVICE_DURATION));
        RCLCPP_ERROR_EXPRESSION(node_->get_logger(), !found, "Service %s not found", c->get_service_name());
        return found;
      }))
  {
    RCLCPP_ERROR(node_->get_logger(), "%s: One or more services were not found", MANAGER_NAME.c_str());
    return false;
  }

  return true;
}

common::ActionResult MotionPlanningManager::configure(const config::MotionPlanningConfig& config)
{
  config_ = std::make_shared<config::MotionPlanningConfig>(config);
  home_js_.reset();
  if (config_->joint_home_position.empty() || config_->home_joint_names.empty())
  {
    common::ActionResult res;
    res.succeeded = false;
    res.err_msg = "No home position was provided";
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }

  // constructing home position
  home_js_ = std::make_shared<sensor_msgs::msg::JointState>();
  home_js_->name = config_->home_joint_names;
  home_js_->position = config_->joint_home_position;
  home_js_->velocity.resize(home_js_->position.size(), 0.0);
  home_js_->effort.resize(home_js_->position.size(), 0.0);
  return true;
}

common::ActionResult MotionPlanningManager::setInput(const std::vector<datatypes::ProcessToolpathData>& input)
{
  if (input.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "%s found no rasters in the process path", MANAGER_NAME.c_str());
    return false;
  }
  input_process_toolpaths.clear();
  input_process_toolpaths.assign(input.begin(),input.end());
  return true;
}

common::ActionResult MotionPlanningManager::checkPreReq()
{
  if (!config_)
  {
    common::ActionResult res = { succeeded : false, err_msg : "No configuration has been provided, can not proceed" };
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }

  if (input_process_toolpaths.empty())
  {
    common::ActionResult res = { succeeded : false, err_msg : "No input data has been provided, can not proceed" };
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }
  return true;
}

common::ActionResult MotionPlanningManager::splitToolpaths()
{
  common::ActionResult res = checkPreReq();
  if (!res)
  {
    return res;
  }

  // computing split distance
  const double split_dist = config_->tool_speed * config_->media_change_time;

  // finding breakpoints
  std::vector<datatypes::ProcessToolpathData> toolpaths_processes;
  double current_dist = 0.0;
  Eigen::Vector3d p1, p2;
  for(std::size_t i = 0; i < input_process_toolpaths.size(); i++)
  {
    const datatypes::ProcessToolpathData& current_toolpath = input_process_toolpaths[i];
    std::vector<std::vector<std::size_t> > rasters_breakpoints(current_toolpath.rasters.size());
    for (std::size_t r_idx = 0; r_idx < current_toolpath.rasters.size(); r_idx++)
    {
      const geometry_msgs::msg::PoseArray& raster = current_toolpath.rasters[r_idx];
      for (std::size_t p_idx = 0; p_idx < raster.poses.size() - 1; p_idx++)
      {
        p1 = toEigen(raster.poses[p_idx].position);
        p2 = toEigen(raster.poses[p_idx + 1].position);

        double d = (p2 - p1).norm();
        if (current_dist + d > split_dist)
        {
          RCLCPP_INFO(node_->get_logger(), "Found split at point %lu of raster %lu", p_idx, r_idx);
          rasters_breakpoints[r_idx].push_back(p_idx);
          current_dist = 0;
        }
        current_dist += d;
      }
    }

    // splitting toolpath
    toolpaths_processes.push_back(datatypes::ProcessToolpathData());
    for (std::size_t r_idx = 0; r_idx < rasters_breakpoints.size(); r_idx++)
    {
      const geometry_msgs::msg::PoseArray& raster = current_toolpath.rasters[r_idx];
      auto& breakpoints_indices = rasters_breakpoints[r_idx];
      if (breakpoints_indices.empty())
      {
        toolpaths_processes.back().rasters.push_back(raster);
        continue;
      }

      std::size_t start_idx = 0;
      std::size_t end_idx;
      geometry_msgs::msg::PoseArray new_raster;
      for (std::size_t p_idx = 0; p_idx < breakpoints_indices.size(); p_idx++)
      {
        // copy portion of raster leading up to breakpoint into current toolpath
        end_idx = breakpoints_indices[p_idx];
        new_raster.poses.clear();
        std::copy(raster.poses.begin() + start_idx, raster.poses.begin() + end_idx, std::back_inserter(new_raster.poses));
        toolpaths_processes.back().rasters.push_back(new_raster);

        // create new toolpath
        toolpaths_processes.resize(toolpaths_processes.size() + 1);
        start_idx = end_idx;
      }

      // copy remaining segment of raster into toolpath
      if (end_idx < raster.poses.size() - 1)
      {
        new_raster.poses.clear();
        std::copy(raster.poses.begin() + end_idx, raster.poses.end(), std::back_inserter(new_raster.poses));
        toolpaths_processes.back().rasters.push_back(new_raster);
      }
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Split original process into %lu", toolpaths_processes.size());
  process_toolpaths_ = std::move(toolpaths_processes);
  return true;
}

common::ActionResult MotionPlanningManager::planProcessPaths()
{
  using namespace crs_msgs;

  common::ActionResult res = checkPreReq();
  if (!res)
  {
    return res;
  }

  // grabbing current state
  auto current_st = common::getCurrentState(this->node_, CURRENT_JOINT_STATE_TOPIC, WAIT_JOINT_STATE_TIMEOUT);
  if (!current_st)
  {
    res.succeeded = false;
    res.err_msg = "Failed to get the current state";
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }

  // grabbing current joint pose
  result_.move_to_start.joint_names.clear();
  result_.move_to_start.points.clear();
  double diff = common::compare(*home_js_, *current_st, res.err_msg);
  if (diff < 0.0)
  {
    res.succeeded = false;
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }

  sensor_msgs::msg::JointState start_position = *current_st;

  if (diff > MAX_JOINT_TOLERANCE && config_->pre_move_home)
  {
    // planning free start motion to home from current
    srv::CallFreespaceMotion::Request::SharedPtr free_motion_req =
        std::make_shared<srv::CallFreespaceMotion::Request>();
    free_motion_req->goal_position = *home_js_;
    free_motion_req->num_steps = 0;
    free_motion_req->target_link = config_->tool_frame;  // shouldn't make a difference since planning to a joint goal
    free_motion_req->start_position = *current_st;
    free_motion_req->execute = true;

    RCLCPP_INFO(node_->get_logger(), "Planning and moving to home position");
    boost::optional<trajectory_msgs::msg::JointTrajectory> opt = planFreeSpace("CURRENT TO HOME", free_motion_req);
    if (!opt.is_initialized())
    {
      return false;
    }
    start_position = *home_js_;
    result_.move_to_start = opt.get();
  }

  // copying data into request
  srv::PlanProcessMotions::Request::SharedPtr req = std::make_shared<srv::PlanProcessMotions::Request>();
  req->tool_link = config_->tool_frame;
  req->approach_dist = config_->approac_dist;
  req->retreat_dist = config_->retreat_dist;
  req->tool_speed = config_->tool_speed;
  req->tool_offset = tf2::toMsg(config_->offset_pose);
  req->target_force = config_->target_force;
  req->start_position = start_position;
  req->end_position = start_position;

  for (datatypes::ProcessToolpathData& pd : process_toolpaths_)
  {
    msg::ToolProcessPath process_path;
    for (const geometry_msgs::msg::PoseArray& raster : pd.rasters)
    {
      process_path.rasters.push_back(raster);
    }
    req->process_paths.push_back(process_path);
  }

  std::shared_future<srv::PlanProcessMotions::Response::SharedPtr> result_future =
      process_motion_planning_client_->async_send_request(req);

  if (result_future.wait_for(std::chrono::duration<double>(WAIT_MOTION_PLANNING_PERIOD)) != std::future_status::ready)
  {
    res.succeeded = false;
    res.err_msg = boost::str(boost::format("%s process planning service error or timeout") % MANAGER_NAME);
    RCLCPP_ERROR_STREAM(node_->get_logger(), res.err_msg);
    return res;
  }

  if (!result_future.get()->succeeded)
  {
    res.succeeded = false;
    res.err_msg = boost::str(boost::format("%s process planning failed") % MANAGER_NAME);
    RCLCPP_ERROR_STREAM(node_->get_logger(), res.err_msg);
    return res;
  }

  // saving process plans
  RCLCPP_INFO(node_->get_logger(), "Successfully planned all process toolpaths");
  result_.process_plans = result_future.get()->plans;
  return true;
}

boost::optional<trajectory_msgs::msg::JointTrajectory>
MotionPlanningManager::planFreeSpace(const std::string& plan_name,
                                     crs_msgs::srv::CallFreespaceMotion::Request::SharedPtr req)
{
  using namespace crs_msgs::srv;
  std::shared_future<CallFreespaceMotion::Response::SharedPtr> result_future =
      call_freespace_planning_client_->async_send_request(req);

  if (result_future.wait_for(std::chrono::duration<double>(WAIT_MOTION_PLANNING_PERIOD)) != std::future_status::ready)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "%s freespace planning service for '%s' error or timeout",
                 MANAGER_NAME.c_str(),
                 plan_name.c_str());
    return boost::none;
  }

  if (!result_future.get()->success)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "%s freespace planning for '%s' failed, %s",
                 MANAGER_NAME.c_str(),
                 plan_name.c_str(),
                 result_future.get()->message.c_str());
    return boost::none;
  }

  RCLCPP_INFO(node_->get_logger(), "%s freespace planning for '%s' succeeded", MANAGER_NAME.c_str(), plan_name.c_str());
  return result_future.get()->output_trajectory;
}

common::ActionResult MotionPlanningManager::planMediaChanges()
{
  using namespace crs_msgs::srv;

  common::ActionResult res = checkPreReq();
  if (!res)
  {
    return res;
  }

  result_.media_change_plans.clear();
  if (result_.process_plans.size() < 2)
  {
    RCLCPP_INFO(node_->get_logger(), "%s no media change needed", MANAGER_NAME.c_str());
    return true;
  }

  CallFreespaceMotion::Request::SharedPtr req = std::make_shared<CallFreespaceMotion::Request>();
  req->target_link = config_->tool_frame;
  if (config_->joint_media_position.empty() || config_->media_joint_names.empty())
  {
    // use cartesian tool pose when no joint pose is available
    geometry_msgs::msg::Pose pose_msg = tf2::toMsg(config_->media_change_pose);
    std::tie(req->goal_pose.translation.x, req->goal_pose.translation.y, req->goal_pose.translation.z) =
        std::make_tuple(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
    req->goal_pose.rotation = pose_msg.orientation;
  }
  else
  {
    req->goal_position.position = config_->joint_media_position;
    req->goal_position.name = config_->media_joint_names;
  }
  req->execute = false;
  req->num_steps = 0;  // planner should use default

  for (std::size_t i = 0; i < result_.process_plans.size() - 1; i++)
  {
    datatypes::MediaChangeMotionPlan media_change_plan;
    const trajectory_msgs::msg::JointTrajectory& traj = result_.process_plans[i].end;
    req->start_position.name = traj.joint_names;
    req->start_position.position = traj.points.back().positions;

    // calling freespace motion planning to media change position
    boost::optional<trajectory_msgs::msg::JointTrajectory> opt = planFreeSpace("START MEDIA CHANGE", req);
    if (!opt.is_initialized())
    {
      return false;
    }
    media_change_plan.start_traj = opt.get();

    // free space motion planning for return move
    req->start_position.position = media_change_plan.start_traj.points.back().positions;
    req->goal_position.position = media_change_plan.start_traj.points.front().positions;
    req->goal_position.name = media_change_plan.start_traj.joint_names;
    opt = planFreeSpace("RETURN TO PROCESS", req);
    if (!opt.is_initialized())
    {
      return false;
    }
    media_change_plan.return_traj = opt.get();

    // saving plan
    result_.media_change_plans.push_back(media_change_plan);
  }

  return true;
}

common::ActionResult MotionPlanningManager::showPreview()
{
  common::ActionResult res = true;
  if (result_.process_plans.empty())
  {
    res.err_msg = "no process plan has been produced";
    RCLCPP_WARN(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }
  publish_preview_enabled_ = true;

  auto publish_joint_trajs = [this](const trajectory_msgs::msg::JointTrajectory& traj, double time_factor) -> bool {
    sensor_msgs::msg::JointState js_msg;
    js_msg.name = traj.joint_names;
    js_msg.position.resize(js_msg.name.size(), 0.0);
    js_msg.velocity.resize(js_msg.name.size(), 0.0);
    js_msg.effort.resize(js_msg.name.size(), 0.0);
    std::chrono::duration<double> prev_dur(0.0);
    std::chrono::duration<double> current_dur;

    // adding preview prefix to joint names
    std::for_each(js_msg.name.begin(), js_msg.name.end(), [](std::string& j) { j = PREVIEW_NAME_PREFIX + j; });

    for (std::size_t j = 0; j < traj.points.size(); j++)
    {
      if (!publish_preview_enabled_)
      {
        return false;
      }

      js_msg.position = traj.points[j].positions;
      current_dur = rclcpp::Duration(traj.points[j].time_from_start).to_chrono<std::chrono::seconds>();
      std::chrono::duration<double> diff = (current_dur - prev_dur) / time_factor;
      if (diff.count() < MIN_PREVIEW_STEP_TIME)
      {
        diff = std::chrono::duration<double>(MIN_PREVIEW_STEP_TIME);
      }
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(diff));

      js_pub_->publish(js_msg);
      prev_dur = current_dur;
    }
    return true;
  };

  while (publish_preview_enabled_)
  {
    // previewing process plan moves now
    if (!publish_joint_trajs(result_.move_to_start, config_->preview_time_scaling))
    {
      return true;
    }

    for (std::size_t i = 0; i < result_.process_plans.size(); i++)
    {
      const crs_msgs::msg::ProcessMotionPlan& process_plan = result_.process_plans[i];

      // previewing process plan moves now
      if (!publish_joint_trajs(process_plan.start, config_->preview_time_scaling))
      {
        return true;
      }

      for (std::size_t j = 0; j < process_plan.process_motions.size(); j++)
      {
        if (!publish_joint_trajs(process_plan.process_motions[j], config_->preview_time_scaling))
        {
          return true;
        }
      }
      if (!publish_joint_trajs(process_plan.end, config_->preview_time_scaling))
      {
        return true;
      }

      // previewing media change moves now
      if (result_.media_change_plans.size() > i)
      {
        if (!publish_joint_trajs(result_.media_change_plans[i].start_traj, config_->preview_time_scaling))
        {
          return true;
        }

        if (!publish_joint_trajs(result_.media_change_plans[i].return_traj, config_->preview_time_scaling))
        {
          return true;
        }
      }
    }
  }
  return true;
}

common::ActionResult MotionPlanningManager::hidePreview()
{
  publish_preview_enabled_ = false;
  if (result_.process_plans.empty())
  {
    return true;
  }

  // wait a second
  rclcpp::sleep_for(std::chrono::seconds(1));

  sensor_msgs::msg::JointState js_msg;
  js_msg.name = result_.process_plans.front().process_motions.front().joint_names;

  // adding preview prefix to joint names
  std::for_each(js_msg.name.begin(), js_msg.name.end(), [](std::string& j) { j = PREVIEW_NAME_PREFIX + j; });

  js_msg.position.resize(js_msg.name.size(), 0.0);
  js_msg.velocity.resize(js_msg.name.size(), 0.0);
  js_msg.effort.resize(js_msg.name.size(), 0.0);
  for (std::size_t i = 0; i < 10; i++)
  {
    js_pub_->publish(js_msg);
  }
  return true;
}

} /* namespace task_managers */
} /* namespace crs_application */
