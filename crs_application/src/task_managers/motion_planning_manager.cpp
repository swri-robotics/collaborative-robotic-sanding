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

static const double WAIT_SERVICE_DURATION = 2.0; // secs
static const double WAIT_SERVICE_COMPLETION_PERIOD = 30.0; // secs
static const std::string CALL_FREESPACE_MOTION_SERVICE = "plan_free_space";
static const std::string PLAN_PROCESS_MOTIONS_SERVICE = "plan_process_motions";
static const std::string MANAGER_NAME = "MotionPlanningManager";

static Eigen::Vector3d toEigen(const geometry_msgs::msg::Point& p_msg)
{
  return Eigen::Vector3d(p_msg.x, p_msg.y, p_msg.z);
}

namespace crs_application
{
namespace task_managers
{

MotionPlanningManager::MotionPlanningManager(std::shared_ptr<rclcpp::Node> node):
    node_(node)
{

}

MotionPlanningManager::~MotionPlanningManager()
{

}

common::ActionResult MotionPlanningManager::init()
{
  using namespace crs_msgs;
  call_freespace_planning_client_ = node_->create_client<srv::CallFreespaceMotion>(CALL_FREESPACE_MOTION_SERVICE);
  process_motion_planning_client_ = node_->create_client<crs_msgs::srv::PlanProcessMotions>(PLAN_PROCESS_MOTIONS_SERVICE);

  // checking clients
  std::vector<rclcpp::ClientBase*> clients = {call_freespace_planning_client_.get(),
                                              process_motion_planning_client_.get()};
  if(std::all_of(clients.begin(), clients.end(),[](rclcpp::ClientBase* c){
    return c->wait_for_service(std::chrono::duration<float>(WAIT_SERVICE_DURATION));
  }))
  {
    RCLCPP_WARN(node_->get_logger(),"%s: One or more services were not found", MANAGER_NAME.c_str());
    return false;
  }

  return true;
}

common::ActionResult MotionPlanningManager::configure(const MotionPlanningConfig& config)
{
  config_ = std::make_shared<MotionPlanningConfig>(config);
  return true;
}

common::ActionResult MotionPlanningManager::setInput(const datatypes::ProcessToolpathData& input)
{
  if(input.rasters.empty())
  {
    RCLCPP_ERROR(node_->get_logger(),"%s found no rasters in the process path", MANAGER_NAME.c_str());
    return false;
  }
  input_ = std::make_shared<datatypes::ProcessToolpathData>(input);
  return true;
}


common::ActionResult MotionPlanningManager::checkPreReq()
{
  if(!config_)
  {
    common::ActionResult res = { succeeded :false, err_msg: "No configuration has been provided, can not proceed"};
    RCLCPP_WARN(node_->get_logger(),"%s %s",MANAGER_NAME.c_str(), res.err_msg.c_str());
  }

  if(!input_)
  {
    common::ActionResult res = { succeeded :false, err_msg: "No input data has been provided, can not proceed"};
    RCLCPP_WARN(node_->get_logger(),"%s %s",MANAGER_NAME.c_str(), res.err_msg.c_str());
  }
  return true;
}

common::ActionResult MotionPlanningManager::splitToolpaths()
{
  common::ActionResult res = checkPreReq();
  if( !res )
  {
    return res;
  }

  // computing split distance
  const double split_dist = config_->tool_speed * config_->media_change_time;

  // splitting paths
  std::vector<datatypes::ProcessToolpathData> toolpaths_processes;
  double current_dist = 0.0;
  Eigen::Vector3d p1, p2;
  toolpaths_processes.resize(1);
  for(std::size_t r_idx = 0 ; r_idx < input_->rasters.size(); r_idx++)
  {
    geometry_msgs::msg::PoseArray& raster = input_->rasters[r_idx];
    int split_point_idx = -1;
    for(std::size_t p_idx = 0; p_idx < raster.poses.size() - 1; p_idx++)
    {
      p1 = toEigen(raster.poses[p_idx].position);
      p2 = toEigen(raster.poses[p_idx + 1].position);

      double d = (p2 - p1).norm();
      if(current_dist + d > split_dist)
      {
        RCLCPP_INFO(node_->get_logger(),"Found split at point %lu of raster %lu",p_idx, r_idx);
        split_point_idx = p_idx;
        current_dist = 0;
        break;
      }
      current_dist += d;
    }

    if(split_point_idx >=0)
    {
      std::remove_reference <decltype(raster)>::type new_raster;
      std::copy(raster.poses.begin(), raster.poses.begin() + split_point_idx,
                std::back_inserter(new_raster.poses));
      toolpaths_processes.back().rasters.push_back(new_raster);

      // copying remaining part of raster into a new  process
      toolpaths_processes.resize(toolpaths_processes.size() + 1);
      new_raster.poses.clear();
      std::copy(raster.poses.begin() + split_point_idx + 1, raster.poses.end(),
                std::back_inserter(new_raster.poses));
      toolpaths_processes.back().rasters.push_back(new_raster);
    }
    else
    {
      // no split so copy current raster into current process
      toolpaths_processes.back().rasters.push_back(raster);
    }
  }
  RCLCPP_INFO(node_->get_logger(),"Split original process into %lu", toolpaths_processes.size());
  process_toolpaths_ = std::move(toolpaths_processes);
  return true;
}

common::ActionResult MotionPlanningManager::planProcessPaths()
{
  using namespace crs_msgs;

  common::ActionResult res = checkPreReq();
  if( !res )
  {
    return res;
  }

  // copying data into request
  srv::PlanProcessMotions::Request::SharedPtr req = std::make_shared<srv::PlanProcessMotions::Request>();
  for(datatypes::ProcessToolpathData& pd : process_toolpaths_)
  {
    msg::ToolProcessPath process_path;
    for(const geometry_msgs::msg::PoseArray& raster : pd.rasters)
    {
      process_path.rasters.push_back(raster);
    }
    req->process_paths.push_back(process_path);
  }

  std::shared_future<srv::PlanProcessMotions::Response::SharedPtr> fut = process_motion_planning_client_->async_send_request(
      req);
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), fut,
                                         std::chrono::duration<double>(WAIT_SERVICE_COMPLETION_PERIOD))
      != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s process planning service error or timeout",MANAGER_NAME.c_str());
    return false;
  }

  if(!fut.get()->succeeded)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s process planning failed",MANAGER_NAME.c_str());
  }

  // saving process plans
  result_.process_plans = fut.get()->plans;
  return true;
}

common::ActionResult MotionPlanningManager::planMediaChanges()
{
  using namespace crs_msgs::srv;

  common::ActionResult res = checkPreReq();
  if( !res )
  {
    return res;
  }

  result_.media_change_plans.clear();
  if(result_.process_plans.size() < 2)
  {
    RCLCPP_INFO(node_->get_logger(),"%s no media change needed", MANAGER_NAME.c_str());
    return true;
  }

  CallFreespaceMotion::Request::SharedPtr req = std::make_shared<CallFreespaceMotion::Request>();
  req->target_link = config_->tool_frame;
  geometry_msgs::msg::Pose pose_msg = tf2::toMsg(config_->media_change_pose);
  std::tie(req->goal_pose.translation.x, req->goal_pose.translation.y,
           req->goal_pose.translation.z ) = std::make_tuple(pose_msg.position.x,
                                                            pose_msg.position.y,
                                                            pose_msg.position.z);
  req->goal_pose.rotation = pose_msg.orientation;
  req->execute = false;
  req->num_steps = 0; // planner should use default

  auto call_planning = [this](const std::string& plan_name, CallFreespaceMotion::Request::SharedPtr req) ->
      boost::optional<trajectory_msgs::msg::JointTrajectory>
    {
      std::shared_future<CallFreespaceMotion::Response::SharedPtr> fut = call_freespace_planning_client_->async_send_request(
          req);

      if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), fut,
                                             std::chrono::duration<double>(WAIT_SERVICE_COMPLETION_PERIOD))
          != rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(), "%s freespace planning service for '%s' move error or timeout",MANAGER_NAME.c_str(),
                     plan_name.c_str());
        return boost::none;
      }

      if(!fut.get()->success)
      {
        RCLCPP_ERROR(node_->get_logger(), "%s freespace planning for media change '%s' move failed, %s",MANAGER_NAME.c_str(),
                     plan_name.c_str(), fut.get()->message.c_str());
        return boost::none;
      }

      RCLCPP_INFO(node_->get_logger(), "%s freespace planning for media change '%s' move succeeded",MANAGER_NAME.c_str(),
                   plan_name.c_str());

      return fut.get()->output_trajectory;
    };

  for(std::size_t i = 0 ; i < result_.process_plans.size() - 1; i++)
  {
    datatypes::MediaChangeMotionPlan media_change_plan;
    const trajectory_msgs::msg::JointTrajectory& traj  = result_.process_plans[i].end;
    req->start_position.name = traj.joint_names;
    req->start_position.position = traj.points.back().positions;

    // calling freespace motion planning to media change position
    boost::optional<trajectory_msgs::msg::JointTrajectory> opt = call_planning("START", req);
    if(!opt.is_initialized())
    {
      return false;
    }
    media_change_plan.start_traj = opt.get();

    // free space motion planning for return move
    req->start_position.position = media_change_plan.start_traj.points.back().positions;
    req->goal_position.position = media_change_plan.start_traj.points.front().positions;
    opt = call_planning("RETURN", req);
    if(!opt.is_initialized())
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
  RCLCPP_WARN(node_->get_logger(),"%s not implemented yet",__PRETTY_FUNCTION__);
  return true;
}

common::ActionResult MotionPlanningManager::hidePreview()
{
  RCLCPP_WARN(node_->get_logger(),"%s not implemented yet",__PRETTY_FUNCTION__);
  return true;
}

} /* namespace task_managers */
} /* namespace crs_application */
