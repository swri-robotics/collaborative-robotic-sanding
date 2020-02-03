/*
 * @author ros-industrial
 * @file process_execution_manager.cpp
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
#include "crs_application/task_managers/process_execution_manager.h"

static const double WAIT_INTERVAL_PERIOD = 0.1;
static const std::string MANAGER_NAME = "ProcessExecutionManager";
static const std::string CURRENT_JOINT_STATE_TOPIC = "joint_states";
static const std::string TRAJECTORY_EXEC_TOPIC = "set_trajectory_test";

template <class VarType, class ValType>
class ScopeExit
{
public:
  ScopeExit(VarType* var_ptr, ValType scope_exit_val):
    val_ptr_(var_ptr),
    scope_exit_val_(scope_exit_val)
  {

  }

  ~ScopeExit()
  {
    *val_ptr_ = scope_exit_val_;
  }

  VarType* val_ptr_;
  ValType scope_exit_val_;
};

namespace crs_application
{
namespace task_managers
{
ProcessExecutionManager::ProcessExecutionManager(std::shared_ptr<rclcpp::Node> node) : node_(node) {}

ProcessExecutionManager::ProcessExecutionManager(std::shared_ptr<rclcpp::Node> node):
    node_(node),
    executing_motion_(false)
{

common::ActionResult ProcessExecutionManager::init()
{
  traj_exec_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_EXEC_TOPIC,
                                                                                  rclcpp::QoS(1));
  executing_motion_ = false;
  return true;
}

common::ActionResult ProcessExecutionManager::configure(const ProcessExecutionConfig& config)
{
  config_ = std::make_shared<ProcessExecutionConfig>(config);
  return true;
}

common::ActionResult ProcessExecutionManager::setInput(const datatypes::ProcessExecutionData& input)
{
  input_ = std::make_shared<datatypes::ProcessExecutionData>(input);
  return true;
}

common::ActionResult ProcessExecutionManager::execProcess()
{
  const crs_msgs::msg::ProcessMotionPlan& process_plan = input_->process_plans[current_process_idx_];
  RCLCPP_INFO(node_->get_logger(),"%s Executing process %i",MANAGER_NAME.c_str(), current_process_idx_);
  if(!execTrajectory(process_plan.start))
  {
    common::ActionResult res;
    res.err_msg = boost::str(boost::format(
        "%s failed to execute start motion") % MANAGER_NAME.c_str());
    RCLCPP_ERROR(node_->get_logger(),"%s", res.err_msg.c_str());
    res.succeeded = false;
    return res;
  }

  for(std::size_t i = 0; i < process_plan.process_motions.size(); i++)
  {
    RCLCPP_INFO(node_->get_logger(),"%s Executing process path %i",MANAGER_NAME.c_str(), i);
    if(!execTrajectory(process_plan.process_motions[i]))
    {
      common::ActionResult res;
      res.err_msg = boost::str(boost::format(
          "%s failed to execute process motion %i") % MANAGER_NAME.c_str() % i);
      RCLCPP_ERROR(node_->get_logger(),"%s", res.err_msg.c_str());
      res.succeeded = false;
      return res;
    }

    if(i >= process_plan.free_motions.size())
    {
      RCLCPP_INFO(node_->get_logger(),"%s No free motion to follow process path %i, skipping",
                  MANAGER_NAME.c_str(), i);
      continue;
    }

    RCLCPP_INFO(node_->get_logger(),"%s Executing free motion %i",MANAGER_NAME.c_str(), i);
    if(!execTrajectory(process_plan.free_motions[i]))
    {
      common::ActionResult res;
      res.err_msg = boost::str(boost::format(
          "%s failed to execute free motion %i") % MANAGER_NAME.c_str() % i);
      RCLCPP_ERROR(node_->get_logger(),"%s", res.err_msg.c_str());
      res.succeeded = false;
      return res;
    }
  }

  RCLCPP_INFO(node_->get_logger(),"%s Executing end motion",MANAGER_NAME.c_str());
  if(!execTrajectory(process_plan.end))
  {
    common::ActionResult res;
    res.err_msg = boost::str(boost::format(
        "%s failed to execute end motion") % MANAGER_NAME.c_str());
    RCLCPP_ERROR(node_->get_logger(),"%s", res.err_msg.c_str());
    res.succeeded = false;
    return res;
  }
  RCLCPP_INFO(node_->get_logger(),"%s Completed process %i",MANAGER_NAME.c_str(), current_process_idx_);

  return true;
}

common::ActionResult ProcessExecutionManager::execMediaChange()
{
  datatypes::MediaChangeMotionPlan& mc_motion_plan = input_->media_change_plans[current_media_change_idx_];
  RCLCPP_INFO(node_->get_logger(),"%s Executing media change %i",MANAGER_NAME.c_str(), current_media_change_idx_);
  if(!execTrajectory(mc_motion_plan.start_traj))
  {
    common::ActionResult res;
    res.err_msg = boost::str(boost::format(
        "%s failed to execute media change motion") % MANAGER_NAME.c_str());
    RCLCPP_ERROR(node_->get_logger(),"%s", res.err_msg.c_str());
    res.succeeded = false;
    return res;
  }

  return true;
}

common::ActionResult ProcessExecutionManager::checkQueue()
{
  RCLCPP_WARN(node_->get_logger(),"%s not implemented yet",__PRETTY_FUNCTION__);
  common::ActionResult res;
  if(current_process_idx_ >= input_->process_plans.size()-1)
  {
    res.succeeded = true;
    res.opt_data = datatypes::ProcessExecActions::DONE;
    return res;
  }


  if(current_process_idx_ > current_media_change_idx_)
  {
    current_media_change_idx_ = current_process_idx_;
    if(current_media_change_idx_ < input_->media_change_plans.size())
    {
      res.succeeded = true;
      res.opt_data = datatypes::ProcessExecActions::EXEC_MEDIA_CHANGE;
      return res;
    }
  }

  current_process_idx_++;
  res.succeeded = true;
  res.opt_data = datatypes::ProcessExecActions::EXEC_PROCESS;
  return res;
}

common::ActionResult ProcessExecutionManager::execHome()
{
  RCLCPP_WARN(node_->get_logger(),"%s No home motion to execute", MANAGER_NAME.c_str());
  return true;
}

common::ActionResult ProcessExecutionManager::moveStart()
{
  // check input
  common::ActionResult res = checkPreReq();
  if(!res)
  {
    return res;
  }

  resetIndexes();

  if(input_->move_to_start.points.empty())
  {
    RCLCPP_WARN(node_->get_logger(),"%s No start trajectory was provided, skipping", MANAGER_NAME.c_str());
    return true;
  }

  RCLCPP_INFO(node_->get_logger(),"%s Executing start trajectory", MANAGER_NAME.c_str());
  res = execTrajectory(input_->move_to_start);
  return res;
}

common::ActionResult ProcessExecutionManager::execMoveReturn()
{
  datatypes::MediaChangeMotionPlan& mc_motion_plan = input_->media_change_plans[current_media_change_idx_];
  RCLCPP_INFO(node_->get_logger(),"%s Executing return move %i back to process ",MANAGER_NAME.c_str(),
              current_media_change_idx_);
  if(!execTrajectory(mc_motion_plan.return_traj))
  {
    common::ActionResult res;
    res.err_msg = boost::str(boost::format(
        "%s failed to execute return move") % MANAGER_NAME.c_str());
    RCLCPP_ERROR(node_->get_logger(),"%s", res.err_msg.c_str());
    res.succeeded = false;
    return res;
  }

  return true;
}

common::ActionResult ProcessExecutionManager::cancelMotion()
{
  if(!executing_motion_)
  {
    return true;
  }

  executing_motion_ = false;

  // attempting to send a new trajectory with just one point
  trajectory_msgs::msg::JointTrajectory traj;
  sensor_msgs::msg::JointState::SharedPtr js = common::getCurrentState(node_,
    CURRENT_JOINT_STATE_TOPIC,WAIT_INTERVAL_PERIOD);
  common::ActionResult res;
  if(!js)
  {
    res.succeeded = false;
    res.err_msg = "Failed to get latest joint state";
    RCLCPP_ERROR(node_->get_logger(),"%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }
  traj.header = js->header;
  traj.joint_names = js->name;
  traj.points.resize(1);
  traj.points.back().positions = js->position;
  traj.points.back().velocities.resize(traj.joint_names.size(),0.0);
  traj.points.back().effort.resize(traj.joint_names.size(),0.0);
  traj_exec_pub_->publish(traj);
  return true;
}

void ProcessExecutionManager::resetIndexes()
{
  current_process_idx_ = 0;
  current_media_change_idx_ = -1;
}

common::ActionResult ProcessExecutionManager::execTrajectory(const trajectory_msgs::msg::JointTrajectory& traj)
{
  executing_motion_ = true;
  ScopeExit<std::atomic<bool>, bool> scope_exit(&executing_motion_, false);

  std::chrono::duration<double> wait_period(WAIT_INTERVAL_PERIOD);
  sensor_msgs::msg::JointState end_js;
  end_js.name = traj.joint_names;
  end_js.position = traj.points.back().positions;
  rclcpp::Duration traj_dur(traj.points.back().time_from_start);

  rclcpp::Time start_time = node_->get_clock()->now();
  common::ActionResult res;

  // publish trajectory
  traj_exec_pub_->publish(traj);

  // now wait for completion
  while(rclcpp::ok())
  {
    if(!executing_motion_)
    {
      res.succeeded = false;
      res.err_msg = "Trajectory execution interrupted";
      return res;
    }

    // sleep
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(wait_period));

    // compute elapsed duration
    rclcpp::Duration elapsed_dur = node_->get_clock()->now() - start_time;
    if(elapsed_dur < traj_dur)
    {
      continue;
    }

    if(elapsed_dur > traj_dur + rclcpp::Duration(config_->traj_time_tolerance))
    {
      res.succeeded = false;
      res.err_msg = "Trajectory execution timeout";
      RCLCPP_ERROR(node_->get_logger(),"%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
      return res;
    }

    sensor_msgs::msg::JointState::SharedPtr js = common::getCurrentState(node_,
      CURRENT_JOINT_STATE_TOPIC,WAIT_INTERVAL_PERIOD);
    if(!js)
    {
      res.succeeded = false;
      res.err_msg = "Failed to get latest joint state";
      RCLCPP_ERROR(node_->get_logger(),"%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
      return res;
    }

    // compare current to end trajectory point
    std::string warn_msg;
    double diff = common::compare(end_js, *js, warn_msg);
    bool at_goal = true;
    if(diff < 0.0)
    {
      RCLCPP_WARN(node_->get_logger(),"%s %s", MANAGER_NAME.c_str(), warn_msg.c_str());
      continue;
    }

    at_goal = diff < config_->joint_tolerance;
    if(at_goal)
    {
      break;
    }
  }
  return true;
}

common::ActionResult ProcessExecutionManager::checkPreReq()
{
  if(!config_)
  {
    common::ActionResult res = { succeeded :false, err_msg: "No configuration has been provided, can not proceed"};
    RCLCPP_ERROR(node_->get_logger(),"%s %s",MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }

  if(!input_)
  {
    common::ActionResult res = { succeeded :false, err_msg: "No input data has been provided, can not proceed"};
    RCLCPP_ERROR(node_->get_logger(),"%s %s",MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }

  if(input_->process_plans.empty())
  {
    common::ActionResult res;
    res.succeeded = false;
    res.err_msg = "Process plans buffer is empty";
    RCLCPP_ERROR(node_->get_logger(),"%s %s",MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }

  if(traj_exec_pub_->get_subscription_count() > 0)
  {
    common::ActionResult res;
    res.succeeded = false;
    res.err_msg = "No trajectory execution subscribers were found";
    RCLCPP_ERROR(node_->get_logger(),"%s %s",MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }
  return true;
}

} /* namespace task_managers */
} /* namespace crs_application */
