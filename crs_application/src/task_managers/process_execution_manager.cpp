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

#include "crs_application/task_managers/process_execution_manager.h"

static const std::string MANAGER_NAME = "ProcessExecutionManager";

namespace crs_application
{
namespace task_managers
{

ProcessExecutionManager::ProcessExecutionManager(std::shared_ptr<rclcpp::Node> node):
    node_(node)
{

}

ProcessExecutionManager::~ProcessExecutionManager()
{

}

common::ActionResult ProcessExecutionManager::init()
{
  RCLCPP_WARN(node_->get_logger(),"%s not implemented yet",__PRETTY_FUNCTION__);
  return true;
}

common::ActionResult ProcessExecutionManager::configure(const ProcessExecutionConfig& config)
{
  RCLCPP_WARN(node_->get_logger(),"%s not implemented yet",__PRETTY_FUNCTION__);
  return true;
}

common::ActionResult ProcessExecutionManager::setInput(const datatypes::ProcessExecutionData& input)
{
  input_ = std::make_shared<datatypes::ProcessExecutionData>(input);
  return true;
}

common::ActionResult ProcessExecutionManager::execProcess()
{
  RCLCPP_WARN(node_->get_logger(),"%s not implemented yet",__PRETTY_FUNCTION__);
  return true;
}

common::ActionResult ProcessExecutionManager::execMediaChange()
{
  RCLCPP_WARN(node_->get_logger(),"%s not implemented yet",__PRETTY_FUNCTION__);
  return true;
}

common::ActionResult ProcessExecutionManager::checkQueue()
{
  RCLCPP_WARN(node_->get_logger(),"%s not implemented yet",__PRETTY_FUNCTION__);
  return true;
}

common::ActionResult ProcessExecutionManager::execHome()
{
  RCLCPP_WARN(node_->get_logger(),"%s not implemented yet",__PRETTY_FUNCTION__);
  return true;
}

common::ActionResult ProcessExecutionManager::moveStart()
{
  // check input
  common::ActionResult res = true;
  if(input_ == nullptr)
  {
    res.succeeded = false;
    res.err_msg = "No process data has been provided";
    RCLCPP_ERROR(node_->get_logger(),"%s %s",MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }

  if(input_->process_plans.empty())
  {
    res.succeeded = false;
    res.err_msg = "Process plans buffer is empty";
    RCLCPP_ERROR(node_->get_logger(),"%s %s",MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }

  current_process_idx_ = 0;
  return true;
}

} /* namespace task_managers */
} /* namespace crs_application */

