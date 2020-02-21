/*
 * @author ros-industrial
 * @file process_execution_manager.h
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

#ifndef INCLUDE_CRS_APPLICATION_TASK_MANAGERS_PROCESS_EXECUTION_MANAGER_H_
#define INCLUDE_CRS_APPLICATION_TASK_MANAGERS_PROCESS_EXECUTION_MANAGER_H_

#include <atomic>
#include <memory>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include "crs_application/common/common.h"
#include "crs_application/common/datatypes.h"

namespace crs_application
{
namespace task_managers
{
struct ProcessExecutionConfig
{
  double traj_time_tolerance = 5.0; /** @brief time tolerance on trajectory duration */
  double wait_state_timeout = 1.0;  /** @brief seconds to wait for the current joint state*/
  double joint_tolerance =
      (3.1416 / 180.0) * 2.0; /** @brief how close the robot needs to be to the last position in radians */
};

class ProcessExecutionManager
{
public:
  ProcessExecutionManager(std::shared_ptr<rclcpp::Node> node);
  virtual ~ProcessExecutionManager();

  // initialization and configuration
  common::ActionResult init();
  common::ActionResult configure(const ProcessExecutionConfig& config);
  common::ActionResult setInput(const datatypes::ProcessExecutionData& input);

  // Process Actions
  /**
   * @brief Checks inputs and other variables and moves to start position if necessary
   * @return True on success, false otherwise
   */
  common::ActionResult moveStart();
  common::ActionResult execProcess();
  common::ActionResult execMediaChange();

  /**
   * @brief moves from media change position back to process
   * @return True on success, false otherwise
   */
  common::ActionResult execMoveReturn();

  /**
   * @brief checks if there are any process motions left in the queue
   * @return  True when the queue is empty
   */
  common::ActionResult checkQueue();

  common::ActionResult execHome();

  common::ActionResult cancelMotion();

protected:
  // support methods
  void resetIndexes();
  common::ActionResult execTrajectory(const trajectory_msgs::msg::JointTrajectory& traj);
  common::ActionResult checkPreReq();

  // roscpp
  using GoalHandleT = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::GoalHandle;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_exec_client_;
  std::shared_future<GoalHandleT::SharedPtr> trajectory_exec_fut_;

  // process data
  std::shared_ptr<ProcessExecutionConfig> config_ = nullptr;
  std::shared_ptr<datatypes::ProcessExecutionData> input_ = nullptr;

  // other
  int current_process_idx_ = 0;
  int current_media_change_idx_ = 0;

};

} /* namespace task_managers */
} /* namespace crs_application */

#endif /* INCLUDE_CRS_APPLICATION_TASK_MANAGERS_PROCESS_EXECUTION_MANAGER_H_ */
