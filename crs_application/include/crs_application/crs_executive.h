/*
 * @author Jorge Nicho
 * @file crs_executive.h
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

#ifndef INCLUDE_CRS_APPLICATION_CRS_EXECUTIVE_H_
#define INCLUDE_CRS_APPLICATION_CRS_EXECUTIVE_H_

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <boost/format.hpp>
#include <chrono>
#include <crs_msgs/srv/get_configuration.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <scxml_core/state_machine.h>
#include <QObject>

#include "crs_application/common/config.h"
#include "crs_application/task_managers/motion_planning_manager.h"
#include "crs_application/task_managers/part_registration_manager.h"
#include "crs_application/task_managers/part_rework_manager.h"
#include "crs_application/task_managers/process_execution_manager.h"
#include "crs_application/task_managers/scan_acquisition_manager.h"

namespace crs_application
{
/**
 * @namespace action_names
 * @brief actions allowed by the SM
 */
namespace action_names
{
static const std::string SM_FAILURE = "sm_failure";
static const std::string SM_NEXT = "sm_next";
static const std::string SM_DONE = "sm_done";
static const std::string SM_EXEC_PROC = "sm_exec_proc";
static const std::string SM_EXEC_MDCH = "sm_exec_mdch";

}  // namespace action_names

class CRSExecutive : public QObject
{
public:
  CRSExecutive(std::shared_ptr<rclcpp::Node> node);
  virtual ~CRSExecutive();

  std::shared_ptr<scxml_core::StateMachine> getSM() { return sm_; }

protected:
  struct StateCallbackInfo
  {
    std::function<crs_application::common::ActionResult()> entry_cb = nullptr;
    bool async = false;
    std::function<crs_application::common::ActionResult()> exit_cb = nullptr;
    std::string on_done_action = action_names::SM_DONE;
    std::string on_failed_action = action_names::SM_FAILURE;
  };

  // general state callbacks
  common::ActionResult initialize();
  common::ActionResult configure();

  // setup functions
  bool setup();
  bool setupGeneralStates();
  bool setupMotionPlanningStates();
  bool setupPartRegistrationStates();
  bool setupProcessExecStates();
  bool setupScanAcquisitionStates();
  bool setupPartReworkStates();

  // support methods
  bool addStateCallbacks(const std::map<std::string, StateCallbackInfo>& st_callbacks_map);
  bool configureManagers(YAML::Node& node);

  std::shared_ptr<scxml_core::StateMachine> sm_;
  std::string current_state_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr execute_state_subs_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr print_actions_server_;

  // task managers
  std::shared_ptr<task_managers::ScanAcquisitionManager> scan_acqt_mngr_;
  std::shared_ptr<task_managers::MotionPlanningManager> motion_planning_mngr_;
  std::shared_ptr<task_managers::PartRegistrationManager> part_regt_mngr_;
  std::shared_ptr<task_managers::ProcessExecutionManager> process_exec_mngr_;
  std::shared_ptr<task_managers::PartReworkManager> part_rework_mngr_;

  // rclcpp
  rclcpp::callback_group::CallbackGroup::SharedPtr get_config_callback_group_;
  rclcpp::Client<crs_msgs::srv::GetConfiguration>::SharedPtr get_config_client_;
  rclcpp::TimerBase::SharedPtr state_pub_timer_;

  // data
  crs_msgs::msg::ProcessConfiguration process_config_;
};

}  // namespace crs_application

#endif /* INCLUDE_CRS_APPLICATION_CRS_EXECUTIVE_H_ */
