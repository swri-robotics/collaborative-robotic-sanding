/*
 * @author Jorge Nicho
 * @file crs_executive.cpp
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

#include "crs_application/crs_executive.h"
#include <rclcpp/rclcpp.hpp>

static const std::string GET_CONFIGURATION_SERVICE = "get_configuration";
static const double WAIT_FOR_SERVICE_PERIOD = 5.0;
static const double WAIT_SERVICE_COMPLETION_PERIOD = 10.0;

namespace parameter_names
{
  static const std::string SM_FILE ="state_machine_file";
}

namespace state_names
{
  // configuration
  namespace general
  {
    static const std::string INITIALIZATION = "Initialization";
    static const std::string CONFIGURATION = "Configuration";
  }

  // scan acquisition
  namespace scan
  {
    static const std::string MOVE_ROBOT = "SC_Move_Robot";
    static const std::string VERIFICATION = "SC_Verification";
    static const std::string CAPTURE = "SC_Capture";
    static const std::string CHECK_QUEUE = "SC_Check_Queue";
    static const std::string SAVE_RESULTS = "SC_Save_Results";
  }

  // part registration
  namespace part_reg
  {
    static const std::string COMPUTE_TRANSFORM = "Compute_Transform";
    static const std::string PREVIEW = "RG_Preview";
    static const std::string APPLY_TRANSFORM = "Apply_transform";
    static const std::string SAVE_RESULTS = "RG_Save_Results";
  }

  // motion planning
  namespace motion_planning
  {
    static const std::string SPLIT_TOOLPATHS = "Split_ToolPaths";
    static const std::string PREVIEW = "MP_Preview";
    static const std::string PLAN_PROCESS_PATHS = "Plan_Process_Paths";
    static const std::string PLAN_MEDIA_CHANGES = "Plan_Media_Changes";
    static const std::string SAVE_RESULTS = "MP_Save_Results";
  }

  // part rework
  namespace part_rework
  {
    static const std::string GET_USER_SELECTION = "Get_User_Selection";
    static const std::string TRIM_TOOLPATHS = "Trim_ToolPaths";
    static const std::string PREVIEW = "PR_Preview";
    static const std::string SAVE_RESULTS = "PR_Save_Results";
  }

  // process execution
  namespace process_exec
  {
    static const std::string MOVE_START = "Move_Start";
    static const std::string EXEC_PROCESS = "Exec_Process";
    static const std::string EXEC_MEDIA_CHANGE = "Exec_Media_Change";
    static const std::string CHECK_QUEUE = "PE_Check_Queue";
    static const std::string EXEC_HOME = "Exec_Home";
  }
}

namespace crs_application
{

CRSExecutive::CRSExecutive(std::shared_ptr<rclcpp::Node> node):
    node_(node)
{
  if(!setup())
  {
    throw std::runtime_error("Failed initial setup");
  }
}

CRSExecutive::~CRSExecutive()
{

}

common::ActionResult CRSExecutive::initialize()
{
  if(scan_acqt_mngr_->init() &&
      motion_planning_mngr_->init() &&
      part_regt_mngr_->init() &&
      process_exec_mngr_->init() &&
      part_rework_mngr_->init() )
  {
    RCLCPP_INFO(node_->get_logger(),"All managers initialized");
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(),"One or more managers failed to initialize");
    return false;
  }

  RCLCPP_WARN(node_->get_logger(),"%s not fully implemented yet",__PRETTY_FUNCTION__);
  return true;
}

common::ActionResult CRSExecutive::configure()
{
  using namespace crs_msgs::srv;
  if(!get_config_client_->wait_for_service(std::chrono::duration<double>(5.0)))
  {
    RCLCPP_ERROR(node_->get_logger(),"%s wait for service '%s' timed out", node_->get_name(),
    		get_config_client_->get_service_name());
    return false;
  }

  GetConfiguration::Request::SharedPtr req = std::make_shared<GetConfiguration::Request>();
  std::shared_future < GetConfiguration::Response::SharedPtr > res = get_config_client_->async_send_request(req);

  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), res,
                                         std::chrono::duration<double>(WAIT_SERVICE_COMPLETION_PERIOD))
      != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s Failed to get configuration",node_->get_name());
    return false;
  }
  if(!res.get()->success)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s configution failed with err msg: ",node_->get_name(), res.get()->err_msg.c_str());
    return false;
  }

  process_config_ = res.get()->config;
  RCLCPP_ERROR(node_->get_logger(), "%s Got Configuration",node_->get_name());
  return configureManagers();
}

bool CRSExecutive::configureManagers()
{
  // TODO: populate each config structure from the data in process_config_

  return scan_acqt_mngr_->configure(task_managers::ScanAcquisitionConfig{}) &&
      motion_planning_mngr_->configure(task_managers::MotionPlanningConfig{}) &&
      part_regt_mngr_->configure(task_managers::PartRegistrationConfig{}) &&
      process_exec_mngr_->configure(task_managers::ProcessExecutionConfig{}) &&
      part_rework_mngr_->configure(task_managers::PartReworkConfig{});
}

bool CRSExecutive::addStateCallbacks(const std::map<std::string, StateCallbackInfo >& st_callbacks_map)
{
  using namespace crs_application;
  using namespace scxml_core;

  for(const auto& kv : st_callbacks_map)
  {
    // create actual callback
    auto sm_entry_cb = [this, kv](const Action& action) -> Response
    {
      auto sm_ = this->sm_;
      auto res = kv.second.entry_cb();
      if(!res)
      {
        sm_->postAction(Action{.id= kv.second.on_failed_action});
        Response sm_res;
        sm_res.success = res.succeeded;
        sm_res.msg = res.err_msg;
        return sm_res;
      }
      if(!kv.second.on_done_action.empty())
      {
        sm_->postAction(Action{.id= kv.second.on_done_action});
      }
      return true;
    };

    if(!sm_->addEntryCallback(kv.first,sm_entry_cb, kv.second.async))
    {
      return false;
    }

    std::cout<<"Added entry cb for state "<< kv.first << std::endl;

    if(!kv.second.exit_cb)
    {
      continue; // no exit callback so go to next
    }

    if(!sm_->addExitCallback(kv.first,[kv](){
      kv.second.exit_cb();
    }))
    {
      return false;
    }
  }
  return true;
}

bool CRSExecutive::setup()
{
  // getting parameters
  using ParamInfo = std::tuple<std::string, rclcpp::ParameterValue>;
  const std::vector<ParamInfo> parameters = {std::make_tuple(parameter_names::SM_FILE,
                                                                          node_->declare_parameter(parameter_names::SM_FILE))};

  // check parameters
  for(const auto& p : parameters)
  {
    if(std::get<1>(p).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      RCLCPP_ERROR(node_->get_logger(),"%s failed to read parameter '%s'", node_->get_name(), std::get<0>(p).c_str());
      return false;
    }
  }

  // create connections
  get_config_client_ = node_->create_client<crs_msgs::srv::GetConfiguration>(GET_CONFIGURATION_SERVICE);
  std::vector<rclcpp::ClientBase*> optional_clients = {get_config_client_.get()};
  for(auto& c : optional_clients)
  {
    if(!c->wait_for_service(std::chrono::duration<double>(WAIT_FOR_SERVICE_PERIOD)))
    {
      RCLCPP_WARN(node_->get_logger(),"%s wait for service '%s' timed out",
    		  node_->get_name(), c->get_service_name());
    }
  }

  // create managers
  scan_acqt_mngr_ = std::make_shared<task_managers::ScanAcquisitionManager>(node_);
  motion_planning_mngr_ = std::make_shared<task_managers::MotionPlanningManager>(node_);
  part_regt_mngr_ = std::make_shared<task_managers::PartRegistrationManager>(node_);
  process_exec_mngr_ = std::make_shared<task_managers::ProcessExecutionManager>(node_);
  part_rework_mngr_ = std::make_shared<task_managers::PartReworkManager>(node_);

  // create state machine
  std::string state_machine_file = std::get<1>(parameters[0]).get<std::string>();
  sm_ = std::make_shared<scxml_core::StateMachine>();
  if(!sm_->loadFile(state_machine_file))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load state machine file %s",state_machine_file.c_str());
    return -1;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded SM file %s", state_machine_file.c_str());

  return setupGeneralStates() &&
      setupScanAcquisitionStates() &&
      setupPartRegistrationStates() &&
      setupMotionPlanningStates() &&
      setupProcessExecStates() &&
      setupPartReworkStates() ;
}


bool CRSExecutive::setupGeneralStates()
{
  using namespace state_names;
  using namespace scxml_core;

  std::map<std::string, StateCallbackInfo > st_callbacks_map;

  st_callbacks_map[general::INITIALIZATION] = StateCallbackInfo{
    entry_cb : std::bind(&CRSExecutive::initialize, this),
    async : false};

  st_callbacks_map[general::CONFIGURATION] = StateCallbackInfo{
    entry_cb : std::bind(&CRSExecutive::configure, this),
    async : false};

  // now adding functions to SM
  return addStateCallbacks(st_callbacks_map);
}

bool CRSExecutive::setupMotionPlanningStates()
{
  using namespace state_names;
  using namespace scxml_core;

  std::map<std::string, StateCallbackInfo > st_callbacks_map;

  st_callbacks_map[motion_planning::SPLIT_TOOLPATHS] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::MotionPlanningManager::splitToolpaths, motion_planning_mngr_.get()),
    async : false};

  st_callbacks_map[motion_planning::PLAN_PROCESS_PATHS] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::MotionPlanningManager::planProcessPaths, motion_planning_mngr_.get()),
    async : true};

  st_callbacks_map[motion_planning::PLAN_MEDIA_CHANGES] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::MotionPlanningManager::planMediaChanges, motion_planning_mngr_.get()),
    async : true};

  st_callbacks_map[motion_planning::PREVIEW] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::MotionPlanningManager::showPreview, motion_planning_mngr_.get()),
    async : true,
    exit_cb : std::bind(&task_managers::MotionPlanningManager::hidePreview, motion_planning_mngr_.get()),
    on_done_action : ""};

  st_callbacks_map[motion_planning::SAVE_RESULTS] = StateCallbackInfo{
    entry_cb : [this]() -> common::ActionResult{
      return process_exec_mngr_->setInput(motion_planning_mngr_->getResult());
    },
    async : false};

  // now adding functions to SM
  return addStateCallbacks(st_callbacks_map);
}

bool CRSExecutive::setupPartRegistrationStates()
{
  using namespace state_names;
  using namespace scxml_core;

  std::map<std::string, StateCallbackInfo > st_callbacks_map;

  st_callbacks_map[part_reg::COMPUTE_TRANSFORM] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::PartRegistrationManager::computeTransform, part_regt_mngr_.get()),
    async : false};

  st_callbacks_map[part_reg::APPLY_TRANSFORM] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::PartRegistrationManager::applyTransform, part_regt_mngr_.get()),
    async : false};

  st_callbacks_map[part_reg::PREVIEW] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::PartRegistrationManager::showPreview, part_regt_mngr_.get()),
    async : false,
    exit_cb: nullptr,
    on_done_action : ""};

  st_callbacks_map[part_reg::SAVE_RESULTS] = StateCallbackInfo{
    entry_cb : [this]() -> common::ActionResult{
      return motion_planning_mngr_->setInput(part_regt_mngr_->getResult());
    },
    async : false};

  // now adding functions to SM
  return addStateCallbacks(st_callbacks_map);
}

bool CRSExecutive::setupProcessExecStates()
{
  using namespace state_names;
  using namespace scxml_core;

  std::map<std::string, StateCallbackInfo > st_callbacks_map;

  st_callbacks_map[process_exec::MOVE_START] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ProcessExecutionManager::moveStart, process_exec_mngr_.get()),
    async : false};

  st_callbacks_map[process_exec::EXEC_PROCESS] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ProcessExecutionManager::execProcess, process_exec_mngr_.get()),
    async : false};

  st_callbacks_map[process_exec::EXEC_MEDIA_CHANGE] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ProcessExecutionManager::execMediaChange, process_exec_mngr_.get()),
    async : false};

  st_callbacks_map[process_exec::EXEC_HOME] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ProcessExecutionManager::execMediaChange, process_exec_mngr_.get()),
    async : false};

  st_callbacks_map[process_exec::CHECK_QUEUE] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ProcessExecutionManager::checkQueue, process_exec_mngr_.get()),
    async : false,
    exit_cb: nullptr,
    on_done_action: action_names::SM_DONE,
    on_failed_action: action_names::SM_NEXT};

  // now adding functions to SM
  return addStateCallbacks(st_callbacks_map);
}

bool CRSExecutive::setupScanAcquisitionStates()
{
  using namespace state_names;
  using namespace scxml_core;

  std::map<std::string, StateCallbackInfo > st_callbacks_map;

  st_callbacks_map[scan::CAPTURE] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ScanAcquisitionManager::capture, scan_acqt_mngr_.get()),
    async : false};

  st_callbacks_map[scan::MOVE_ROBOT] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ScanAcquisitionManager::moveRobot, scan_acqt_mngr_.get()),
    async : false};

  st_callbacks_map[scan::VERIFICATION] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ScanAcquisitionManager::verify, scan_acqt_mngr_.get()),
    async : false};

  st_callbacks_map[scan::CHECK_QUEUE] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ScanAcquisitionManager::checkQueue, scan_acqt_mngr_.get()),
    async : false,
    exit_cb: nullptr,
    on_done_action: action_names::SM_DONE,
    on_failed_action: action_names::SM_NEXT};

  st_callbacks_map[scan::SAVE_RESULTS] = StateCallbackInfo{
    entry_cb : [this]() -> common::ActionResult{
      return part_regt_mngr_->setInput(scan_acqt_mngr_->getResult());
    },
    async : false};

  // now adding functions to SM
  return addStateCallbacks(st_callbacks_map);
}

bool CRSExecutive::setupPartReworkStates()
{
  using namespace state_names;
  using namespace scxml_core;

  std::map<std::string, StateCallbackInfo > st_callbacks_map;

  st_callbacks_map[part_rework::GET_USER_SELECTION] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::PartReworkManager::getUserSelection, part_rework_mngr_.get()),
    async : false};

  st_callbacks_map[part_rework::PREVIEW] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::PartReworkManager::showPreview, part_rework_mngr_.get()),
    async : false,
    exit_cb: nullptr,
    on_done_action: ""};

  st_callbacks_map[part_rework::TRIM_TOOLPATHS] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::PartReworkManager::trimToolpaths, part_rework_mngr_.get()),
    async : false};

  st_callbacks_map[part_rework::SAVE_RESULTS] = StateCallbackInfo{
    entry_cb : [this]() -> common::ActionResult{
      return motion_planning_mngr_->setInput(part_rework_mngr_->getResult());
    },
    async : false};

  // now adding functions to SM
  return addStateCallbacks(st_callbacks_map);
}
}
