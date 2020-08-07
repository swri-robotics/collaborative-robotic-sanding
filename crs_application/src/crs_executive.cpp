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
static const double WAIT_SERVICE_COMPLETION_PERIOD = 2.0;
static const std::string CRS_YAML_ELEMENT = "crs";

namespace parameter_names
{
static const std::string SM_FILE = "state_machine_file";
}

namespace state_names
{
// configuration
namespace general
{
static const std::string INITIALIZATION = "Initialization";
static const std::string CONFIGURATION = "Configuration";
static const std::string WAIT_USER_CMD = "Wait_User_Cmd";
}  // namespace general

// scan acquisition
namespace scan
{
static const std::string MOVE_ROBOT = "SC_Move_Robot";
static const std::string VERIFICATION = "SC_Verification";
static const std::string CAPTURE = "SC_Capture";
static const std::string CHECK_QUEUE = "SC_Check_Queue";
static const std::string SAVE_RESULTS = "SC_Save_Results";
}  // namespace scan

// part registration
namespace part_reg
{
static const std::string COMPUTE_TRANSFORM = "Compute_Transform";
static const std::string PREVIEW = "RG_Preview";
static const std::string APPLY_TRANSFORM = "Apply_transform";
static const std::string SAVE_RESULTS = "RG_Save_Results";
}  // namespace part_reg

// motion planning
namespace motion_planning
{
static const std::string SPLIT_TOOLPATHS = "Split_ToolPaths";
static const std::string PREVIEW = "MP_Preview";
static const std::string PLAN_PROCESS_PATHS = "Plan_Process_Paths";
static const std::string PLAN_MEDIA_CHANGES = "Plan_Media_Changes";
static const std::string SAVE_RESULTS = "MP_Save_Results";
}  // namespace motion_planning

// part rework
namespace part_rework
{
static const std::string RESET = "PR_Reset";
static const std::string MOVE_ROBOT = "PR_Move_Robot";
static const std::string ACQUIRE_SCAN = "PR_Acquire_Scan";
static const std::string CHECK_QUEUE = "PR_Check_Queue";
static const std::string DETECT_REGIONS = "PR_Detect_Regions";
static const std::string USER_REGION_SELECTION = "PR_User_Region_Selection";
static const std::string TRIM_TOOLPATHS = "PR_Trim_Toolpaths";
static const std::string PREVIEW_TOOLPATHS = "PR_Preview_Toolpaths";
static const std::string SAVE_RESULTS = "PR_Save_Results";
}  // namespace part_rework

// process execution
namespace process_exec
{
static const std::string MOVE_START = "Move_Start";
static const std::string EXEC_PROCESS = "Exec_Process";
static const std::string EXEC_MEDIA_CHANGE = "Exec_Media_Change";
static const std::string EXEC_MOVE_RETURN = "Exec_Move_Return";
static const std::string CHECK_QUEUE = "PE_Check_Queue";
static const std::string EXEC_HOME = "Exec_Home";
}  // namespace process_exec
}  // namespace state_names

namespace crs_application
{
CRSExecutive::CRSExecutive(std::shared_ptr<rclcpp::Node> node)
  : node_(node)
  , pnode_(std::make_shared<rclcpp::Node>(std::string(node_->get_name()) + "_exec"))
  , managers_node_(std::make_shared<rclcpp::Node>(std::string(node_->get_name()) + "_tasks"))
{
  if (!setup())
  {
    throw std::runtime_error("Failed initial setup");
  }
}

CRSExecutive::~CRSExecutive() {}
common::ActionResult CRSExecutive::initialize()
{
  if (scan_acqt_mngr_->init() && motion_planning_mngr_->init() && part_regt_mngr_->init() &&
      process_exec_mngr_->init() && part_rework_mngr_->init())
  {
    RCLCPP_INFO(node_->get_logger(), "All managers initialized");
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "One or more managers failed to initialize");
    return false;
  }

  RCLCPP_WARN(node_->get_logger(), "%s not fully implemented yet", __PRETTY_FUNCTION__);
  return true;
}

common::ActionResult CRSExecutive::configure()
{
  using namespace crs_msgs::srv;
  if (!get_config_client_->wait_for_service(std::chrono::duration<double>(5.0)))
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "%s wait for service '%s' timed out",
                 node_->get_name(),
                 get_config_client_->get_service_name());
    return false;
  }

  auto call_config = [&](const crs_msgs::msg::ProcessConfiguration config_msg) -> common::ActionResult {
    process_config_ = config_msg;
    RCLCPP_INFO(node_->get_logger(), "Got Configuration", node_->get_name());

    // parsing
    YAML::Node config = YAML::LoadFile(process_config_.yaml_config);
    if (!config)
    {
      common::ActionResult result;
      result.err_msg = boost::str(boost::format("Invalid yaml file") % process_config_.yaml_config);
      result.succeeded = false;
      RCLCPP_ERROR(node_->get_logger(), "%s", result.err_msg.c_str());
      return result;
    }

    YAML::Node crs_config = config[CRS_YAML_ELEMENT];
    if (!crs_config)
    {
      common::ActionResult result;
      result.err_msg =
          boost::str(boost::format("Did not find the '%s' element in the yaml configuration") % CRS_YAML_ELEMENT);
      result.succeeded = false;
      RCLCPP_ERROR(node_->get_logger(), "%s", result.err_msg.c_str());
      return result;
    }

    return configureManagers(crs_config);
  };

  GetConfiguration::Request::SharedPtr req = std::make_shared<GetConfiguration::Request>();
  std::shared_future<GetConfiguration::Response::SharedPtr> res = get_config_client_->async_send_request(req);
  rclcpp::executor::FutureReturnCode ret_code = rclcpp::spin_until_future_complete(
      pnode_,
      res,
      rclcpp::Duration::from_seconds(WAIT_SERVICE_COMPLETION_PERIOD).to_chrono<std::chrono::nanoseconds>());
  if (ret_code != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    common::ActionResult result;
    result.err_msg = "Call to get config service timed out";
    result.succeeded = false;
    RCLCPP_ERROR_STREAM(node_->get_logger(), result.err_msg);
    return result;
  }

  return call_config(res.get()->config);
}

bool CRSExecutive::configureManagers(YAML::Node& node)
{
  using namespace config;

  std::string err_msg;
  boost::optional<ScanAcquisitionConfig> sc_config = config::parse<ScanAcquisitionConfig>(node, err_msg);

  boost::optional<MotionPlanningConfig> mp_config =
      sc_config ? config::parse<MotionPlanningConfig>(node, err_msg) : boost::none;

  boost::optional<PartRegistrationConfig> pr_config =
      mp_config ? config::parse<PartRegistrationConfig>(node, err_msg) : boost::none;

  boost::optional<ProcessExecutionConfig> pe_config =
      pr_config ? config::parse<ProcessExecutionConfig>(node, err_msg) : boost::none;

  boost::optional<PartReworkConfig> pw_config =
      pe_config ? config::parse<PartReworkConfig>(node, err_msg) : boost::none;

  if (!sc_config || !mp_config || !pr_config || !pe_config || !pw_config)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse configurations from yaml, err msg: %s", err_msg.c_str());
    return false;
  }

  task_mngrs_configured_ =
      scan_acqt_mngr_->configure(sc_config.get()) && motion_planning_mngr_->configure(mp_config.get()) &&
      part_regt_mngr_->configure(pr_config.get()) && process_exec_mngr_->configure(pe_config.get()) &&
      part_rework_mngr_->configure(pw_config.get());
  if (task_mngrs_configured_)
  {
    RCLCPP_INFO(node_->get_logger(), "Task Managers successfully configured");
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Task Managers failed configuration");
  }
  return task_mngrs_configured_;
}

bool CRSExecutive::addStateCallbacks(const std::map<std::string, StateCallbackInfo>& st_callbacks_map)
{
  using namespace crs_application;
  using namespace scxml_core;

  for (const auto& kv : st_callbacks_map)
  {
    // create actual callback
    auto sm_entry_cb = [this, kv](const Action& action) -> Response {
      auto sm_ = this->sm_;
      auto res = kv.second.entry_cb();
      if (!res)
      {
        sm_->postAction(Action{ .id = kv.second.on_failed_action });
        Response sm_res;
        sm_res.success = res.succeeded;
        sm_res.msg = res.err_msg;
        return sm_res;
      }
      if (!kv.second.on_done_action.empty())
      {
        sm_->postAction(Action{ .id = kv.second.on_done_action });
      }
      return true;
    };

    if (!sm_->addEntryCallback(kv.first, sm_entry_cb, kv.second.async))
    {
      return false;
    }

    std::cout << "Added entry cb for state " << kv.first << std::endl;

    if (!kv.second.exit_cb)
    {
      continue;  // no exit callback so go to next
    }

    if (!sm_->addExitCallback(kv.first, [kv]() { kv.second.exit_cb(); }))
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
  const std::vector<ParamInfo> parameters = { std::make_tuple(parameter_names::SM_FILE,
                                                              node_->declare_parameter(parameter_names::SM_FILE)) };

  // check parameters
  for (const auto& p : parameters)
  {
    if (std::get<1>(p).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      RCLCPP_ERROR(node_->get_logger(), "%s failed to read parameter '%s'", node_->get_name(), std::get<0>(p).c_str());
      return false;
    }
  }

  // create connections
  get_config_client_ = pnode_->create_client<crs_msgs::srv::GetConfiguration>(GET_CONFIGURATION_SERVICE,
                                                                              rmw_qos_profile_services_default);

  std::vector<rclcpp::ClientBase*> optional_clients = { get_config_client_.get() };
  for (auto& c : optional_clients)
  {
    if (!c->wait_for_service(std::chrono::duration<double>(WAIT_FOR_SERVICE_PERIOD)))
    {
      RCLCPP_WARN(node_->get_logger(), "%s wait for service '%s' timed out", node_->get_name(), c->get_service_name());
    }
  }

  // create managers
  scan_acqt_mngr_ = std::make_shared<task_managers::ScanAcquisitionManager>(managers_node_);
  motion_planning_mngr_ = std::make_shared<task_managers::MotionPlanningManager>(managers_node_);
  part_regt_mngr_ = std::make_shared<task_managers::PartRegistrationManager>(managers_node_);
  process_exec_mngr_ = std::make_shared<task_managers::ProcessExecutionManager>(managers_node_);
  part_rework_mngr_ = std::make_shared<task_managers::PartReworkManager>(managers_node_);

  std::thread managers_node_thread([this]() {
    while (rclcpp::ok())
    {
      rclcpp::spin_some(managers_node_);
    }
  });

  managers_node_thread.detach();

  // create state machine
  std::string state_machine_file = std::get<1>(parameters[0]).get<std::string>();
  sm_ = std::make_shared<scxml_core::StateMachine>();
  if (!sm_->loadFile(state_machine_file))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load state machine file %s", state_machine_file.c_str());
    return -1;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded SM file %s", state_machine_file.c_str());

  return setupGeneralStates() && setupScanAcquisitionStates() && setupPartRegistrationStates() &&
         setupMotionPlanningStates() && setupProcessExecStates() && setupPartReworkStates();
}

bool CRSExecutive::setupGeneralStates()
{
  using namespace state_names;
  using namespace scxml_core;

  std::map<std::string, StateCallbackInfo> st_callbacks_map;

  st_callbacks_map[general::INITIALIZATION] =
  StateCallbackInfo{ entry_cb : std::bind(&CRSExecutive::initialize, this), async : false };

  st_callbacks_map[general::CONFIGURATION] =
  StateCallbackInfo{ entry_cb : std::bind(&CRSExecutive::configure, this), async : false };

  st_callbacks_map[general::WAIT_USER_CMD] = StateCallbackInfo{
    entry_cb : [this]() -> common::ActionResult {
      if (!task_mngrs_configured_)
      {
        RCLCPP_ERROR(node_->get_logger(), "Task Managers have not been configured");
        sm_->postAction(Action{ .id = action_names::SM_FAILURE });
      }
      return true;
    },
    async : false,
    exit_cb : nullptr,
    on_done_action : ""
  };

  // now adding functions to SM
  return addStateCallbacks(st_callbacks_map);
}

bool CRSExecutive::setupMotionPlanningStates()
{
  using namespace state_names;
  using namespace scxml_core;

  std::map<std::string, StateCallbackInfo> st_callbacks_map;

  st_callbacks_map[motion_planning::SPLIT_TOOLPATHS] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::MotionPlanningManager::splitToolpaths, motion_planning_mngr_.get()),
    async : false
  };

  st_callbacks_map[motion_planning::PLAN_PROCESS_PATHS] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::MotionPlanningManager::planProcessPaths, motion_planning_mngr_.get()),
    async : true
  };

  st_callbacks_map[motion_planning::PLAN_MEDIA_CHANGES] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::MotionPlanningManager::planMediaChanges, motion_planning_mngr_.get()),
    async : true
  };

  st_callbacks_map[motion_planning::PREVIEW] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::MotionPlanningManager::showPreview, motion_planning_mngr_.get()),
    async : true,
    exit_cb : std::bind(&task_managers::MotionPlanningManager::hidePreview, motion_planning_mngr_.get()),
    on_done_action : ""
  };

  st_callbacks_map[motion_planning::SAVE_RESULTS] = StateCallbackInfo{
    entry_cb :
        [this]() -> common::ActionResult { return process_exec_mngr_->setInput(motion_planning_mngr_->getResult()); },
    async : false
  };

  // now adding functions to SM
  return addStateCallbacks(st_callbacks_map);
}

bool CRSExecutive::setupPartRegistrationStates()
{
  using namespace state_names;
  using namespace scxml_core;

  std::map<std::string, StateCallbackInfo> st_callbacks_map;

  st_callbacks_map[part_reg::COMPUTE_TRANSFORM] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::PartRegistrationManager::computeTransform, part_regt_mngr_.get()),
    async : true
  };

  st_callbacks_map[part_reg::APPLY_TRANSFORM] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::PartRegistrationManager::applyTransform, part_regt_mngr_.get()),
    async : false
  };

  st_callbacks_map[part_reg::PREVIEW] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::PartRegistrationManager::showPreview, part_regt_mngr_.get()),
    async : false,
    exit_cb : nullptr,
    on_done_action : ""
  };

  st_callbacks_map[part_reg::SAVE_RESULTS] = StateCallbackInfo{
    entry_cb : [this]() -> common::ActionResult {
      common::ActionResult res = motion_planning_mngr_->setInput({part_regt_mngr_->getResult()});
      if (!res)
      {
        return res;
      }
      res = part_rework_mngr_->setInput(part_regt_mngr_->getResult());
      if (!res)
      {
        return res;
      }
      return true;
    },
    async : false
  };

  // now adding functions to SM
  return addStateCallbacks(st_callbacks_map);
}

bool CRSExecutive::setupProcessExecStates()
{
  using namespace state_names;
  using namespace scxml_core;

  std::map<std::string, StateCallbackInfo> st_callbacks_map;

  st_callbacks_map[process_exec::MOVE_START] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ProcessExecutionManager::moveStart, process_exec_mngr_.get()),
    async : true
  };

  st_callbacks_map[process_exec::EXEC_PROCESS] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ProcessExecutionManager::execProcess, process_exec_mngr_.get()),
    async : true
  };

  st_callbacks_map[process_exec::EXEC_MEDIA_CHANGE] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ProcessExecutionManager::execMediaChange, process_exec_mngr_.get()),
    async : true
  };

  st_callbacks_map[process_exec::EXEC_MOVE_RETURN] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ProcessExecutionManager::execMoveReturn, process_exec_mngr_.get()),
    async : true
  };

  st_callbacks_map[process_exec::EXEC_HOME] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ProcessExecutionManager::execMediaChange, process_exec_mngr_.get()),
    async : false
  };

  // custom callback to handle process checkQueue() response with multiple options
  auto check_queue_cb = [this](const Action& action) -> scxml_core::Response {
    scxml_core::Response sm_res;
    crs_application::common::ActionResult res = process_exec_mngr_->checkQueue();
    if (!res)
    {
      sm_res.success = false;
      sm_res.msg = res.err_msg;
      sm_->postAction(Action{ .id = action_names::SM_FAILURE });
      return sm_res;
    }

    datatypes::ProcessExecActions proc_action = boost::any_cast<datatypes::ProcessExecActions>(res.opt_data);
    switch (proc_action)
    {
      case datatypes::ProcessExecActions::EXEC_MEDIA_CHANGE:
        sm_->postAction(Action{ .id = action_names::SM_EXEC_MDCH });
        break;
      case datatypes::ProcessExecActions::EXEC_PROCESS:
        sm_->postAction(Action{ .id = action_names::SM_EXEC_PROC });
        break;
      case datatypes::ProcessExecActions::DONE:
        sm_->postAction(Action{ .id = action_names::SM_DONE });
        break;
      default:
        sm_->postAction(Action{ .id = action_names::SM_FAILURE });
        sm_res.success = false;
        sm_res.msg = boost::str(boost::format("process action %i not recognized") % static_cast<int>(proc_action));
        return sm_res;
    }
  };

  if (!sm_->addEntryCallback(process_exec::CHECK_QUEUE, check_queue_cb, false))
  {
    return false;
  }

  // now adding functions to SM
  return addStateCallbacks(st_callbacks_map);
}

bool CRSExecutive::setupScanAcquisitionStates()
{
  using namespace state_names;
  using namespace scxml_core;

  std::map<std::string, StateCallbackInfo> st_callbacks_map;

  st_callbacks_map[scan::CAPTURE] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ScanAcquisitionManager::capture, scan_acqt_mngr_.get()),
    async : false
  };

  st_callbacks_map[scan::MOVE_ROBOT] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ScanAcquisitionManager::moveRobot, scan_acqt_mngr_.get()),
    async : true
  };

  st_callbacks_map[scan::VERIFICATION] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ScanAcquisitionManager::verify, scan_acqt_mngr_.get()),
    async : false
  };

  st_callbacks_map[scan::CHECK_QUEUE] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::ScanAcquisitionManager::checkQueue, scan_acqt_mngr_.get()),
    async : false,
    exit_cb : nullptr,
    on_done_action : action_names::SM_DONE,
    on_failed_action : action_names::SM_NEXT
  };

  st_callbacks_map[scan::SAVE_RESULTS] = StateCallbackInfo{
    entry_cb : [this]() -> common::ActionResult { return part_regt_mngr_->setInput(scan_acqt_mngr_->getResult()); },
    async : false
  };

  // now adding functions to SM
  return addStateCallbacks(st_callbacks_map);
}

bool CRSExecutive::setupPartReworkStates()
{
  using namespace state_names;
  using namespace scxml_core;

  std::map<std::string, StateCallbackInfo> st_callbacks_map;

  st_callbacks_map[part_rework::RESET] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::PartReworkManager::reset, part_rework_mngr_.get()),
    async : true,
    exit_cb : nullptr,
    on_done_action : action_names::SM_DONE,
  };

  st_callbacks_map[part_rework::MOVE_ROBOT] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::PartReworkManager::moveRobot, part_rework_mngr_.get()),
    async : true
  };

  st_callbacks_map[part_rework::ACQUIRE_SCAN] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::PartReworkManager::acquireScan, part_rework_mngr_.get()),
    async : true
  };

  st_callbacks_map[part_rework::CHECK_QUEUE] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::PartReworkManager::doneScanning, part_rework_mngr_.get()),
    async : false,
    exit_cb : nullptr,
    on_done_action : action_names::SM_DONE,
    on_failed_action : action_names::SM_NEXT
  };

  st_callbacks_map[part_rework::DETECT_REGIONS] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::PartReworkManager::detectRegions, part_rework_mngr_.get()),
    async : true,
  };

  st_callbacks_map[part_rework::USER_REGION_SELECTION] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::PartReworkManager::showRegions, part_rework_mngr_.get()),
    async : false,
    exit_cb : nullptr,
    on_done_action : "",
    on_failed_action : action_names::SM_FAILURE
  };

  st_callbacks_map[part_rework::TRIM_TOOLPATHS] = StateCallbackInfo{
    entry_cb : [this]() -> common::ActionResult {
      common::ActionResult res = part_rework_mngr_->setInput(part_regt_mngr_->getResult());
      if (!res)
      {
        return res;
      }
      return part_rework_mngr_->trimToolpaths();
    },
    async : false
  };

  st_callbacks_map[part_rework::PREVIEW_TOOLPATHS] = StateCallbackInfo{
    entry_cb : std::bind(&task_managers::PartReworkManager::showPreview, part_rework_mngr_.get()),
    async : false,
    exit_cb : std::bind(&task_managers::PartReworkManager::hidePreview, part_rework_mngr_.get()),
    on_done_action : ""
  };

  st_callbacks_map[part_rework::SAVE_RESULTS] = StateCallbackInfo{
    entry_cb :
        [this]() -> common::ActionResult { return motion_planning_mngr_->setInput(part_rework_mngr_->getResult()); },
    async : false
  };

  // now adding functions to SM
  return addStateCallbacks(st_callbacks_map);
}
}  // namespace crs_application
