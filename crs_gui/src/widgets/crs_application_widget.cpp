/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ui_crs_application.h"

#include <atomic>
#include <QMessageBox>
#include <QStateMachine>
#include <QProgressBar>
#include <QProgressDialog>

#include <chrono>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>

#include <crs_gui/widgets/crs_application_widget.h>

#include <crs_gui/widgets/part_selection_widget.h>
#include <crs_gui/widgets/polygon_area_selection_widget.h>
#include <crs_gui/widgets/state_machine_interface_widget.h>

#include <crs_motion_planning/path_processing_utils.h>

#include <boost/filesystem.hpp>

// general
const static std::string CURRENT_STATE_TOPIC = "current_state";
const static std::string GET_AVAILABLE_ACTIONS = "get_available_actions";
const static std::string GET_CONFIGURATION_SERVICE = "get_configuration";
const static std::string EXECUTE_ACTION = "execute_action";
const static std::string MESH_MARKER_TOPIC = "mesh_marker";
const static std::string TOOLPATH_MARKER_TOPIC = "toolpath_marker";
const static double WAIT_FOR_SERVICE_PERIOD = 1.0;
static const std::string TOOLPATH_FILE_EXT = ".yaml";
static const std::string CONFIG_FILES_DIR = "configs";
static const std::string CONFIG_FILE_SUFFIX = "_config.yaml";
static const std::string WORLD_FRAME = "world";
static const std::string CAD_EXT = ".ply";

// configuration variables
static const std::string CONFIG_ROOT_ITEM = "crs";
static const std::vector<std::string> CONFIG_REQUIRED_FIELDS = {"general", "motion_planning","scan_acquisition",
                                                   "process_execution", "part_registration"};
static const std::vector<std::string> CONFIG_PART_REG_FIELDS = {"part_file", "toolpath_file"};

// logging
static const rclcpp::Logger GUI_LOGGER = rclcpp::get_logger("CRS_GUI");

namespace crs_gui
{
CRSApplicationWidget::CRSApplicationWidget(rclcpp::Node::SharedPtr node,
                                           QWidget* parent,
                                           std::string database_directory)
  : QWidget(parent)
  , ui_(new Ui::CRSApplication)
  , node_(node)
  , database_directory_(database_directory)
  , part_selector_widget_(new PartSelectionWidget(parent, database_directory))
  , area_selection_widget_(new PolygonAreaSelectionWidget(node, WORLD_FRAME, WORLD_FRAME))
  , state_machine_interface_widget_(new StateMachineInterfaceWidget(node))
{
  ui_->setupUi(this);

  // Set up ROS Interfaces to crs_application
  auto get_configuration_cb =
      std::bind(&CRSApplicationWidget::getConfigurationCb, this, std::placeholders::_1, std::placeholders::_2);
  get_configuration_srv_ =
      node_->create_service<crs_msgs::srv::GetConfiguration>(GET_CONFIGURATION_SERVICE, get_configuration_cb);

  // load parameters
  const std::vector<std::string> parameter_names = { "default_config" };
  if(std::any_of(parameter_names.begin(), parameter_names.end(),[this](const std::string& p){
    rclcpp::ParameterValue pv =node_->declare_parameter(p);
    if( pv.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      RCLCPP_ERROR(GUI_LOGGER,"Failed to find parameter %s", p.c_str());
      return true;
    }
    return false;
  }))
  {
    throw std::runtime_error("Failed to load parameter");
  }

  // Set up ROS interfaces for area selection
  mesh_marker_pub_.reset();
  mesh_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(MESH_MARKER_TOPIC, 1);
  toolpath_marker_pub_.reset();
  toolpath_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(TOOLPATH_MARKER_TOPIC, 1);

  // Visualization Setup
  {
    using namespace std::chrono_literals;
    mesh_marker_timer_ = node_->create_wall_timer(500ms, std::bind(&CRSApplicationWidget::meshMarkerTimerCb, this));
    toolpath_marker_timer_ =
        node_->create_wall_timer(500ms, std::bind(&CRSApplicationWidget::toolpathMarkerTimerCb, this));
  }
  visualization_msgs::msg::Marker marker;
  marker.action = marker.DELETEALL;
  delete_all_marker_.markers.push_back(marker);

  //  // Add the widgets to the UI
  ui_->vertical_layout_part_selector->addWidget(part_selector_widget_.get());
  ui_->vertical_layout_area_selection->addWidget(area_selection_widget_.get());
  ui_->vertical_layout_sm_interface->addWidget(state_machine_interface_widget_.get());

  // Connect signals and slots
  connect(part_selector_widget_.get(), &PartSelectionWidget::partSelected, this, &CRSApplicationWidget::onPartSelected);
  connect(part_selector_widget_.get(),
          &PartSelectionWidget::partPathSelected,
          this,
          &CRSApplicationWidget::onPartPathSelected);
}

CRSApplicationWidget::~CRSApplicationWidget() = default;

void CRSApplicationWidget::onPartSelected(const std::string selected_part)
{
  RCLCPP_INFO(GUI_LOGGER, "Selected Part: %s", selected_part.c_str());
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = WORLD_FRAME;

  cad_part_file_ = "file://" + database_directory_ + "/" + selected_part + "/" + selected_part + CAD_EXT;
  marker.mesh_resource = cad_part_file_;
  marker.mesh_use_embedded_materials = true;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.id = 0;
  marker.type = marker.MESH_RESOURCE;
  marker.action = marker.MODIFY;
  visualization_msgs::msg::MarkerArray array;
  array.markers.push_back(marker);
  current_mesh_marker_ = array;
}

void CRSApplicationWidget::onPartPathSelected(const std::string selected_part, const std::string selected_path)
{
  namespace fs = boost::filesystem;

  // Read toolpath yamls
  toolpath_file_ = (fs::path(database_directory_) / fs::path(selected_part) /
      fs::path(selected_path + TOOLPATH_FILE_EXT)).string();
  std::string config_file = (fs::path(database_directory_) / fs::path(selected_part) / fs::path(CONFIG_FILES_DIR) /
      fs::path(selected_path + CONFIG_FILE_SUFFIX)).string();

  // check existence of config file
  if(!fs::exists(fs::path(config_file)))
  {
    if(!fs::exists(fs::path(default_config_path_)))
    {
      RCLCPP_ERROR(GUI_LOGGER,"Default config file '%s' was not found",default_config_path_.c_str());
      return;
    }

    RCLCPP_WARN(GUI_LOGGER,"config file '%s' not found, copying default '%s'", config_file.c_str(),
                default_config_path_.c_str());
    fs::create_directories(fs::path(config_file).parent_path());
    fs::copy(fs::path(default_config_path_), fs::path(config_file));
  }

  RCLCPP_INFO(GUI_LOGGER,"Loading Toolpath: %s", toolpath_file_.c_str());
  std::vector<geometry_msgs::msg::PoseArray> rasters;
  crs_motion_planning::parsePathFromFile(toolpath_file_, WORLD_FRAME, rasters);

  // loading config
  if(!loadConfig(config_file) || !updateConfig() || !saveConfig())
  {
    return;
  }

  // Convert to markers
  visualization_msgs::msg::MarkerArray raster_markers;
  crs_motion_planning::rasterStripsToMarkerArray(rasters, WORLD_FRAME, raster_markers);
  current_toolpath_marker_.markers.clear();
  current_toolpath_marker_ = raster_markers;

  // Clear the old toolpath
  toolpath_marker_pub_->publish(delete_all_marker_);
}

void CRSApplicationWidget::getConfigurationCb(crs_msgs::srv::GetConfiguration::Request::SharedPtr req,
                                              crs_msgs::srv::GetConfiguration::Response::SharedPtr res)
{
  if(!updateConfig() || !saveConfig())
  {
    res->success = false;
    return;
  }

  res->config.yaml_config = config_file_path_;
  res->success = true;
}

bool CRSApplicationWidget::loadConfig(const std::string& config_file)
{
  config_node_ = YAML::LoadFile(config_file);
  if(!config_node_)
  {
    RCLCPP_ERROR(GUI_LOGGER,"Failed to load config file %s", config_file);
    return false;
  }

  // check fields
  YAML::Node crs_node = config_node_[CONFIG_ROOT_ITEM];
  if(!crs_node || std::any_of(CONFIG_REQUIRED_FIELDS.begin(), CONFIG_REQUIRED_FIELDS.end(), [&crs_node] (const std::string& f){
    if(crs_node[f].IsNull())
    {
      RCLCPP_ERROR(GUI_LOGGER,"Config file %s is missing required field '%s'", f.c_str());
      return true;
    }
    return false;
  }))
  {
    return false;
  }
  config_file_path_ = config_file;
  return true;
}

bool CRSApplicationWidget::updateConfig()
{
  if(!config_node_)
  {
    RCLCPP_ERROR(GUI_LOGGER,"No config file has been loaded, can not update");
    return false;
  }

  try
  {
    config_node_[CONFIG_ROOT_ITEM][CONFIG_REQUIRED_FIELDS[4]][CONFIG_PART_REG_FIELDS[0]] = cad_part_file_;
    config_node_[CONFIG_ROOT_ITEM][CONFIG_REQUIRED_FIELDS[4]][CONFIG_PART_REG_FIELDS[1]] = toolpath_file_;
  }
  catch(YAML::InvalidNode& e)
  {
    RCLCPP_ERROR(GUI_LOGGER,"Failed to write to config with err msg: %s", e.what());
    return false;
  }
  return true;
}

bool CRSApplicationWidget::saveConfig()
{
  if(!config_node_)
  {
    RCLCPP_ERROR(GUI_LOGGER,"No config file has been loaded, can not save");
    return false;
  }

  std::ofstream config_out(config_file_path_);
  config_out << config_node_;
  config_out.close();
  return true;
}

void CRSApplicationWidget::meshMarkerTimerCb() { mesh_marker_pub_->publish(current_mesh_marker_); }

void CRSApplicationWidget::toolpathMarkerTimerCb() { toolpath_marker_pub_->publish(current_toolpath_marker_); }
}  // namespace crs_gui
