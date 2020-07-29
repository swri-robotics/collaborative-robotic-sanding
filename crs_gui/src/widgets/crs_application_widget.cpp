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
#include <QtConcurrent/QtConcurrentRun>
#include <QFuture>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/emitter.h>

#include <chrono>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>

#include <crs_gui/widgets/crs_application_widget.h>

#include <crs_gui/widgets/part_selection_widget.h>
#include <crs_gui/widgets/polygon_area_selection_widget.h>
#include <crs_gui/widgets/state_machine_interface_widget.h>

#include <crs_motion_planning/path_processing_utils.h>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

// general
const static std::string GET_CONFIGURATION_SERVICE = "get_configuration";
const static std::string MESH_MARKER_TOPIC = "mesh_marker";
const static std::string TOOLPATH_MARKER_TOPIC = "toolpath_marker";
static const std::string TOOLPATH_FILE_EXT = ".yaml";
static const std::string CONFIG_FILES_DIR = "configs";
static const std::string CONFIG_FILE_SUFFIX = "_config.yaml";
static const std::string DEFAULT_CONFIG_FILE = "crs.yaml";  // in part directory
static const std::string WORLD_FRAME = "world";
static const std::string CACHE_DIR = std::string(getenv("HOME")) + std::string("/.crs");
static const std::string TEMP_CONFIG_FILE_NAME = "crs.yaml";

// configuration variables
static const std::vector<std::string> PART_SELECTION_STATES = { "Ready::Wait_Config", "Ready::Wait_User_Cmd" };
static const std::string CONFIG_ROOT_ITEM = "crs";
static const std::vector<std::string> CONFIG_REQUIRED_FIELDS = { "general",
                                                                 "motion_planning",
                                                                 "scan_acquisition",
                                                                 "process_execution",
                                                                 "part_registration" };
static const std::vector<std::string> CONFIG_PART_REG_FIELDS = { "part_file", "toolpath_file" };

namespace crs_gui
{
CRSApplicationWidget::CRSApplicationWidget(rclcpp::Node::SharedPtr node, QWidget* parent)
  : QWidget(parent)
  , ui_(new Ui::CRSApplication)
  , node_(node)
  , support_widgets_node_(std::make_shared<rclcpp::Node>("support_widget"))
  , area_selection_widget_(new PolygonAreaSelectionWidget(support_widgets_node_, WORLD_FRAME, WORLD_FRAME))
  , state_machine_interface_widget_(new StateMachineInterfaceWidget(support_widgets_node_))
{
  namespace fs = boost::filesystem;
  ui_->setupUi(this);

  // Set up ROS Interfaces to crs_application
  auto get_configuration_cb =
      std::bind(&CRSApplicationWidget::getConfigurationCb, this, std::placeholders::_1, std::placeholders::_2);
  get_configuration_srv_ =
      node_->create_service<crs_msgs::srv::GetConfiguration>(GET_CONFIGURATION_SERVICE, get_configuration_cb);

  // creating cache directory
  if (!fs::exists(fs::path(CACHE_DIR)))
  {
    if (!fs::create_directories(fs::path(CACHE_DIR)))
    {
      std::string err_msg = boost::str(boost::format("Failed to create cached directory in location %s") % CACHE_DIR);
      throw std::runtime_error(err_msg);
    }
    RCLCPP_WARN(node_->get_logger(), "Created cached directory %s", CACHE_DIR.c_str());
  }
  RCLCPP_INFO(node_->get_logger(), "Found cached directory %s", CACHE_DIR.c_str());

  // load parameters
  const std::vector<std::string> parameter_names = { "default_config_file", "database_dir" };
  if (std::any_of(parameter_names.begin(), parameter_names.end(), [this](const std::string& p) {
        rclcpp::ParameterValue pv = node_->declare_parameter(p);
        if (pv.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
        {
          RCLCPP_ERROR(node_->get_logger(), "Failed to find parameter %s", p.c_str());
          return true;
        }
        return false;
      }))
  {
    throw std::runtime_error("Failed to load parameter");
  }
  default_config_path_ = node_->get_parameter(parameter_names[0]).as_string();
  database_directory_ = node_->get_parameter(parameter_names[1]).as_string();

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

  // create part selection widget
  part_selector_widget_.reset(new PartSelectionWidget(parent, database_directory_));

  //  // Add the widgets to the UI
  ui_->vertical_layout_part_selector->addWidget(part_selector_widget_.get());
  ui_->vertical_layout_area_selection->addWidget(area_selection_widget_.get());
  ui_->vertical_layout_sm_interface->addWidget(state_machine_interface_widget_.get());

  // disable sm gui at startup
  state_machine_interface_widget_->setEnabled(false);

  // Connect signals and slots
  connect(part_selector_widget_.get(),
          &PartSelectionWidget::partSelected,
          this,
          &CRSApplicationWidget::onPartSelected,
          Qt::QueuedConnection);
  connect(part_selector_widget_.get(),
          &PartSelectionWidget::partPathSelected,
          this,
          &CRSApplicationWidget::onPartPathSelected,
          Qt::QueuedConnection);

  connect(state_machine_interface_widget_.get(), &StateMachineInterfaceWidget::onStateChange, [this](std::string st) {
    bool enable_part_select_wd = false;
    if (std::find(PART_SELECTION_STATES.begin(), PART_SELECTION_STATES.end(), st) != PART_SELECTION_STATES.end())
    {
      enable_part_select_wd = true;
    }

    part_selector_widget_->setEnabled(enable_part_select_wd);
  });
}

CRSApplicationWidget::~CRSApplicationWidget() = default;

void CRSApplicationWidget::onPartSelected(const QString selected_part, const QString part_mesh)
{
  RCLCPP_INFO(node_->get_logger(), "Selected Part: %s", selected_part.toStdString().c_str());
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = WORLD_FRAME;

  cad_part_file_ = database_directory_ + "/" + selected_part.toStdString() + "/" + part_mesh.toStdString();
  marker.mesh_resource = "file://" + cad_part_file_;
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

void CRSApplicationWidget::onPartPathSelected(const QString qselected_part, const QString qselected_path)
{
  namespace fs = boost::filesystem;
  std::string selected_part = qselected_part.toStdString();
  std::string selected_path = qselected_path.toStdString();
  RCLCPP_INFO(node_->get_logger(), "Selected Toolpath: %s", selected_path.c_str());

  // Read toolpath yamls
  toolpath_file_ =
      (fs::path(database_directory_) / fs::path(selected_part) / fs::path(selected_path + TOOLPATH_FILE_EXT)).string();
  std::string config_file = (fs::path(database_directory_) / fs::path(selected_part) / fs::path(CONFIG_FILES_DIR) /
                             fs::path(selected_path + CONFIG_FILE_SUFFIX))
                                .string();

  // check existence of config file
  if (!fs::exists(fs::path(config_file)))
  {
    std::vector<fs::path> default_config_files = { fs::path(database_directory_) / fs::path(selected_part) /
                                                       fs::path(DEFAULT_CONFIG_FILE),
                                                   fs::path(default_config_path_) };
    bool default_config_found = false;
    for (fs::path& df : default_config_files)
    {
      if (!fs::exists(df))
      {
        RCLCPP_WARN(node_->get_logger(), "Default config file '%s' was not found", df.c_str());
        continue;
      }

      RCLCPP_WARN(
          node_->get_logger(), "config file '%s' not found, loading default '%s'", config_file.c_str(), df.c_str());
      RCLCPP_WARN(
          node_->get_logger(), "In order to use a path specific config then create one at \"%s\"", config_file.c_str());
      config_file = df.string();
      default_config_found = true;
      break;
    }

    if (!default_config_found)
    {
      RCLCPP_ERROR(node_->get_logger(), "No default config file was not found");
      return;
    }
  }

  if (!fs::exists(fs::path(toolpath_file_)))
  {
    RCLCPP_ERROR(node_->get_logger(), "Toolpath file '%s' was not found", toolpath_file_.c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Loading Toolpath: %s", toolpath_file_.c_str());
  std::vector<geometry_msgs::msg::PoseArray> rasters;
  crs_motion_planning::parsePathFromFile(toolpath_file_, WORLD_FRAME, rasters);

  // loading config
  if (!loadConfig(config_file) || !updateConfig() || !saveConfig())
  {
    return;
  }

  state_machine_interface_widget_->setEnabled(true);

  // Convert to markers
  visualization_msgs::msg::MarkerArray raster_markers;
  crs_motion_planning::rasterStripsToMarkerArray(rasters, WORLD_FRAME, raster_markers);
  current_toolpath_marker_.markers.clear();
  current_toolpath_marker_ = raster_markers;

  // Clear the old toolpath
  toolpath_marker_pub_->publish(delete_all_marker_);

  // calling configuration request
  state_machine_interface_widget_->requestConfiguration();
}

std::vector<rclcpp::Node::SharedPtr> CRSApplicationWidget::getNodes() { return { node_, support_widgets_node_ }; }

void CRSApplicationWidget::getConfigurationCb(crs_msgs::srv::GetConfiguration::Request::SharedPtr req,
                                              crs_msgs::srv::GetConfiguration::Response::SharedPtr res)
{
  if (!updateConfig() || !saveConfig())
  {
    res->success = false;
    return;
  }

  res->config.yaml_config = config_file_path_;
  res->success = true;
  RCLCPP_INFO(node_->get_logger(), "Sent configuration yaml file %s", res->config.yaml_config.c_str());
  return;
}

bool CRSApplicationWidget::loadConfig(const std::string config_file)
{
  namespace fs = boost::filesystem;
  config_yaml_node_.reset();
  YAML::Node config_node = YAML::LoadFile(config_file);
  if (!config_node)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load config file %s", config_file.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded config yaml %s", config_file.c_str());
  std::cout << YAML::Clone(config_node) << std::endl;

  // check fields
  try
  {
    if (!config_node[CONFIG_ROOT_ITEM])
    {
      RCLCPP_ERROR(node_->get_logger(), "Top element %s was not found in the yaml", CONFIG_ROOT_ITEM.c_str());
      return false;
    }

    YAML::Node crs_node = config_node[CONFIG_ROOT_ITEM];
    if (!crs_node ||
        std::any_of(
            CONFIG_REQUIRED_FIELDS.begin(), CONFIG_REQUIRED_FIELDS.end(), [this, &crs_node](const std::string& f) {
              auto node = crs_node[f];
              if (!node)
              {
                RCLCPP_ERROR(node_->get_logger(), "Config file %s is missing required field '%s'", f.c_str());
                return true;
              }
              return false;
            }))
    {
      return false;
    }
  }
  catch (YAML::InvalidNode& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse config file %s with msg: %S", config_file.c_str(), e.what());
  }
  config_yaml_node_ = std::make_shared<YAML::Node>(YAML::Clone(config_node));

  config_file_path_ = (fs::path(CACHE_DIR) / fs::path(TEMP_CONFIG_FILE_NAME)).string();
  return true;
}

bool CRSApplicationWidget::updateConfig()
{
  if (!config_yaml_node_)
  {
    RCLCPP_ERROR(node_->get_logger(), "No config file has been loaded, can not update");
    return false;
  }

  try
  {
    (*config_yaml_node_)[CONFIG_ROOT_ITEM][CONFIG_REQUIRED_FIELDS[4]][CONFIG_PART_REG_FIELDS[0]] = cad_part_file_;
    (*config_yaml_node_)[CONFIG_ROOT_ITEM][CONFIG_REQUIRED_FIELDS[4]][CONFIG_PART_REG_FIELDS[1]] = toolpath_file_;
  }
  catch (YAML::InvalidNode& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to write to config with err msg: %s", e.what());
    return false;
  }
  return true;
}

bool CRSApplicationWidget::saveConfig()
{
  if (!config_yaml_node_)
  {
    RCLCPP_ERROR(node_->get_logger(), "No config file has been loaded, can not save");
    return false;
  }

  std::ofstream config_out(config_file_path_);
  config_out << *config_yaml_node_;
  config_out.close();
  RCLCPP_INFO(node_->get_logger(), "Saved yaml config to %s", config_file_path_.c_str());
  return true;
}

void CRSApplicationWidget::meshMarkerTimerCb() { mesh_marker_pub_->publish(current_mesh_marker_); }
void CRSApplicationWidget::toolpathMarkerTimerCb() { toolpath_marker_pub_->publish(current_toolpath_marker_); }
}  // namespace crs_gui
