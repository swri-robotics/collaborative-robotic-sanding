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

#include <crs_gui/widgets/crs_application_widget.h>

#include <crs_gui/widgets/part_selection_widget.h>
#include <crs_gui/widgets/polygon_area_selection_widget.h>
#include <crs_gui/widgets/state_machine_interface_widget.h>

#include <crs_motion_planning/path_processing_utils.h>

const static std::string CURRENT_STATE_TOPIC = "current_state";
const static std::string GET_AVAILABLE_ACTIONS = "get_available_actions";
const static std::string GET_CONFIGURATION_SERVICE = "get_configuration";
const static std::string EXECUTE_ACTION = "execute_action";
const static std::string MESH_MARKER_TOPIC = "mesh_marker";
const static std::string TOOLPATH_MARKER_TOPIC = "toolpath_marker";
const static double WAIT_FOR_SERVICE_PERIOD = 1.0;

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
  , area_selection_widget_(new PolygonAreaSelectionWidget(node, "world", "world", parent))
  , state_machine_interface_widget_(new StateMachineInterfaceWidget(node, parent))
{
  ui_->setupUi(this);

  // Set up ROS Interfaces to crs_application
  auto get_configuration_cb =
      std::bind(&CRSApplicationWidget::getConfigurationCb, this, std::placeholders::_1, std::placeholders::_2);
  get_configuration_srv_ =
      node_->create_service<crs_msgs::srv::GetConfiguration>(GET_CONFIGURATION_SERVICE, get_configuration_cb);

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
  std::cout << "Selected Part: " << selected_part << std::endl;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";

  std::string path = "file://" + database_directory_ + "/" + selected_part + "/" + selected_part + ".ply";
  marker.mesh_resource = path;
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

  std::string path2 = database_directory_ + "/" + selected_part + "/" + selected_part + ".ply";
  area_selection_widget_->loadMeshFile(path2);

  // Clear the old toolpath
  current_toolpath_marker_ = delete_all_marker_;
  toolpath_marker_pub_->publish(delete_all_marker_);
}

void CRSApplicationWidget::onPartPathSelected(const std::string selected_part, const std::string selected_path)
{
  // Read toolpath yamls
  std::string path = database_directory_ + "/" + selected_part + "/" + selected_path + ".yaml";
  std::cout << "Loading Toolpath: " << path << std::endl;
  std::vector<geometry_msgs::msg::PoseArray> rasters;
  crs_motion_planning::parsePathFromFile(path, "world", rasters);

  // Convert to markers
  visualization_msgs::msg::MarkerArray raster_markers;
  crs_motion_planning::rasterStripsToMarkerArray(rasters, "world", raster_markers);
  current_toolpath_marker_.markers.clear();
  current_toolpath_marker_ = raster_markers;

  // Clear the old toolpath
  toolpath_marker_pub_->publish(delete_all_marker_);
}

void CRSApplicationWidget::getConfigurationCb(crs_msgs::srv::GetConfiguration::Request::SharedPtr req,
                                              crs_msgs::srv::GetConfiguration::Response::SharedPtr res)
{
  // TODO: Write part and send response.
  res->config.speed = 0.2;
  //  res->config
  res->success = true;
}

void CRSApplicationWidget::meshMarkerTimerCb() { mesh_marker_pub_->publish(current_mesh_marker_); }

void CRSApplicationWidget::toolpathMarkerTimerCb() { toolpath_marker_pub_->publish(current_toolpath_marker_); }
}  // namespace crs_gui
