/*
 * Copyright 2020 Southwest Research Institute
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

#ifndef CRS_GUI_WIDGETS_CRS_APPLICATION_WIDGET_H
#define CRS_GUI_WIDGETS_CRS_APPLICATION_WIDGET_H

#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <crs_msgs/srv/get_available_actions.hpp>
#include <crs_msgs/srv/execute_action.hpp>
#include <crs_msgs/srv/get_configuration.hpp>
#include <string>

namespace Ui
{
class CRSApplication;
}

namespace crs_gui
{
class PartSelectionWidget;
class PolygonAreaSelectionWidget;
class StateMachineInterfaceWidget;

class CRSApplicationWidget : public QWidget
{
  Q_OBJECT
public:
  CRSApplicationWidget(rclcpp::Node::SharedPtr node,
                       QWidget* parent = nullptr,
                       std::string database_directory = std::string(std::getenv("HOME")) + "/.local/share/"
                                                                                           "offline_generated_paths");

protected Q_SLOTS:

  void onPartSelected(const std::string);

  void onPartPathSelected(const std::string, const std::string);

protected:
  Ui::CRSApplication* ui_;

  rclcpp::Node::SharedPtr node_;
  std::string database_directory_;

  /** @brief Service that provides process Config when called */
  rclcpp::Service<crs_msgs::srv::GetConfiguration>::SharedPtr get_configuration_srv_;
  /** @brief Callback for get_configuration_srv_*/
  void getConfigurationCb(crs_msgs::srv::GetConfiguration::Request::SharedPtr req,
                          crs_msgs::srv::GetConfiguration::Response::SharedPtr res);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mesh_marker_pub_;
  visualization_msgs::msg::MarkerArray current_mesh_marker_;
  rclcpp::TimerBase::SharedPtr mesh_marker_timer_;
  void meshMarkerTimerCb();

  PartSelectionWidget* part_selector_widget_;
  PolygonAreaSelectionWidget* area_selection_widget_;
  StateMachineInterfaceWidget* state_machine_interface_widget_;
};

}  // namespace crs_gui

#endif  // crs_GUI_WIDGETS_APPLICATION_WIDGET_BASE_H
