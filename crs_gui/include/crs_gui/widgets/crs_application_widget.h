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

namespace YAML
{
  class Node;
}

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
                       QWidget* parent = nullptr);
  ~CRSApplicationWidget();
protected slots:

  void onPartSelected(const QString selected_part);

  void onPartPathSelected(const QString qselected_part, const QString qselected_path);

  bool loadConfig(const std::string config_file);
  bool updateConfig();
  bool saveConfig();

protected:
  std::unique_ptr<Ui::CRSApplication> ui_;

  rclcpp::Node::SharedPtr node_;
  std::string database_directory_;

  /** @brief Service that provides process Config when called */
  rclcpp::callback_group::CallbackGroup::SharedPtr get_config_callback_group_;
  rclcpp::Service<crs_msgs::srv::GetConfiguration>::SharedPtr get_configuration_srv_;
  /** @brief Callback for get_configuration_srv_*/
  void getConfigurationCb(crs_msgs::srv::GetConfiguration::Request::SharedPtr req,
                          crs_msgs::srv::GetConfiguration::Response::SharedPtr res);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mesh_marker_pub_;
  visualization_msgs::msg::MarkerArray current_mesh_marker_;
  rclcpp::TimerBase::SharedPtr mesh_marker_timer_;
  void meshMarkerTimerCb();

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr toolpath_marker_pub_;
  visualization_msgs::msg::MarkerArray current_toolpath_marker_;
  rclcpp::TimerBase::SharedPtr toolpath_marker_timer_;
  void toolpathMarkerTimerCb();

  std::string default_config_path_;
  std::string config_file_path_;
  std::string toolpath_file_;
  std::string cad_part_file_;
  std::shared_ptr<YAML::Node> config_node_;

  visualization_msgs::msg::MarkerArray delete_all_marker_;

  std::unique_ptr<PartSelectionWidget> part_selector_widget_;
  std::unique_ptr<PolygonAreaSelectionWidget> area_selection_widget_;
  std::unique_ptr<StateMachineInterfaceWidget> state_machine_interface_widget_;
};

}  // namespace crs_gui

#endif  // crs_GUI_WIDGETS_APPLICATION_WIDGET_BASE_H
