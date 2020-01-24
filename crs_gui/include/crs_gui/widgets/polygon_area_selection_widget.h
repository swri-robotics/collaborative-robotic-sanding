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

#ifndef CRS_GUI_WIDGETS_POLYGON_AREA_SELECTION_WIDGET_H
#define CRS_GUI_WIDGETS_POLYGON_AREA_SELECTION_WIDGET_H

#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include <crs_area_selection/selection_artist.h>
#include "crs_gui/register_ros_msgs_for_qt.h"

namespace Ui
{
class PolygonAreaSelectionWidget;
}

namespace crs_gui
{
class PolygonAreaSelectionWidget : public QWidget
{
  Q_OBJECT

public:
  explicit PolygonAreaSelectionWidget(rclcpp::Node::SharedPtr node,
                                      const std::string& selection_world_frame,
                                      const std::string& selection_sensor_frame,
                                      QWidget* parent = nullptr);
  ~PolygonAreaSelectionWidget();

public Q_SLOTS:

  void init(const shape_msgs::msg::Mesh& mesh);

Q_SIGNALS:
  void selectedSubmesh(const shape_msgs::msg::Mesh::SharedPtr& selected_submesh);

private Q_SLOTS:
  void clearROISelection();

  void applySelection();

private:
  Ui::PolygonAreaSelectionWidget* ui_;

  rclcpp::Node::SharedPtr node_;

  shape_msgs::msg::Mesh::SharedPtr mesh_;

  shape_msgs::msg::Mesh::SharedPtr submesh_;

  crs_area_selection::SelectionArtist selector_;
};  // end class PolygonAreaSelectionWidget

}  // namespace crs_gui

#endif  // OPP_GUI_WIDGETS_POLYGON_AREA_SELECTION_WIDGET_H
