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

#include "crs_gui/widgets/polygon_area_selection_widget.h"

#include <QMessageBox>
#include <QPushButton>

#include <std_srvs/srv/trigger.h>

#include <ui_polygon_area_selection_widget.h>

namespace crs_gui
{
PolygonAreaSelectionWidget::PolygonAreaSelectionWidget(rclcpp::Node::SharedPtr node,
                                                       const std::string& selection_world_frame,
                                                       const std::string& selection_sensor_frame,
                                                       QWidget* parent)
  : QWidget(parent)
  , ui_(new Ui::PolygonAreaSelectionWidget)
  , node_(node)
  , selector_(node, selection_world_frame, selection_sensor_frame)
{
  ui_->setupUi(this);
  connect(
      ui_->push_button_clear_selection, &QPushButton::clicked, this, &PolygonAreaSelectionWidget::clearROISelection);
  connect(ui_->push_button_apply_selection, &QPushButton::clicked, this, &PolygonAreaSelectionWidget::applySelection);
}

PolygonAreaSelectionWidget::~PolygonAreaSelectionWidget() { delete ui_; }

void PolygonAreaSelectionWidget::init(const shape_msgs::msg::Mesh& mesh)
{
  mesh_.reset(new shape_msgs::msg::Mesh(mesh));
  clearROISelection();
  return;
}

void PolygonAreaSelectionWidget::clearROISelection()
{
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto res = std::make_shared<std_srvs::srv::Trigger::Response>();
  // Currently, this callback is being called directly, so it will always return true.
  selector_.clearROIPointsCb(req, res);
  if (!res->success)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        "Tool Path Parameter Editor Widget: Area Selection error:" << res->message);
  }
  submesh_.reset(new shape_msgs::msg::Mesh(*mesh_));

  emit(selectedSubmesh(submesh_));
  return;
}

void PolygonAreaSelectionWidget::applySelection()
{
  if (!mesh_)
  {
    QMessageBox::warning(this, "Tool Path Planning Error", "No mesh available to crop");
    return;
  }
  submesh_.reset(new shape_msgs::msg::Mesh());
  std::string error_message;
  bool success = selector_.collectROIMesh(*mesh_, *submesh_, error_message);
  if (!success)
  {
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "Tool Path Parameter Editor Widget: Area Selection error: could not compute submesh: " << error_message);
  }
  if (submesh_->vertices.size() < 3 || submesh_->triangles.size() < 1)
  {
    submesh_.reset(new shape_msgs::msg::Mesh(*mesh_));
  }

  emit(selectedSubmesh(submesh_));
  return;
}

}  // namespace crs_gui
