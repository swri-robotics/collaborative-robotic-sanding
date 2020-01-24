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

#include <QApplication>

#include <rclcpp/rclcpp.hpp>

#include "crs_gui/widgets/polygon_area_selection_widget.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("area_selection_widget_demo_node");

  // Create and start the Qt application
  QApplication app(argc, argv);

  auto widget = std::make_unique<crs_gui::PolygonAreaSelectionWidget>(node,"world", "sensor_frame");
//  crs_gui::PolygonAreaSelectionWidget* widget = new crs_gui::PolygonAreaSelectionWidget(node,"world", "sensor_frame");
  widget->show();

  app.exec();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
