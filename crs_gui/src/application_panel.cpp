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

#include <crs_gui/application_panel.h>
#include <crs_gui/widgets/crs_application_widget.h>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrent>

namespace crs_gui
{
ApplicationPanel::ApplicationPanel(QWidget* parent)
  : rviz_common::Panel(parent), node_(new rclcpp::Node("application_panel_node"))
{
  application_widget_.reset(new CRSApplicationWidget(node_, this));
  auto nodes = application_widget_->getNodes();
  for (auto& n : nodes)
  {
    executor_.add_node(n);
  }
  QtConcurrent::run([this]() { executor_.spin(); });
}

void ApplicationPanel::onInitialize()
{
  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(application_widget_.get());

  setLayout(layout);
}
}  // namespace crs_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(crs_gui::ApplicationPanel, rviz_common::Panel)
