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

#ifndef CRS_GUI_APPLICATION_PANEL_H
#define CRS_GUI_APPLICATION_PANEL_H

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QTimer>

namespace crs_gui
{
class CRSApplicationWidget;

/**
 * @brief Simple RViz panel that wraps the application widget such that can easily
 * be used within the context of RViz
 */
class ApplicationPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  ApplicationPanel(QWidget* parent = nullptr);

  virtual void onInitialize() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::MultiThreadedExecutor executor_;

  std::shared_ptr<CRSApplicationWidget> application_widget_;
};
}  // namespace crs_gui

#endif
