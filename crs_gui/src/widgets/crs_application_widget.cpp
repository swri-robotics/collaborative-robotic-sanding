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

const static std::string CURRENT_STATE_TOPIC = "current_state";
const static std::string GET_AVAILABLE_ACTIONS = "get_available_actions";
const static std::string EXECUTE_ACTION = "execute_action";
const static double WAIT_FOR_SERVICE_PERIOD = 1.0;

namespace crs_gui
{
CRSApplicationWidget::CRSApplicationWidget(rclcpp::Node::SharedPtr node,
                                           QWidget* parent,
                                           std::string database_directory)
  : QWidget(parent)
  , ui_(new Ui::CRSApplication)
  , node_(node)
  , part_selector_widget_(new PartSelectionWidget())
  , area_selection_widget_(new PolygonAreaSelectionWidget(node, "", ""))
  , state_machine_interface_widget_(new StateMachineInterfaceWidget(node))
{
  ui_->setupUi(this);

  //  // Add the widgets to the UI
  ui_->vertical_layout_part_selector->addWidget(part_selector_widget_);
  ui_->vertical_layout_area_selection->addWidget(area_selection_widget_);
  ui_->vertical_layout_sm_interface->addWidget(state_machine_interface_widget_);

  connect(part_selector_widget_, &PartSelectionWidget::partSelected, this, &CRSApplicationWidget::onPartSelected);

  // TODO: Connect sm widget and remove ui
}

void CRSApplicationWidget::onPartSelected(const std::string selected_part) { std::cout << selected_part << std::endl; }

}  // namespace crs_gui
