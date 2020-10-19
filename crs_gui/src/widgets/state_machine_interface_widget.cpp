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

#include "ui_state_machine_interface.h"

#include <atomic>
#include <QMessageBox>
#include <QtConcurrent/QtConcurrent>

#include <chrono>

#include <boost/format.hpp>

#include <crs_gui/widgets/state_machine_interface_widget.h>

const static std::string CURRENT_STATE_TOPIC = "current_state";
const static std::string EXECUTE_ACTION = "execute_action";
const static std::string GET_AVAILABLE_ACTIONS = "get_available_actions";
const static std::string USER_APPROVES_ACTION_ID = "user_approves";
const static std::string USER_CONFIGURES_ACTION_ID = "user_configs";
const static std::string USER_CANCELS_ACTION_ID = "user_rejects";
const static double WAIT_FOR_SERVICE_PERIOD = 2.0;

namespace standard_user_actions
{
static const std::string APPROVES = "user_approves";
static const std::string CANCELS = "user_cancels";
}  // namespace standard_user_actions

// TODO: Crashes too frequently, disable popup window for now
void showMsgBox(bool succeeded, std::string msg)
{
  QMessageBox msg_box;
  // msg_box.setWindowModality(Qt::NonModal);
  if (succeeded)
  {
    msg_box.setStandardButtons(QMessageBox::Ok);
    msg_box.setText(QString::fromStdString(msg));
    msg_box.setIcon(QMessageBox::Information);
  }
  else
  {
    msg_box.setStandardButtons(QMessageBox::Ok);
    msg_box.setText(QString::fromStdString(msg));
    msg_box.setIcon(QMessageBox::Critical);
  }
  // msg_box.show();
}

namespace crs_gui
{
StateMachineInterfaceWidget::StateMachineInterfaceWidget(rclcpp::Node::SharedPtr node, QWidget* parent)
  : QWidget(parent), current_state_(""), ui_(new Ui::StateMachineInterface), node_(node)

{
  ui_->setupUi(this);

  // Initialize state machine interfaces
  auto current_state_cb = std::bind(&StateMachineInterfaceWidget::currentStateCB, this, std::placeholders::_1);
  current_state_sub_ = node_->create_subscription<std_msgs::msg::String>(CURRENT_STATE_TOPIC, 1, current_state_cb);
  get_available_actions_client_ = node_->create_client<crs_msgs::srv::GetAvailableActions>(GET_AVAILABLE_ACTIONS);
  execute_action_client_ = node_->create_client<crs_msgs::srv::ExecuteAction>(EXECUTE_ACTION);

  // TODO: Make state machine interaction a standalone widget
  connect(ui_->push_button_sm_apply, &QPushButton::clicked, this, &StateMachineInterfaceWidget::onSMApply);
  connect(ui_->push_button_sm_cancel, &QPushButton::clicked, this, &StateMachineInterfaceWidget::onSMCancel);
  connect(ui_->push_button_sm_approve, &QPushButton::clicked, this, &StateMachineInterfaceWidget::onSMApprove);
  connect(this, &StateMachineInterfaceWidget::show_msg, [](bool b, std::string msg) { showMsgBox(b, msg); });
}

StateMachineInterfaceWidget::~StateMachineInterfaceWidget() = default;

void StateMachineInterfaceWidget::currentStateCB(const std_msgs::msg::String::ConstSharedPtr current_state)
{
  ui_->line_edit_sm_current_state->setText(QString::fromUtf8(current_state.get()->data.c_str()));
  if (current_state.get()->data != current_state_)
  {
    current_state_ = current_state.get()->data;
    RCLCPP_INFO(node_->get_logger(), "State changed to '%s'", current_state_.c_str());
    onSMQuery();
    emit onStateChange(current_state_);
  }
}

// State machine button callbacks
void StateMachineInterfaceWidget::onSMApply()
{
  // Specify service request to get all user actions in current state
  auto request = std::make_shared<crs_msgs::srv::ExecuteAction::Request>();
  request->action_id = ui_->combo_box_sm_available_actions->currentText().toStdString();

  // Wait if service is not available
  if (!execute_action_client_->wait_for_service(std::chrono::duration<double>(WAIT_FOR_SERVICE_PERIOD)))
  {
    const std::string msg =
        boost::str(boost::format("ROS2 Service %s is not available") % execute_action_client_->get_service_name());
    RCLCPP_ERROR_STREAM(node_->get_logger(), msg);
    emit show_msg(false, msg);
    return;
  }
  // Send request
  using ResFut = decltype(execute_action_client_)::element_type::SharedFuture;
  execute_action_client_->async_send_request(request, [this](ResFut future) {
    auto result = future.get();
    if (result.get()->succeeded)
    {
      emit show_msg(true, "Request approved");
    }
    else
    {
      emit show_msg(false, "Request rejected");
    }
  });
}

void StateMachineInterfaceWidget::onSMQuery()
{
  // Specify service request to get all user actions in current state
  auto request = std::make_shared<crs_msgs::srv::GetAvailableActions::Request>();
  request->search_pattern = request->PATTERN_USER_ACTIONS;

  // Wait if service is not available
  if (!get_available_actions_client_->wait_for_service(std::chrono::duration<double>(WAIT_FOR_SERVICE_PERIOD)))
  {
    RCLCPP_ERROR(node_->get_logger(), "service not available");
    return;
  }
  // Send request and wait for result
  using ResFut = decltype(get_available_actions_client_)::element_type::SharedFuture;
  get_available_actions_client_->async_send_request(request, [this](ResFut future) {
    auto result = future.get();
    if (!result.get()->succeeded)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Failed to get actions list in state '%s' with error msg: %s",
                   current_state_.c_str(),
                   result.get()->err_msg.c_str());
      return;
    }

    const std::vector<std::string>& actions_ids = result.get()->action_ids;
    bool buttons_enabled = !actions_ids.empty();
    ui_->push_button_sm_apply->setEnabled(buttons_enabled);
    ui_->combo_box_sm_available_actions->clear();

    // enable/disable std action buttons
    ui_->push_button_sm_approve->setEnabled(
        std::find(actions_ids.begin(), actions_ids.end(), standard_user_actions::APPROVES) != actions_ids.end());
    ui_->push_button_sm_cancel->setEnabled(
        std::find(actions_ids.begin(), actions_ids.end(), standard_user_actions::CANCELS) != actions_ids.end());

    if (actions_ids.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "No available actions in the current state");
      return;
    }

    // Convert ROS msg to QStringList and update combo box
    QStringList available_actions;
    for (std::string action : actions_ids)
      available_actions.push_back(QString::fromUtf8(action.c_str()));
    ui_->combo_box_sm_available_actions->addItems(available_actions);
  });
}

void StateMachineInterfaceWidget::onSMCancel()
{
  // Specify service request to get all user actions in current state
  auto request = std::make_shared<crs_msgs::srv::ExecuteAction::Request>();
  request->action_id = USER_CANCELS_ACTION_ID;

  // Wait if service is not available
  if (!execute_action_client_->wait_for_service(std::chrono::duration<double>(WAIT_FOR_SERVICE_PERIOD)))
  {
    RCLCPP_ERROR(node_->get_logger(), "service not available");
    return;
  }
  // Send request
  using ResFut = decltype(execute_action_client_)::element_type::SharedFuture;
  execute_action_client_->async_send_request(request, [this](ResFut) {});
}

void StateMachineInterfaceWidget::onSMApprove()
{
  // Specify service request to get all user actions in current state
  auto request = std::make_shared<crs_msgs::srv::ExecuteAction::Request>();
  request->action_id = USER_APPROVES_ACTION_ID;

  // Wait if service is not available
  if (!execute_action_client_->wait_for_service(std::chrono::duration<double>(WAIT_FOR_SERVICE_PERIOD)))
  {
    const std::string msg =
        boost::str(boost::format("ROS2 Service %s is not available") % execute_action_client_->get_service_name());
    RCLCPP_ERROR_STREAM(node_->get_logger(), msg);
    emit show_msg(false, msg);
    return;
  }
  // Send request
  using ResFut = decltype(execute_action_client_)::element_type::SharedFuture;
  execute_action_client_->async_send_request(request, [this](ResFut future) {
    auto result = future.get();
    if (result.get()->succeeded)
    {
      emit show_msg(true, "Request approved");
    }
    else
    {
      emit show_msg(false, "Request rejected");
    }
  });
}

void StateMachineInterfaceWidget::requestConfiguration()
{
  // Specify service request to get all user actions in current state
  auto request = std::make_shared<crs_msgs::srv::ExecuteAction::Request>();
  request->action_id = USER_CONFIGURES_ACTION_ID;

  // Wait if service is not available
  if (!execute_action_client_->wait_for_service(std::chrono::duration<double>(WAIT_FOR_SERVICE_PERIOD)))
  {
    const std::string msg =
        boost::str(boost::format("ROS2 Service %s is not available") % execute_action_client_->get_service_name());
    RCLCPP_ERROR_STREAM(node_->get_logger(), msg);
    emit show_msg(false, msg);
    return;
  }

  // Send request
  using ResFut = decltype(execute_action_client_)::element_type::SharedFuture;
  execute_action_client_->async_send_request(request, [this](ResFut future) {
    auto result = future.get();
    if (result.get()->succeeded)
    {
      emit show_msg(true, "Configuration Request approved");
    }
    else
    {
      emit show_msg(false, "Configuration Request rejected");
    }
  });
}

}  // namespace crs_gui
