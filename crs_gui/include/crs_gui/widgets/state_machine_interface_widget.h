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

#ifndef CRS_GUI_WIDGETS_STATE_MACHINE_INTERFACE_WIDGET_H
#define CRS_GUI_WIDGETS_STATE_MACHINE_INTERFACE_WIDGET_H

#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <crs_msgs/srv/get_available_actions.hpp>
#include <crs_msgs/srv/execute_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace Ui
{
class StateMachineInterface;
}

namespace crs_gui
{
class StateMachineInterfaceWidget : public QWidget
{
  Q_OBJECT
public:
  StateMachineInterfaceWidget(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);
  ~StateMachineInterfaceWidget();
Q_SIGNALS:
  /** @brief emitted inside currentStateCB when the state changes with the name of new state as the argument*/
  void onStateChange(std::string);

protected Q_SLOTS:

  /** @brief Calls ROS service to check available sm actions*/
  void onSMQuery();
  /** @brief Applies selected sm action via ROS service*/
  void onSMApply();
  /** @brief Applies user_approves action via ROS service*/
  void onSMApprove();
  /** @brief Applies user_cancels action via ROS service*/
  void onSMCancel();

public:
  /** @brief Current state machine state */
  std::string current_state_;

protected:
  std::unique_ptr<Ui::StateMachineInterface> ui_;

  /** @brief Node that is used for service calls and subscriber*/
  rclcpp::Node::SharedPtr node_;
  /** @brief Subscriber to topic publishing the current sm state*/
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_state_sub_;
  /** @brief Current state subscriber callback*/
  void currentStateCB(const std_msgs::msg::String::ConstSharedPtr current_state);
  /** @brief ROS client to get available sm actions*/
  rclcpp::Client<crs_msgs::srv::GetAvailableActions>::SharedPtr get_available_actions_client_;
  /** @brief ROS client to execute sm action*/
  rclcpp::Client<crs_msgs::srv::ExecuteAction>::SharedPtr execute_action_client_;
};

}  // namespace crs_gui

#endif  // crs_GUI_WIDGETS_APPLICATION_WIDGET_BASE_H
