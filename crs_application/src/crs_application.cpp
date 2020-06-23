/*
 * @author Jorge Nicho
 * @file crs_application.cpp
 * @date Jan 20, 2020
 * @copyright Copyright (c) 2020, Southwest Research Institute
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <regex>

#include <QApplication>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <crs_msgs/srv/execute_action.hpp>
#include <crs_msgs/srv/get_available_actions.hpp>

#include "crs_application/crs_executive.h"

static const double STATE_PUB_RATE = 0.5;
static const double ROS_SPIN_TIMEOUT = 0.1;
static const double WAIT_SM_BUSY_TIMEOUT = 2.0;
static const std::string NODE_NAME = "crs_application";
static const std::string CURRENT_ST_TOPIC = "current_state";
static const std::string EXECUTE_ACTION_SERVICE = "execute_action";
static const std::string GET_AVAILABLE_ACTIONS_SERVICE = "get_available_actions";

int main(int argc, char** argv)
{
  using namespace crs_application;
  using namespace crs_msgs;
  using namespace scxml_core;

  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  QApplication app(argc, argv);

  std::cout << "CRS Starting" << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>(NODE_NAME);
  executor.add_node(node);
  rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // crs executive
  CRSExecutive exec(node);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub =
      node->create_publisher<std_msgs::msg::String>(CURRENT_ST_TOPIC, rclcpp::QoS(rclcpp::KeepLast(7)));

  rclcpp::Service<srv::ExecuteAction>::SharedPtr exec_action_server = node->create_service<srv::ExecuteAction>(
      EXECUTE_ACTION_SERVICE,
      [&exec, &node](std::shared_ptr<srv::ExecuteAction::Request> req,
                     std::shared_ptr<srv::ExecuteAction::Response> res) -> void {
        res->succeeded = true;
        if (exec.getSM()->isBusy())
        {
          res->succeeded = false;
          res->err_msg = "State Machine is busy";
          RCLCPP_ERROR(node->get_logger(), res->err_msg.c_str());
          return;
        }

        TransitionResult sm_res = exec.getSM()->execute(Action{ id : req->action_id, data : boost::any() });
        RCLCPP_INFO(node->get_logger(),"Executed requested action %s",req->action_id.c_str());
        if (!sm_res)
        {
          res->succeeded = false;
          res->err_msg = sm_res.getErrorMessage();
          RCLCPP_ERROR(node->get_logger(), res->err_msg.c_str());
          return;
        }
      });

  rclcpp::Service<srv::GetAvailableActions>::SharedPtr get_actions_srv = node->create_service<srv::GetAvailableActions>(
      GET_AVAILABLE_ACTIONS_SERVICE,
      [&exec, &node](std::shared_ptr<srv::GetAvailableActions::Request> req,
                     std::shared_ptr<srv::GetAvailableActions::Response> res) -> void {
        res->succeeded = true;
        if (!exec.getSM()->wait(WAIT_SM_BUSY_TIMEOUT))
        {
          res->succeeded = false;
          res->err_msg = "State Machine is busy";
          RCLCPP_ERROR(node->get_logger(), res->err_msg.c_str());
          return;
        }
        std::vector<std::string> action_ids = exec.getSM()->getAvailableActions();
        if (req->search_pattern.empty())
        {
          res->action_ids = action_ids;
          return;
        }

        if (action_ids.empty())
        {
          return;
        }

        for (const auto& id : action_ids)
        {
          if (std::regex_search(id, std::regex(req->search_pattern)))
          {
            res->action_ids.push_back(id);
          }
        }
        RCLCPP_WARN_EXPRESSION(node->get_logger(),
                               res->action_ids.empty(),
                               "No action ids matched the '%s' search pattern",
                               req->search_pattern.c_str());
      });

  rclcpp::TimerBase::SharedPtr pub_timer =
      node->create_wall_timer(std::chrono::duration<double>(STATE_PUB_RATE), [&exec, &state_pub]() {
        if (!exec.getSM()->isRunning())
        {
          return;
        }
        std_msgs::msg::String msg;
        msg.data = exec.getSM()->getCurrentState(true);
        state_pub->publish(msg);
      });

  // start sm
  if (!exec.getSM()->start())
  {
    RCLCPP_ERROR(node->get_logger(), "CRS SM failed to start");
  }
  RCLCPP_INFO(node->get_logger(), "CRS Application started");

  // main loop
  while (rclcpp::ok())
  {
    app.processEvents(QEventLoop::AllEvents,100);
    executor.spin_some(rclcpp::Duration::from_seconds(ROS_SPIN_TIMEOUT).to_chrono<std::chrono::nanoseconds>());
  }
  app.exit();

  return 0;
}
