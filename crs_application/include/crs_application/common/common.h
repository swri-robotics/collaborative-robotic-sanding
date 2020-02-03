/*
 * @author Jorge Nicho
 * @file common.h
 * @date Jan 16, 2020
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

#ifndef INCLUDE_CRS_APPLICATION_COMMON_COMMON_H_
#define INCLUDE_CRS_APPLICATION_COMMON_COMMON_H_

#include <string>
#include <boost/any.hpp>
#include <boost/format.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace crs_application
{
namespace common
{

struct ActionResult
{
  bool succeeded = false; /** @brief indicates that the whether or not the action succeeded*/
  std::string err_msg; /** @brief an optional error message if the requested action failed **/
  boost::any opt_data = boost::any(); /** @brief optional data returned if the requested action failed */

  ActionResult(bool succeeded = true, std::string msg = "", boost::any data = boost::any()) :
      succeeded(succeeded), err_msg(msg), opt_data(data)
  {

  }

  ActionResult& operator=(const bool& b)
  {
    this->succeeded = b;
    return *this;
  }

  operator bool() const
  {
    return succeeded;
  }

};

/**
 * @brief compares the joints in js1 to js2 and retuns the max difference.  If a joint in js1 is not
 * found in js2 then -1 will be returned
 * @param js1 joint state 1
 * @param js2 joint state 1
 * @return A double value > 0 or -1 when a joint isn't found.
 */
static double compare(const sensor_msgs::msg::JointState& js1, const sensor_msgs::msg::JointState& js2,
                      std::string& err_msg)
{
  double diff = 0.0;
  for(std::size_t j = 0; j < js1.name.size(); j++)
  {
    std::vector<std::string>::const_iterator pos = std::find(js2.name.begin(), js2.name.end(), js1.name[j]);
    if(pos == js2.name.end())
    {
      err_msg = boost::str(boost::format("Joint %s was not found") % js1.name[j]);
      return -1.0;
    }
    std::size_t idx = std::distance(js2.name.begin(), pos);
    double temp_diff = std::abs(js1.position[j] - js2.position[idx]);
    diff = temp_diff > diff ? temp_diff : diff;
  }
  return diff;
}

static sensor_msgs::msg::JointState::SharedPtr getCurrentState(std::shared_ptr<rclcpp::Node> node,
                                                               const std::string topic_name,
                                                               double timeout)
{
  sensor_msgs::msg::JointState::SharedPtr msg = nullptr;
  std::promise<sensor_msgs::msg::JointState> promise_obj;
  std::future<sensor_msgs::msg::JointState> fut_obj = promise_obj.get_future();

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subs;
  subs = node->create_subscription<sensor_msgs::msg::JointState>(
      topic_name,rclcpp::QoS(1),[&promise_obj]
                                   (const sensor_msgs::msg::JointState::SharedPtr msg) -> void
  {
    promise_obj.set_value(*msg);
  });

  std::future_status sts = fut_obj.wait_for(std::chrono::duration<double>(timeout));
  subs.reset();
  /** @warning there's no clean way to close a subscription but according to this issue
                           https://github.com/ros2/rclcpp/issues/205, destroying the subscription
                           should accomplish the same */
  if(sts != std::future_status::ready)
  {
    return nullptr;
  }
  msg = std::make_shared<sensor_msgs::msg::JointState>(fut_obj.get());
  return msg;
}

}
}

#endif /* INCLUDE_CRS_APPLICATION_COMMON_COMMON_H_ */
