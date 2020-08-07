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
#include <Eigen/Geometry>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>

namespace crs_application
{
namespace common
{
struct ActionResult
{
  bool succeeded = false;             /** @brief indicates that the whether or not the action succeeded*/
  std::string err_msg;                /** @brief an optional error message if the requested action failed **/
  boost::any opt_data = boost::any(); /** @brief optional data returned if the requested action failed */

  ActionResult(bool succeeded = true, std::string msg = "", boost::any data = boost::any())
    : succeeded(succeeded), err_msg(msg), opt_data(data)
  {
  }

  ActionResult& operator=(const bool& b)
  {
    this->succeeded = b;
    return *this;
  }

  operator bool() const { return succeeded; }
};

/**
 * @brief compares the joints in js1 to js2 and retuns the max difference.  If a joint in js1 is not
 * found in js2 then -1 will be returned
 * @param js1 joint state 1
 * @param js2 joint state 1
 * @return A double value > 0 or -1 when a joint isn't found.
 */
static double compare(const sensor_msgs::msg::JointState& js1,
                      const sensor_msgs::msg::JointState& js2,
                      std::string& err_msg)
{
  double diff = 0.0;
  for (std::size_t j = 0; j < js1.name.size(); j++)
  {
    std::vector<std::string>::const_iterator pos = std::find(js2.name.begin(), js2.name.end(), js1.name[j]);
    if (pos == js2.name.end())
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

static Eigen::Isometry3d toEigen(std::array<double, 6>& pose_data)
{
  using namespace Eigen;
  Isometry3d eigen_t = Translation3d(Vector3d(pose_data[0], pose_data[1], pose_data[2])) *
                       AngleAxisd(pose_data[3], Vector3d::UnitX()) * AngleAxisd(pose_data[4], Vector3d::UnitY()) *
                       AngleAxisd(pose_data[5], Vector3d::UnitZ());
  return eigen_t;
}

static geometry_msgs::msg::Pose toPoseMsg(const Eigen::Isometry3d& eigen_t)
{
  using namespace Eigen;
  geometry_msgs::msg::Pose pose_msg;
  auto& t = pose_msg.position;
  auto& q = pose_msg.orientation;
  Quaterniond eigen_q(eigen_t.linear());
  std::tie(t.x, t.y, t.z) =
      std::make_tuple(eigen_t.translation().x(), eigen_t.translation().y(), eigen_t.translation().z());
  std::tie(q.x, q.y, q.z, q.w) = std::make_tuple(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w());
  return pose_msg;
}

static geometry_msgs::msg::Pose toPoseMsg(std::array<double, 6>& pose_data) { return toPoseMsg(toEigen(pose_data)); }

static geometry_msgs::msg::Transform toTransformMsg(const Eigen::Isometry3d& eigen_t)
{
  using namespace Eigen;
  geometry_msgs::msg::Transform transform_msg;
  auto& t = transform_msg.translation;
  auto& q = transform_msg.rotation;
  Quaterniond eigen_q(eigen_t.linear());
  std::tie(t.x, t.y, t.z) =
      std::make_tuple(eigen_t.translation().x(), eigen_t.translation().y(), eigen_t.translation().z());
  std::tie(q.x, q.y, q.z, q.w) = std::make_tuple(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w());
  return transform_msg;
}

static geometry_msgs::msg::Transform toTransformMsg(std::array<double, 6>& pose_data)
{
  return toTransformMsg(toEigen(pose_data));
}

template <typename Srv>
static typename Srv::Response::SharedPtr waitForResponse(typename rclcpp::Client<Srv>::SharedPtr client,
                                                         typename Srv::Request::SharedPtr request,
                                                         double timeout,
                                                         double interval = 0.05)
{
  using namespace std::chrono;
  static rclcpp::Logger logger = rclcpp::get_logger(client->get_service_name());

  std::promise<typename Srv::Response::SharedPtr> promise;
  std::shared_future<typename Srv::Response::SharedPtr> res(promise.get_future());
  client->async_send_request(request, [&promise](const typename rclcpp::Client<Srv>::SharedFuture future) {
    auto result = future.get();
    promise.set_value(result);
  });
  system_clock::time_point timeout_time = system_clock::now() + duration_cast<nanoseconds>(duration<double>(timeout));

  std::future_status status = std::future_status::timeout;
  while (system_clock::now() < timeout_time)
  {
    status = res.wait_for(duration<double>(interval));
    if (status == std::future_status::ready)
    {
      RCLCPP_DEBUG(logger, "future ready");
      break;
    }
  }

  if (status != std::future_status::ready)
  {
    RCLCPP_ERROR(
        logger, "Timed out while waiting for response from service with status flag '%i'", static_cast<int>(status));
    return nullptr;
  }
  return res.get();
}

template <class Msg>
static std::shared_ptr<Msg>
waitForMessage(std::shared_ptr<rclcpp::Node> node, const std::string& topic_name, bool spin_node, double timeout)
{
  std::shared_ptr<Msg> msg = nullptr;
  std::promise<Msg> promise_obj;
  std::future<Msg> fut_obj = promise_obj.get_future();

  std::shared_ptr<rclcpp::Subscription<Msg>> subs;
  subs = node->create_subscription<Msg>(
      topic_name, rclcpp::QoS(1), [&promise_obj](const std::shared_ptr<Msg> msg) -> void {
        promise_obj.set_value(*msg);
      });

  std::atomic<bool> done;
  done = false;
  std::future<bool> spinner_fut;
  if (spin_node)
  {
    spinner_fut = std::async(std::launch::async, [&done, node, subs]() mutable -> bool {
      while (!done)
      {
        rclcpp::spin_some(node);
      }

      return true;
    });
  }

  std::future_status sts = fut_obj.wait_for(std::chrono::duration<double>(timeout));
  done = true;  // should stop the spining thread

  /** @warning there's no clean way to close a subscription but according to this issue
                           https://github.com/ros2/rclcpp/issues/205, destroying the subscription
                           should accomplish the same */
  std::string true_topic_name = subs->get_topic_name();
  subs.reset();  // stopping subscriber

  // waiting for spinning thread to exit
  if (spinner_fut.valid())
  {
    spinner_fut.get();
  }

  if (sts != std::future_status::ready)
  {
    std::string err_code = sts == std::future_status::timeout ? std::string("Timeout") : std::string("Deferred");
    RCLCPP_ERROR(node->get_logger(),
                 "%s error while waiting for message in topic %s",
                 err_code.c_str(),
                 true_topic_name.c_str());
    return nullptr;
  }
  msg = std::make_shared<Msg>(fut_obj.get());
  return msg;
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
      topic_name, rclcpp::QoS(1), [&promise_obj, node](const sensor_msgs::msg::JointState::SharedPtr msg) -> void {
        RCLCPP_INFO(node->get_logger(), "Got current state message");
        promise_obj.set_value(*msg);
      });

  RCLCPP_INFO(node->get_logger(), "Waiting for  current joint state");
  std::future_status sts = fut_obj.wait_for(std::chrono::duration<double>(timeout));

  RCLCPP_INFO(node->get_logger(), "Done waiting for current state");
  if (sts != std::future_status::ready)
  {
    return nullptr;
  }
  msg = std::make_shared<sensor_msgs::msg::JointState>(fut_obj.get());
  return msg;
}

}  // namespace common
}  // namespace crs_application

#endif /* INCLUDE_CRS_APPLICATION_COMMON_COMMON_H_ */
