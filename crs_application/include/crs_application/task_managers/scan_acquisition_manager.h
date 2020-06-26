/*
 * @author Jorge Nicho
 * @file scan_acquisition_manager.h
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

#ifndef INCLUDE_CRS_APPLICATION_TASK_MANAGERS_SCAN_ACQUISITION_MANAGER_H_
#define INCLUDE_CRS_APPLICATION_TASK_MANAGERS_SCAN_ACQUISITION_MANAGER_H_

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "crs_application/common/common.h"
#include "crs_application/common/datatypes.h"
#include "crs_application/common/config.h"
#include <crs_msgs/srv/call_freespace_motion.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace crs_application
{
namespace task_managers
{
class ScanAcquisitionManager
{
public:
  ScanAcquisitionManager(std::shared_ptr<rclcpp::Node> node);
  virtual ~ScanAcquisitionManager();

  // initialization and configuration
  common::ActionResult init();
  common::ActionResult configure(const config::ScanAcquisitionConfig& config);

  // Process Actions
  /**
   * @brief verifies if robot is ready for scan
   * @return True when it is
   */
  common::ActionResult verify();
  common::ActionResult moveRobot();
  common::ActionResult capture();

  /**
   * @brief checks if there are any scan positions left in the queue
   * @return  True all scan positions have been visited
   */
  common::ActionResult checkQueue();

  // Results
  const datatypes::ScanAcquisitionResult& getResult() { return result_; }

protected:
  // support methods
  common::ActionResult checkPreReqs();

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> private_node_;
  rclcpp::executors::MultiThreadedExecutor pnode_executor_;
  datatypes::ScanAcquisitionResult result_;

  // parameters
  std::vector<geometry_msgs::msg::Transform> scan_poses_;
  std::string tool_frame_;
  double pre_acquisition_pause_;
  double max_time_since_last_point_cloud_;

  // subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr scan_poses_pub_;

  // service clients
  rclcpp::Client<crs_msgs::srv::CallFreespaceMotion>::SharedPtr call_freespace_motion_client_;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // timers
  rclcpp::TimerBase::SharedPtr scan_poses_pub_timer_;

  sensor_msgs::msg::PointCloud2 curr_point_cloud_;
  datatypes::ScanAcquisitionResult current_data_;
  uint scan_index_;

  void handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

}  // namespace task_managers
}  // namespace crs_application

#endif /* INCLUDE_CRS_APPLICATION_TASK_MANAGERS_SCAN_ACQUISITION_MANAGER_H_ */
