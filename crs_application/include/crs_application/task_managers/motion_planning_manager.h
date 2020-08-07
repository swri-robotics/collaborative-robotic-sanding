/*
 * @author ros-industrial
 * @file motion_planning_manager.h
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

#ifndef INCLUDE_CRS_APPLICATION_TASK_MANAGERS_MOTION_PLANNING_MANAGER_H_
#define INCLUDE_CRS_APPLICATION_TASK_MANAGERS_MOTION_PLANNING_MANAGER_H_

#include <memory>
#include <atomic>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <crs_msgs/srv/call_freespace_motion.hpp>
#include <crs_msgs/srv/plan_process_motions.hpp>
#include "crs_application/common/common.h"
#include "crs_application/common/config.h"
#include "crs_application/common/datatypes.h"

namespace crs_application
{
namespace task_managers
{
class MotionPlanningManager
{
public:
  MotionPlanningManager(std::shared_ptr<rclcpp::Node> node);
  virtual ~MotionPlanningManager();

  // initialization and configuration
  common::ActionResult init();
  common::ActionResult configure(const config::MotionPlanningConfig& config);
  common::ActionResult setInput(const std::vector<datatypes::ProcessToolpathData>& input);

  // Process Actions
  common::ActionResult splitToolpaths();
  common::ActionResult planProcessPaths();
  common::ActionResult planMediaChanges();

  /**
   * @brief previews the entire set of robot trajectories.  This method is blocking
   * therefore it must be executed asynchronously
   * @return True always
   */
  common::ActionResult showPreview();

  /**
   * @brief stops an ongoing preview
   * @return  True always
   */
  common::ActionResult hidePreview();

  // Results
  const datatypes::ProcessExecutionData& getResult() { return result_; }

protected:
  // support methods
  common::ActionResult checkPreReq();
  boost::optional<trajectory_msgs::msg::JointTrajectory>
  planFreeSpace(const std::string& plan_name, crs_msgs::srv::CallFreespaceMotion::Request::SharedPtr req);

  std::shared_ptr<rclcpp::Node> node_;
  std::vector<datatypes::ProcessToolpathData> input_process_toolpaths;
  std::shared_ptr<config::MotionPlanningConfig> config_ = nullptr;
  sensor_msgs::msg::JointState::SharedPtr home_js_;
  datatypes::ProcessExecutionData result_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;  // preview publisher
  rclcpp::Client<crs_msgs::srv::CallFreespaceMotion>::SharedPtr call_freespace_planning_client_;
  rclcpp::Client<crs_msgs::srv::PlanProcessMotions>::SharedPtr process_motion_planning_client_;

  // process data
  std::vector<datatypes::ProcessToolpathData> process_toolpaths_;

  // others
  std::atomic<bool> publish_preview_enabled_;
};

} /* namespace task_managers */
} /* namespace crs_application */

#endif /* INCLUDE_CRS_APPLICATION_TASK_MANAGERS_MOTION_PLANNING_MANAGER_H_ */
