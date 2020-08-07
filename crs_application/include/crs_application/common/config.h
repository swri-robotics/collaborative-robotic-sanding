/*
 * @author Jorge Nicho
 * @file config.h
 * @date Feb 25, 2020
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

#ifndef INCLUDE_CRS_APPLICATION_COMMON_CONFIG_H_
#define INCLUDE_CRS_APPLICATION_COMMON_CONFIG_H_

#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <boost/optional.hpp>

namespace crs_application
{
namespace config
{
struct MotionPlanningConfig
{
  // home pose
  bool pre_move_home;
  std::vector<std::string> home_joint_names;
  std::vector<double> joint_home_position;

  // process path
  double tool_speed;
  Eigen::Isometry3d offset_pose;
  double retreat_dist;
  double approac_dist;
  std::string tool_frame;
  double target_force = 20;

  // media change
  double media_change_time;            /** @brief time that needs to elapse for the next media change secs */
  Eigen::Isometry3d media_change_pose; /** @brief in world coordinates */
  std::vector<std::string> media_joint_names;
  std::vector<double> joint_media_position;

  // preview
  double preview_time_scaling = 1.0; /** @brief preview will be played at a scaled speed */
};

struct ProcessExecutionConfig
{
  double traj_time_tolerance = 5.0; /** @brief time tolerance on trajectory duration */
  std::vector<double> joint_tolerance =
      std::vector<double>(6, (3.1416 / 180.0) * 2.0); /** @brief how close the robot needs to be to the last position in
                                                         radians */
  Eigen::Vector3d position_path_tolerance = Eigen::Vector3d::Ones() * 0.01;
  Eigen::Vector3d orientation_path_tolerance = Eigen::Vector3d::Ones() * 0.05;
  Eigen::Vector3d position_goal_tolerance = Eigen::Vector3d::Ones() * 0.01;
  Eigen::Vector3d orientation_goal_tolerance = Eigen::Vector3d::Ones() * 0.05;
  double tool_speed = 0.05;
  double target_force = 20;
  double force_tolerance = 5;

  bool force_controlled_trajectories = false;

  std::string ur_tool_change_script;
  bool execute_tool_change;
};

struct ScanAcquisitionConfig
{
  std::vector<std::vector<double> > scan_poses;
  std::string tool_frame;
  bool skip_on_failure = false;
};

struct PartRegistrationConfig
{
  std::string target_frame_id;
  std::string part_file;
  std::string toolpath_file;
  std::array<double, 6> seed_pose;
  std::array<double, 6> simulation_pose;
};

struct PartReworkConfig
{
  std::vector<std::vector<double> > scan_poses;
  std::string tool_frame;
  double pre_acquisition_pause = 2.0; /** @brief seconds */
  bool skip_on_failure = false;
};

template <class T>
boost::optional<T> parse(YAML::Node& config, std::string& err_msg);

template <>
boost::optional<MotionPlanningConfig> parse(YAML::Node& config, std::string& err_msg);

template <>
boost::optional<ProcessExecutionConfig> parse(YAML::Node& config, std::string& err_msg);

template <>
boost::optional<ScanAcquisitionConfig> parse(YAML::Node& config, std::string& err_msg);

template <>
boost::optional<PartRegistrationConfig> parse(YAML::Node& config, std::string& err_msg);

template <>
boost::optional<PartReworkConfig> parse(YAML::Node& config, std::string& err_msg);

}  // namespace config
}  // namespace crs_application

#endif /* INCLUDE_CRS_APPLICATION_COMMON_CONFIG_H_ */
