/*
 * @author Jorge Nicho
 * @file datatypes.h
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

#ifndef INCLUDE_CRS_APPLICATION_COMMON_DATATYPES_H_
#define INCLUDE_CRS_APPLICATION_COMMON_DATATYPES_H_

#include <vector>
#include <Eigen/Geometry>
#include <crs_msgs/msg/process_motion_plan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

namespace crs_application
{
namespace datatypes
{
struct ScanAcquisitionResult
{
  Eigen::Isometry3d transform;
};

struct ProcessToolpathData
{
  std::vector<geometry_msgs::msg::PoseArray> rasters;
};

struct MediaChangeMotionPlan
{
  trajectory_msgs::msg::JointTrajectory start_traj;
  trajectory_msgs::msg::JointTrajectory return_traj;
};

enum class ProcessExecActions: int
{
  EXEC_PROCESS = 1,
  EXEC_MEDIA_CHANGE,
  DONE
};

struct ProcessExecutionData
{
  trajectory_msgs::msg::JointTrajectory move_to_start;
  std::vector< crs_msgs::msg::ProcessMotionPlan > process_plans;
  std::vector< MediaChangeMotionPlan > media_change_plans;
};

struct PartInspectionResult
{
};

}  // namespace datatypes
}  // namespace crs_application

#endif /* INCLUDE_CRS_APPLICATION_COMMON_DATATYPES_H_ */
