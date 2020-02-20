/*
 * @author Jorge Nicho
 * @file joint_pose_trajectory.hpp
 * @date Feb 19, 2020
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

#ifndef INCLUDE_CRS_GAZEBO_PLUGINS_JOINT_POSE_TRAJECTORY_HPP_
#define INCLUDE_CRS_GAZEBO_PLUGINS_JOINT_POSE_TRAJECTORY_HPP_

#include <gazebo/common/Plugin.hh>
#include <memory>

namespace crs_gazebo_plugins
{
class JointPoseTrajectoryPrivate;

/// Set the trajectory of points to be followed by joints in simulation.
/// Currently only positions specified in the trajectory_msgs are handled.
/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_joint_pose_trajectory"
        filename="libgazebo_ros_joint_pose_trajectory.so">

      <ros>

        <!-- Add a namespace -->
        <namespace>/my_namespace</namespace>

        <!-- Remap the default topic -->
        <remapping>set_joint_trajectory:=my_trajectory</remapping>

      </ros>

      <!-- Update rate in Hz -->
      <update_rate>2</update_rate>

    </plugin>
  \endcode
*/
class JointPoseTrajectory : public gazebo::ModelPlugin
{
public:
  /// Constructor
  JointPoseTrajectory();

  /// Destructor
  ~JointPoseTrajectory();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<JointPoseTrajectoryPrivate> impl_;
};
}  // namespace crs_gazebo_plugins



#endif /* INCLUDE_CRS_GAZEBO_PLUGINS_JOINT_POSE_TRAJECTORY_HPP_ */
