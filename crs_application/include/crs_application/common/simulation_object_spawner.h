/*
 * @author Jorge Nicho
 * @file simulation_object_spawner.h
 * @date May 4, 2020
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

#ifndef INCLUDE_CRS_APPLICATION_COMMON_SIMULATION_OBJECT_SPAWNER_H_
#define INCLUDE_CRS_APPLICATION_COMMON_SIMULATION_OBJECT_SPAWNER_H_

#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>

#include <rclcpp/rclcpp.hpp>

namespace crs_application
{
namespace common
{
class SimulationObjectSpawner
{
public:
  SimulationObjectSpawner(rclcpp::Node::SharedPtr node);
  virtual ~SimulationObjectSpawner();

  bool spawn(const std::string& obj_name,
             const std::string& reference_frame_id,
             const std::string& mesh_path,
             const std::array<double, 6>& pose);
  bool remove(const std::string& obj_name);

protected:
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_; /** @brief used to spawn the object on gazebo
                                                                             when if in simulation mode*/
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace common
}  // namespace crs_application
#endif /* INCLUDE_CRS_APPLICATION_COMMON_SIMULATION_OBJECT_SPAWNER_H_ */
