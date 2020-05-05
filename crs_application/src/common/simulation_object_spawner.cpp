/*
 * @author Jorge Nicho
 * @file simulation_object_spawner.cpp
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

#include <boost/format.hpp>

#include <tf2_eigen/tf2_eigen.h>

#include "crs_application/common/simulation_object_spawner.h"

static const std::string SPAWN_ENTITY_SERVICE = "/spawn_entity";
static const std::string DELETE_ENTITY_SERVICE = "/delete_entity";

/**@
 * @brief creates a urdf formatted xml string
 * @param object_name
 * @param mesh_path
 * @param mass        Mass of the object in kg
 * @return
 */
static std::string createObjectURDF(const std::string& object_name,
                             const std::string& mesh_path,
                             const std::array<double,6>& pose_vals,
                             const double mass = 0.1 )
{
  std::string joint_xml_template = R"(
    <joint name="world_to_part" type="fixed">
        <parent link="world" />
        <child link="%s_link" />
        <origin xyz="%f %f %f" rpy="%f %f %f"/>
    </joint>
  )";

  std::string joint_xml = boost::str(boost::format(joint_xml_template) % object_name %
                                     pose_vals[0] %
                                     pose_vals[1] %
                                     pose_vals[2] %
                                     pose_vals[3] %
                                     pose_vals[4] %
                                     pose_vals[5]);

  std::string robot_xml_template = R"(
    <robot name="%s">
    <link name="world"/>
    <link name="%s_link">
        <visual>
            <geometry>
                <mesh filename="file://%s" />
            </geometry>
            <material name="Orange">
                <color rgba="1.0 0.4 0.0 1.0" />
            </material>
        </visual>
        <collision>
            <geometry>
                <mesh filename="file://%s" />
            </geometry>
        </collision>

        <inertial>
            <mass value="%f" />
            <origin rpy="0 0 0" xyz="0.0 0.0 0.0" />
            <inertia ixx="0.03" iyy="0.03" izz="0.03" ixy="0.0" ixz="0.0" iyz="0.0" />
        </inertial>
    </link>

    %s
    <gazebo>
        <static>true</static>
        <kinematic>true</kinematic>
    </gazebo>
    </robot>)";

  return boost::str(boost::format(robot_xml_template) %
                    object_name %
                    object_name %
                    mesh_path %
                    mesh_path %
                    mass %
                    joint_xml );
}

namespace crs_application{
namespace common{

SimulationObjectSpawner::SimulationObjectSpawner(rclcpp::Node::SharedPtr node):
    node_(node)
{
  spawn_client_ = node_->create_client<gazebo_msgs::srv::SpawnEntity>(SPAWN_ENTITY_SERVICE);
  delete_client_ = node_->create_client<gazebo_msgs::srv::DeleteEntity>(DELETE_ENTITY_SERVICE);
}

SimulationObjectSpawner::~SimulationObjectSpawner()
{

}

bool SimulationObjectSpawner::spawn(const std::string &obj_name, const std::string &reference_frame_id,
                                    const std::string& mesh_path, const std::array<double,6>& pose)
{
  using namespace std::chrono_literals;

  // load part into simulator if its running
  if(spawn_client_->wait_for_service(1s))
  {
     gazebo_msgs::srv::SpawnEntity::Request::SharedPtr spawn_req = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
     spawn_req->initial_pose = tf2::toMsg(Eigen::Isometry3d::Identity());
     spawn_req->xml = createObjectURDF(obj_name,mesh_path, pose);
     spawn_req->name = obj_name;
     spawn_req->reference_frame = reference_frame_id;
     using ResponseFutT = decltype(spawn_client_)::element_type::SharedFuture;
     RCLCPP_INFO_STREAM(node_->get_logger()," Spawning gazebo object:\n" << spawn_req->xml);
     spawn_client_->async_send_request(spawn_req,[this, xml = spawn_req->xml](ResponseFutT fut){
       if(fut.valid() && fut.get()->success)
       {
         RCLCPP_INFO(node_->get_logger(),"Spawned object in gazebo");
       }
       else
       {
         RCLCPP_ERROR(node_->get_logger(),"Spawn request in gazebo failed");
       }
     });
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(),"Did not find service \"%s\", skipping", spawn_client_->get_service_name());
    return false;
  }
  return true;
}

bool SimulationObjectSpawner::remove(const std::string &obj_name)
{
  using namespace std::chrono_literals;

  // load part into simulator if its running
  if(delete_client_->wait_for_service(1s))
  {
     gazebo_msgs::srv::DeleteEntity::Request::SharedPtr delete_req = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
     delete_req->name = obj_name;
     RCLCPP_INFO_STREAM(node_->get_logger()," Deleting gazebo object:\n" << obj_name);
     using ResponseFutT = decltype(delete_client_)::element_type::SharedFuture;
     delete_client_->async_send_request(delete_req,[this](ResponseFutT fut){
       if(fut.valid() && fut.get()->success)
       {
         RCLCPP_INFO(node_->get_logger(),"Deleted object from gazebo");
       }
       else
       {
         RCLCPP_ERROR(node_->get_logger(),"Deletion request in gazebo failed");
       }
     });
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(),"Did not find service \"%s\", skipping", delete_client_->get_service_name());
    return false;
  }
  return true;
}

}
}
