/*
 * @author Jorge Nicho
 * @file scan_acquisition_manager.cpp
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

#include <boost/format.hpp>
#include <Eigen/Core>
#include "crs_application/task_managers/scan_acquisition_manager.h"

static const double WAIT_FOR_SERVICE_PERIOD = 10.0;
static const double WAIT_MESSAGE_TIMEOUT = 2.0;
static const std::size_t POSES_ARRAY_SIZE = 6;
static const double TRAJECTORY_TIME_TOLERANCE = 5.0;
static const double ROS_SPIN_TIMEOUT = 0.1;
static const double WAIT_JOINT_STATE_TIMEOUT = 2.0;

static const std::string CURRENT_JOINT_STATE_TOPIC = "joint_states";
static const std::string POINT_CLOUD_TOPIC = "custom_camera/custom_points";
static const std::string FREESPACE_MOTION_PLAN_SERVICE = "plan_freespace_motion";
static const std::string FOLLOW_JOINT_TRAJECTORY_ACTION = "follow_joint_trajectory";
static const std::string SCAN_POSES_TOPIC = "scan_poses";

static const std::string MANAGER_NAME = "ScanAcquisitionManager";
static const std::string DEFAULT_WORLD_FRAME_ID = "world";
static const std::string MOVEIT_SCAN_GROUP = "manipulator_scan";

namespace crs_application
{
namespace task_managers
{
ScanAcquisitionManager::ScanAcquisitionManager(std::shared_ptr<rclcpp::Node> node, moveit::planning_interface::PlanningComponentPtr moveit_arm)
  : node_(node)
  , moveit_arm_(moveit_arm)
  , scan_poses_(std::vector<geometry_msgs::msg::Transform>())
  , tool_frame_("")
  , max_time_since_last_point_cloud_(0.1)
  , point_clouds_(std::vector<sensor_msgs::msg::PointCloud2>())
  , scan_index_(0)
{
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  private_node_ = std::make_shared<rclcpp::Node>(MANAGER_NAME + "_private", node_options);
  pnode_executor_.add_node(private_node_);
}

ScanAcquisitionManager::~ScanAcquisitionManager() {}
common::ActionResult ScanAcquisitionManager::init()
{
  using namespace std::chrono_literals;

  // general parameters
  tool_frame_ = node_->declare_parameter("camera_frame_id", "eoat_link");
  max_time_since_last_point_cloud_ = node_->declare_parameter("max_time_since_last_point_cloud", 0.1);
  pre_acquisition_pause_ = node_->declare_parameter("pre_acquisition_pause", 1.0);

  // subscribers
  point_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      POINT_CLOUD_TOPIC, 1, std::bind(&ScanAcquisitionManager::handlePointCloud, this, std::placeholders::_1));

  // publishers
  scan_poses_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(SCAN_POSES_TOPIC, rclcpp::QoS(1));

  // service client
  call_freespace_motion_client_ =
      node_->create_client<crs_msgs::srv::CallFreespaceMotion>(FREESPACE_MOTION_PLAN_SERVICE);

  // trajectory execution action
  trajectory_exec_client_cbgroup_ =
      private_node_->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  trajectory_exec_client_ =
      rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(private_node_->get_node_base_interface(),
                                                                                private_node_->get_node_graph_interface(),
                                                                                private_node_->get_node_logging_interface(),
                                                                                private_node_->get_node_waitables_interface(),
                                                                                FOLLOW_JOINT_TRAJECTORY_ACTION,
                                                                                trajectory_exec_client_cbgroup_);
  // moveit setup
  if(!moveit_arm_)
  {
    moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(private_node_);
    moveit_arm_ = std::make_shared<moveit::planning_interface::PlanningComponent>(MOVEIT_SCAN_GROUP, moveit_cpp_);
  }

  // waiting for services
  common::ActionResult res;
  std::vector<rclcpp::ClientBase*> srv_clients = { call_freespace_motion_client_.get() };
  RCLCPP_INFO(node_->get_logger(), "%s waiting for services", MANAGER_NAME.c_str());
  if (!std::all_of(srv_clients.begin(), srv_clients.end(), [this, &res](rclcpp::ClientBase* c) {
        if (!c->wait_for_service(std::chrono::duration<double>(WAIT_FOR_SERVICE_PERIOD)))
        {
          res.succeeded = false;
          res.err_msg = boost::str(boost::format("service '%s' was not found") % c->get_service_name());
          return false;
        }
        return true;
      }))
  {
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
  }

  return true;
}

common::ActionResult ScanAcquisitionManager::configure(const config::ScanAcquisitionConfig& config)
{
  using namespace std::chrono_literals;
  using namespace Eigen;

  common::ActionResult res;
  if (config.scan_poses.empty())
  {
    res.err_msg = "no scan poses were found in configuration";
    res.succeeded = false;
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }

  scan_poses_.clear();
  for (std::size_t i = 0; i < config.scan_poses.size(); i++)
  {
    geometry_msgs::msg::Transform tf;
    auto& t = tf.translation;
    auto& q = tf.rotation;
    const std::vector<double>& pose_data = config.scan_poses[i];
    if (pose_data.size() < POSES_ARRAY_SIZE)
    {
      res.err_msg = boost::str(boost::format("Scan Pose has less than %lu elements") % POSES_ARRAY_SIZE);
      res.succeeded = false;
      RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
      return res;
    }

    Isometry3d eigen_t = Translation3d(Vector3d(pose_data[0], pose_data[1], pose_data[2])) *
                         AngleAxisd(pose_data[3], Vector3d::UnitX()) * AngleAxisd(pose_data[4], Vector3d::UnitY()) *
                         AngleAxisd(pose_data[5], Vector3d::UnitZ());
    Quaterniond eigen_q(eigen_t.linear());
    std::tie(t.x, t.y, t.z) =
        std::make_tuple(eigen_t.translation().x(), eigen_t.translation().y(), eigen_t.translation().z());
    std::tie(q.x, q.y, q.z, q.w) = std::make_tuple(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w());
    scan_poses_.push_back(tf);
  }

  // publish scan poses
  scan_poses_pub_timer_ = node_->create_wall_timer(10ms, [this]() -> void {
    geometry_msgs::msg::PoseArray poses;
    poses.header.frame_id = DEFAULT_WORLD_FRAME_ID;
    for (std::size_t i = 0; i < scan_poses_.size(); i++)
    {
      geometry_msgs::msg::Pose p;
      p.position.x = scan_poses_[i].translation.x;
      p.position.y = scan_poses_[i].translation.y;
      p.position.z = scan_poses_[i].translation.z;
      p.orientation = scan_poses_[i].rotation;
      poses.poses.push_back(p);
    }
    scan_poses_pub_->publish(poses);
  });
  rclcpp::spin_some(node_);

  tool_frame_ = config.tool_frame;

  RCLCPP_INFO(node_->get_logger(), "%s got %lu scan poses", MANAGER_NAME.c_str(), scan_poses_.size());
  return true;
}

common::ActionResult ScanAcquisitionManager::verify()
{
  common::ActionResult res = checkPreReqs();
  if (!res)
  {
    return res;
  }

  // resetting variables
  scan_index_ = 0;
  point_clouds_.clear();
  return true;
}

common::ActionResult ScanAcquisitionManager::moveRobot()
{

  enum PlanningPipelines: int
  {
    TESSERACT = 1,
    MOVEIT2
  };
  static const std::map<std::string, int> planning_pipeline_map = {{"tesseract",PlanningPipelines::TESSERACT},
                                                                   {"moveit2",PlanningPipelines::MOVEIT2}};

  std::string default_pipeline_name = "tesseract";
  int id = planning_pipeline_map.at(default_pipeline_name);
  std::string pipeline_name;
  if(private_node_->get_parameter("planning_pipeline",pipeline_name) &&
      planning_pipeline_map.count(pipeline_name) > 0)
  {
    id = planning_pipeline_map.at(pipeline_name);
    RCLCPP_WARN(node_->get_logger(), "%s selected '%s' planning pipeline", MANAGER_NAME.c_str(),
                pipeline_name.c_str());
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "%s no planner selected, defaulting to %s", MANAGER_NAME.c_str(),
                default_pipeline_name.c_str());
  }

  switch(id)
  {
    case PlanningPipelines::TESSERACT:
      return moveRobotWithTesseract();
    case PlanningPipelines::MOVEIT2:
      return moveRobotWithMoveIt();
  }
  return moveRobotWithTesseract();
}

common::ActionResult ScanAcquisitionManager::moveRobotWithTesseract()
{
  auto freespace_motion_request = std::make_shared<crs_msgs::srv::CallFreespaceMotion::Request>();
    freespace_motion_request->target_link = tool_frame_;
    freespace_motion_request->goal_pose = scan_poses_.at(scan_index_);
    freespace_motion_request->execute = true;

    auto result_future = call_freespace_motion_client_->async_send_request(freespace_motion_request);
    if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "%s Call Freespace Motion service call failed", MANAGER_NAME.c_str());
      return false;
    }
    auto result = result_future.get();

    if (result->success)
    {
      // todo(ayoungs): wait for robot to finish moving, for now just wait 10 seconds
      std::chrono::duration<double> sleep_dur(10.0);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::seconds>(sleep_dur));

      return true;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), result->message.c_str());
      return false;
    }
}

common::ActionResult ScanAcquisitionManager::moveRobotWithMoveIt()
{
  using namespace moveit::planning_interface;

  // getting workspace parameters
  std::vector<double> workspace_min, workspace_max;
  std::map<std::string, rclcpp::Parameter> workspace_bound_param;
  if(private_node_->get_parameters("workspace_bounds", workspace_bound_param))
  {
    std::stringstream ss;
    for(auto& kv : workspace_bound_param)
    {
      ss << kv.first + " ";
    }
    RCLCPP_INFO(node_->get_logger(),"Found workspace_bounds parameters: %s", ss.str().c_str());

    workspace_min = workspace_bound_param["min"].as_double_array();
    workspace_max = workspace_bound_param["max"].as_double_array();
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(),"Failed to get workspace bound parameters");
  }

  // create goal
  geometry_msgs::msg::PoseStamped goal_pose;
  const auto transform = scan_poses_.at(scan_index_);
  auto& pos = goal_pose.pose.position;
  auto& t = transform.translation;
  std::tie(pos.x, pos.y, pos.z) = std::make_tuple(t.x, t.y, t.z);
  goal_pose.pose.orientation = transform.rotation;
  goal_pose.header.frame_id = DEFAULT_WORLD_FRAME_ID;

  // grabbing current state
  sensor_msgs::msg::JointState::SharedPtr current_st = common::getCurrentState(this->node_, CURRENT_JOINT_STATE_TOPIC, WAIT_JOINT_STATE_TIMEOUT);
  if (!current_st)
  {
    common::ActionResult res;
    res.succeeded = false;
    res.err_msg = "Failed to get the current state";
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }
  std::map<std::string, double> variable_pos;
  std::transform(current_st->name.begin(), current_st->name.end(), current_st->position.begin(),
                 std::inserter(variable_pos,variable_pos.end()), [](std::string name, double val){
    return std::make_pair(name,val);
  });

  moveit::core::RobotState current_robot_st(moveit_cpp_->getRobotModel());
  current_robot_st.setVariablePositions(variable_pos);
  //moveit_arm_->setStartStateToCurrentState();
  moveit_arm_->setStartState(current_robot_st);
  moveit_arm_->setGoal(goal_pose, tool_frame_);

  moveit::planning_interface::PlanningComponent::PlanRequestParameters planning_parameters;
  planning_parameters.planning_attempts = 4;
  planning_parameters.planning_time = 30.0;
  planning_parameters.max_velocity_scaling_factor = 0.5;
  planning_parameters.max_acceleration_scaling_factor = 0.5;
  planning_parameters.planner_id = "geometric::RRTConnect";
  planning_parameters.planning_pipeline = "ompl";
  moveit_arm_->setWorkspace(workspace_min[0], workspace_min[1], workspace_min[2],
                            workspace_max[0], workspace_max[1], workspace_max[2]);

  PlanningComponent::PlanSolution plan_solution = moveit_arm_->plan(planning_parameters);
  if(!plan_solution)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s MoveIt Planning to scan pose %lu failed failed", MANAGER_NAME.c_str(), scan_index_);
    return false;
  }

  moveit_msgs::msg::RobotTrajectory robot_trajectory;
  plan_solution.trajectory->getRobotTrajectoryMsg(robot_trajectory);
  auto res = execTrajectory(robot_trajectory.joint_trajectory);
  return res;
}

common::ActionResult ScanAcquisitionManager::execTrajectory(const trajectory_msgs::msg::JointTrajectory& traj)
{
  using namespace control_msgs::action;
  static const double GOAL_ACCEPT_TIMEOUT_PERIOD = 1.0;

  auto print_time = [this](const trajectory_msgs::msg::JointTrajectory& traj)
  {
    for(std::size_t i = 0; i < traj.points.size(); i++)
    {
      auto& p  = traj.points[i];
      rclcpp::Duration dur(p.time_from_start);
      RCLCPP_INFO(node_->get_logger(),"Point %lu time: %f", i,dur.seconds());
    }
  };
  print_time(traj);

  rclcpp::Duration traj_dur(traj.points.back().time_from_start);
  common::ActionResult res = false;

  FollowJointTrajectory::Goal goal;
  goal.trajectory = traj;
  auto goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  // TODO populate tolerances

  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  std::promise<bool> result_promise;
  std::future<bool> result_fut = result_promise.get_future();
  goal_options.goal_response_callback = [this,&result_promise, &res](std::shared_future<GoalHandle::SharedPtr> future){
    if(!future.get())
    {
      res.err_msg = "Failed to accepte goal";
      RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
      trajectory_exec_client_->async_cancel_all_goals();
      result_promise.set_value(false);
    }
  };

  goal_options.result_callback = [this,&result_promise, &res](const GoalHandle::WrappedResult& result){
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      res.err_msg = result.result->error_string;
      RCLCPP_ERROR(node_->get_logger(), "Trajectory execution failed with error message: %s", res.err_msg.c_str());
      result_promise.set_value(false);
      return;
    }
    result_promise.set_value(true);
  };

  // send goal
  trajectory_exec_fut_ = trajectory_exec_client_->async_send_goal(goal, goal_options);

  // spinning
  std::atomic<bool> done;
  done = false;
  auto spinner_fut = std::async([&done, this]() -> bool {
    while (!done)
    {
      pnode_executor_.spin_some(
          rclcpp::Duration::from_seconds(ROS_SPIN_TIMEOUT).to_chrono<std::chrono::nanoseconds>());
    }
    return true;
  });

  traj_dur = traj_dur + rclcpp::Duration(std::chrono::duration<double>(TRAJECTORY_TIME_TOLERANCE));
  RCLCPP_INFO(node_->get_logger(), "Waiting %f seconds for goal", traj_dur.seconds());
  std::future_status status = result_fut.wait_for(traj_dur.to_chrono<std::chrono::seconds>());
  done = true;
  if (status != std::future_status::ready)
  {
    res.err_msg = boost::str(boost::format("Trajectory execution timed out with flag %i") %  static_cast<int>(status));
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
    trajectory_exec_client_->async_cancel_all_goals();
    return res;
  }

  return true;
}

common::ActionResult ScanAcquisitionManager::capture()
{
  // sleeping first
  // std::chrono::duration<double> sleep_dur(WAIT_MESSAGE_TIMEOUT);
  // rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::seconds>(sleep_dur));

  // todo(ayoungs): waitForMessage seems to be broken?
  // auto msg = common::waitForMessage<sensor_msgs::msg::PointCloud2>(node_, POINT_CLOUD_TOPIC, WAIT_MESSAGE_TIMEOUT);
  // if (!msg)
  //{
  //  common::ActionResult res;
  //  res.succeeded = false;
  //  res.err_msg = "Failed to get point cloud message";
  //  return res;
  //}
  // curr_point_cloud_ = *msg;

  // TODO(ayoungs): transform point cloud

  // TODO asses if the logic below is still needed
  if (node_->now() - curr_point_cloud_.header.stamp >= rclcpp::Duration(max_time_since_last_point_cloud_))
  {
    point_clouds_.push_back(curr_point_cloud_);
    return true;
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get scan");
    return false;
  }
}

common::ActionResult ScanAcquisitionManager::checkQueue()
{
  scan_index_++;
  if (scan_index_ < scan_poses_.size())
  {
    return false;
  }
  else
  {
    // save off results and reset the point clouds
    result_.point_clouds = point_clouds_;
    scan_index_ = 0;
    point_clouds_.clear();
    return true;
  }
}

common::ActionResult ScanAcquisitionManager::checkPreReqs()
{
  common::ActionResult res;

  if (scan_poses_.empty())
  {
    res.succeeded = false;
    res.err_msg = "No scan positions available, cannot proceed";
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }

  if (tool_frame_.empty())
  {
    res.succeeded = false;
    res.err_msg = "No camera frame has been specified, cannot proceed";
    RCLCPP_ERROR(node_->get_logger(), "%s %s", MANAGER_NAME.c_str(), res.err_msg.c_str());
    return res;
  }
  return true;
}

void ScanAcquisitionManager::handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  curr_point_cloud_ = *msg;
}

}  // end of namespace task_managers
}  // end of namespace crs_application
