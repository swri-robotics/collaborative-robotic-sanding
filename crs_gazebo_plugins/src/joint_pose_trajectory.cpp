/*
 * joint_pose_trajectory.cpp
 *
 *  Created on: Feb 19, 2020
 *      Author: jnicho
 */

#include <atomic>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <crs_gazebo_plugins/joint_pose_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <boost/format.hpp>
#include <mutex>
#include <memory>
#include <string>
#include <vector>

static const std::string FOLLOW_JOINT_TRAJECTORY_ACTION = "follow_joint_trajectory";

using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>;

namespace crs_gazebo_plugins
{
class JointPoseTrajectoryPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  /// \param[in] info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /// \brief Callback for set joint trajectory topic.
  /// \param[in] msg Trajectory msg
  void SetJointTrajectory(trajectory_msgs::msg::JointTrajectory::SharedPtr _msg);

  rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);

  bool executeTrajectory(const trajectory_msgs::msg::JointTrajectory& msg);

  rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

  void acceptedCallback(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);


  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to Trajectory messages.
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;

  // action server
  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr ac_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// Pointer to world.
  gazebo::physics::WorldPtr world_;

  /// Pointer to link wrt which the trajectory is set.
  gazebo::physics::LinkPtr reference_link_;

  /// Joints for setting the trajectory.
  std::vector<gazebo::physics::JointPtr> joints_;

  /// Command trajectory points
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points_;

  /// Period in seconds
  double update_period_;

  /// Keep last time an update was published
  gazebo::common::Time last_update_time_;

  /// Desired trajectory start time
  gazebo::common::Time trajectory_start_time_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Index number of trajectory to be executed
  unsigned int trajectory_index_;

  /// True if trajectory is available
  std::atomic<bool> has_trajectory_;
  std::mutex result_mtx;
  control_msgs::action::FollowJointTrajectory::Result result_;

  std::shared_ptr<GoalHandleFollowJointTrajectory> gh_ = nullptr;

  /// Pointer to the update event connection.
  gazebo::event::ConnectionPtr update_connection_;
};

JointPoseTrajectory::JointPoseTrajectory()
: impl_(std::make_unique<JointPoseTrajectoryPrivate>())
{
}

JointPoseTrajectory::~JointPoseTrajectory()
{
}

void JointPoseTrajectory::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  impl_->model_ = model;

  impl_->world_ = model->GetWorld();

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Update rate
  auto update_rate = sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = impl_->world_->SimTime();

  // Set Joint Trajectory Callback
  impl_->sub_ = impl_->ros_node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "set_joint_trajectory", rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(&JointPoseTrajectoryPrivate::SetJointTrajectory,
    impl_.get(), std::placeholders::_1));

  impl_->ac_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
    impl_->ros_node_->get_node_base_interface(),
    impl_->ros_node_->get_node_clock_interface(),
    impl_->ros_node_->get_node_logging_interface(),
    impl_->ros_node_->get_node_waitables_interface(),
    FOLLOW_JOINT_TRAJECTORY_ACTION,
    std::bind(&JointPoseTrajectoryPrivate::goalCallback, impl_.get(), std::placeholders::_1, std::placeholders::_2),
    std::bind(&JointPoseTrajectoryPrivate::cancelCallback, impl_.get(), std::placeholders::_1),
    std::bind(&JointPoseTrajectoryPrivate::acceptedCallback, impl_.get(), std::placeholders::_1));

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&JointPoseTrajectoryPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

rclcpp_action::GoalResponse JointPoseTrajectoryPrivate::goalCallback(const rclcpp_action::GoalUUID & uuid,
	    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
  if(has_trajectory_)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointPoseTrajectoryPrivate::cancelCallback(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointPoseTrajectoryPrivate::acceptedCallback(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  gh_ = std::shared_ptr<GoalHandleFollowJointTrajectory>(goal_handle);
  std::thread(std::bind(&JointPoseTrajectoryPrivate::executeTrajectory, this, std::placeholders::_1),
    std::cref(goal_handle->get_goal()->trajectory)).detach();
}

void JointPoseTrajectoryPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  gazebo::common::Time current_time = info.simTime;

  // If the world is reset, for example
  if (current_time < last_update_time_) {
    RCLCPP_INFO(ros_node_->get_logger(), "Negative sim time difference detected.");
    last_update_time_ = current_time;
  }

  // Check period
  double seconds_since_last_update = (current_time - last_update_time_).Double();

  if (seconds_since_last_update < update_period_) {
    return;
  }

  std::lock_guard<std::mutex> scoped_lock(lock_);
  if (has_trajectory_ && current_time >= trajectory_start_time_) {
    if (trajectory_index_ < points_.size()) {
      RCLCPP_INFO(ros_node_->get_logger(), "time [%f] updating configuration [%d/%lu]",
        current_time.Double(), trajectory_index_ + 1, points_.size());

      // get reference link pose before updates
      auto reference_pose = model_->WorldPose();

      if (reference_link_) {
        reference_pose = reference_link_->WorldPose();
      }

      // trajectory roll-out based on time:
      // set model configuration from trajectory message
      auto chain_size = static_cast<unsigned int>(joints_.size());
      if (chain_size == points_[trajectory_index_].positions.size()) {
        for (unsigned int i = 0; i < chain_size; ++i) {
          // this is not the most efficient way to set things
          if (joints_[i]) {
            joints_[i]->SetPosition(0, points_[trajectory_index_].positions[i], true);
          }
        }
        // set model pose
        if (reference_link_) {
          model_->SetLinkWorldPose(reference_pose, reference_link_);
        } else {
          model_->SetWorldPose(reference_pose);
        }
      } else {
        std::string err_msg = boost::str(
            boost::format("point[%u] has different number of joint names[%u] and positions[%lu].") %
            (trajectory_index_ + 1) %  chain_size %  points_[trajectory_index_].positions.size());
        RCLCPP_ERROR(ros_node_->get_logger(),err_msg.c_str());
        {
          std::lock_guard<std::mutex> lock(result_mtx);
          result_.error_code = result_.INVALID_JOINTS;
          result_.error_string = err_msg;
          return;
        }

      }

      auto duration =
        gazebo_ros::Convert<gazebo::common::Time>(points_[trajectory_index_].time_from_start);

      // reset start time for next trajectory point
      trajectory_start_time_ += duration;
      trajectory_index_++;  // increment to next trajectory point

      // Update time
      last_update_time_ = current_time;
    } else {
      // trajectory finished
      reference_link_.reset();
      // No more trajectory points
      has_trajectory_ = false;
    }
  }
}

void JointPoseTrajectoryPrivate::SetJointTrajectory(
  trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
	executeTrajectory(*msg);
}

bool JointPoseTrajectoryPrivate::executeTrajectory(const trajectory_msgs::msg::JointTrajectory& msg)
{
  using namespace control_msgs::action;
	std::lock_guard<std::mutex> scoped_lock(lock_);

	std::string reference_link_name = msg.header.frame_id;
	// do this every time a new joint trajectory is supplied,
	// use header.frame_id as the reference_link_name_
	if (!(reference_link_name == "world" || reference_link_name == "map"))
	{
	  gazebo::physics::EntityPtr entity = world_->EntityByName(reference_link_name);
    if (entity)
    {
      reference_link_ = boost::dynamic_pointer_cast<gazebo::physics::Link>(entity);
    }

    if (!reference_link_)
    {
      RCLCPP_ERROR(ros_node_->get_logger(),
      "Plugin needs a reference link [%s] as frame_id, aborting.", reference_link_name.c_str());
      return false;
    }
    model_ = reference_link_->GetParentModel();
    RCLCPP_DEBUG(ros_node_->get_logger(),
      "Update model pose by keeping link [%s] stationary inertially",
      reference_link_->GetName().c_str());
	}

	// copy joint configuration into a map
	auto chain_size = static_cast<unsigned int>(msg.joint_names.size());
	joints_.resize(chain_size);
	for (unsigned int i = 0; i < chain_size; ++i)
	{
    joints_[i] = model_->GetJoint(msg.joint_names[i]);
    if (!joints_[i])
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Joint [%s] not found. Trajectory not set.",
      msg.joint_names[i].c_str());
      return false;
    }
	}

	auto points_size = static_cast<unsigned int>(msg.points.size());
	points_.resize(points_size);
	for (unsigned int i = 0; i < points_size; ++i)
	{
		points_[i].positions.resize(chain_size);
		points_[i].time_from_start = msg.points[i].time_from_start;
		for (unsigned int j = 0; j < chain_size; ++j)
		{
		  points_[i].positions[j] = msg.points[i].positions[j];
		}
	}

	// trajectory start time
	trajectory_start_time_ = gazebo_ros::Convert<gazebo::common::Time>(msg.header.stamp);

	gazebo::common::Time cur_time = world_->SimTime();
	if (trajectory_start_time_ < cur_time)
	{
	  trajectory_start_time_ = cur_time;
	}

	// update the joint trajectory to play
	has_trajectory_ = true;
	// reset trajectory_index to beginning of new trajectory
	trajectory_index_ = 0;

	std::chrono::duration<double> check_pause(this->update_period_);
	result_.error_code = result_.SUCCESSFUL;

  decltype(result_.error_code) error_code;
	while(rclcpp::ok())
	{
	  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(check_pause));
	  if(!has_trajectory_)
	  {
	    break;
	  }

	  // getting error code
	  {
	    std::lock_guard<std::mutex> lock(result_mtx);
	    error_code = result_.error_code;
	  }

	  if(error_code != result_.SUCCESSFUL)
	  {
      std::shared_ptr<FollowJointTrajectory::Result> res;
      {
        std::lock_guard<std::mutex> lock(result_mtx);
        res = std::make_shared<FollowJointTrajectory::Result>(result_);
      }
      has_trajectory_ = false;
      gh_->abort(res);
      return false;
	  }

	  if(gh_->is_canceling() && gh_->is_executing())
	  {
	    std::shared_ptr<FollowJointTrajectory::Result> res = std::make_shared<FollowJointTrajectory::Result>();
	    has_trajectory_ = false;
	    res->error_code = res->INVALID_GOAL;
	    res->error_string ="action canceled";
	    gh_->canceled(res);
	    return false;
	  }
	}

  std::shared_ptr<FollowJointTrajectory::Result> res = std::make_shared<FollowJointTrajectory::Result>();
	if(gh_->is_executing() && trajectory_index_ >= points_size - 1)
	{
    has_trajectory_ = false;
    res->error_code = res->SUCCESSFUL;
	  gh_->succeed(res);
	}
	else
	{
	  res->error_code = res->PATH_TOLERANCE_VIOLATED;
	  res->error_string = "unknown error, didn't complete trajectory";
	  gh_->abort(res);
	}
	return true;
}

GZ_REGISTER_MODEL_PLUGIN(JointPoseTrajectory)
}  // namespace crs_gazebo_plugins
