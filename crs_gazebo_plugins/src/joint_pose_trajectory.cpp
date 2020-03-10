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
  void OnUpdate(const gazebo::common::UpdateInfo& info);

  /// \brief Callback for set joint trajectory topic.
  /// \param[in] msg Trajectory msg

  bool executeTrajectory(const std::shared_ptr<GoalHandleFollowJointTrajectory> gh);

  void publishFeedback(const std::shared_ptr<GoalHandleFollowJointTrajectory> gh);

  rclcpp_action::GoalResponse
  goalCallback(const rclcpp_action::GoalUUID& uuid,
               std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);

  rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

  void acceptedCallback(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  // action server
  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr ac_;
  rclcpp::callback_group::CallbackGroup::SharedPtr ac_cbgroup_;

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

  /// Pointer to the update event connection.
  gazebo::event::ConnectionPtr update_connection_;
};

JointPoseTrajectory::JointPoseTrajectory() : impl_(std::make_unique<JointPoseTrajectoryPrivate>()) {}

JointPoseTrajectory::~JointPoseTrajectory() {}

void JointPoseTrajectory::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  impl_->model_ = model;

  impl_->world_ = model->GetWorld();

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  impl_->has_trajectory_ = false;

  // Update rate
  auto update_rate = sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0)
  {
    impl_->update_period_ = 1.0 / update_rate;
  }
  else
  {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = impl_->world_->SimTime();

  // creating action interface
  impl_->ac_cbgroup_ =
      impl_->ros_node_->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  impl_->ac_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
      impl_->ros_node_->get_node_base_interface(),
      impl_->ros_node_->get_node_clock_interface(),
      impl_->ros_node_->get_node_logging_interface(),
      impl_->ros_node_->get_node_waitables_interface(),
      FOLLOW_JOINT_TRAJECTORY_ACTION,
      std::bind(&JointPoseTrajectoryPrivate::goalCallback, impl_.get(), std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointPoseTrajectoryPrivate::cancelCallback, impl_.get(), std::placeholders::_1),
      std::bind(&JointPoseTrajectoryPrivate::acceptedCallback, impl_.get(), std::placeholders::_1),
      rcl_action_server_get_default_options(),
      impl_->ac_cbgroup_);

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&JointPoseTrajectoryPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

rclcpp_action::GoalResponse
JointPoseTrajectoryPrivate::goalCallback(const rclcpp_action::GoalUUID& uuid,
                                         std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
  RCLCPP_INFO(ros_node_->get_logger(), "Received Trajectory goal %s", rclcpp_action::to_string(uuid).c_str());
  if (has_trajectory_)
  {
    RCLCPP_ERROR(ros_node_->get_logger(), "Goal Rejected");
    return rclcpp_action::GoalResponse::REJECT;
  }
  has_trajectory_ = true;
  RCLCPP_INFO(ros_node_->get_logger(), "Goal Accepted");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
JointPoseTrajectoryPrivate::cancelCallback(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  has_trajectory_ = false;
  RCLCPP_WARN(ros_node_->get_logger(),
              "Cancel requested for goal %s",
              rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointPoseTrajectoryPrivate::acceptedCallback(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread

  std::thread(std::bind(&JointPoseTrajectoryPrivate::executeTrajectory, this, std::placeholders::_1), goal_handle)
      .detach();
}

void JointPoseTrajectoryPrivate::OnUpdate(const gazebo::common::UpdateInfo& info)
{
  gazebo::common::Time current_time = info.simTime;

  // If the world is reset, for example
  if (current_time < last_update_time_)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Negative sim time difference detected.");
    last_update_time_ = current_time;
  }

  // Check period
  double seconds_since_last_update = (current_time - last_update_time_).Double();

  if (seconds_since_last_update < update_period_)
  {
    return;
  }

  std::lock_guard<std::mutex> scoped_lock(lock_);
  if (has_trajectory_ && current_time >= trajectory_start_time_)
  {
    if (trajectory_index_ < points_.size())
    {
      RCLCPP_DEBUG(ros_node_->get_logger(),
                   "time [%f] updating configuration [%d/%lu]",
                   current_time.Double(),
                   trajectory_index_ + 1,
                   points_.size());

      // get reference link pose before updates
      auto reference_pose = model_->WorldPose();

      if (reference_link_)
      {
        reference_pose = reference_link_->WorldPose();
      }

      // trajectory roll-out based on time:
      // set model configuration from trajectory message
      auto chain_size = static_cast<unsigned int>(joints_.size());
      if (chain_size == points_[trajectory_index_].positions.size())
      {
        RCLCPP_DEBUG(
            ros_node_->get_logger(), "Setting joint position %lu out of %lu", trajectory_index_, points_.size());
        for (unsigned int i = 0; i < chain_size; ++i)
        {
          // this is not the most efficient way to set things
          if (joints_[i])
          {
            joints_[i]->SetPosition(0, points_[trajectory_index_].positions[i], true);
          }
        }
        // set model pose
        if (reference_link_)
        {
          model_->SetLinkWorldPose(reference_pose, reference_link_);
        }
        else
        {
          model_->SetWorldPose(reference_pose);
        }
      }
      else
      {
        std::string err_msg =
            boost::str(boost::format("point[%u] has different number of joint names[%u] and positions[%lu].") %
                       (trajectory_index_ + 1) % chain_size % points_[trajectory_index_].positions.size());
        RCLCPP_ERROR(ros_node_->get_logger(), err_msg.c_str());
        {
          std::lock_guard<std::mutex> lock(result_mtx);
          result_.error_code = result_.INVALID_JOINTS;
          result_.error_string = err_msg;
          return;
        }
      }

      auto duration = gazebo_ros::Convert<gazebo::common::Time>(points_[trajectory_index_].time_from_start);

      // Subtracting from previous
      if (trajectory_index_ > 0)
      {
        duration -= gazebo_ros::Convert<gazebo::common::Time>(points_[trajectory_index_ - 1].time_from_start);
      }

      // reset start time for next trajectory point
      trajectory_start_time_ += duration;
      trajectory_index_++;  // increment to next trajectory point

      // Update time
      last_update_time_ = current_time;
    }
    else
    {
      // trajectory finished
      reference_link_.reset();
      // No more trajectory points
      has_trajectory_ = false;
    }
  }
}

void JointPoseTrajectoryPrivate::publishFeedback(const std::shared_ptr<GoalHandleFollowJointTrajectory> gh)
{
  using namespace control_msgs::action;
  if (trajectory_index_ >= points_.size())
  {
    RCLCPP_DEBUG(ros_node_->get_logger(), "trajectory index exceeded trajectory size");
    return;
  }

  std::shared_ptr<FollowJointTrajectory::Feedback> fb_ptr = std::make_shared<FollowJointTrajectory::Feedback>();
  auto& fb = *fb_ptr;
  fb.joint_names = gh->get_goal()->trajectory.joint_names;
  fb.desired.positions.resize(fb.joint_names.size(), 0);
  fb.desired.accelerations.resize(fb.joint_names.size());
  fb.desired.velocities.resize(fb.joint_names.size());
  fb.error = fb.desired;
  fb.desired.positions = points_[trajectory_index_].positions;
  fb.actual = fb.desired;
  gh->publish_feedback(fb_ptr);
}

bool JointPoseTrajectoryPrivate::executeTrajectory(const std::shared_ptr<GoalHandleFollowJointTrajectory> gh)
{
  using namespace control_msgs::action;
  const trajectory_msgs::msg::JointTrajectory& msg = gh->get_goal()->trajectory;
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
                   "Plugin needs a reference link [%s] as frame_id, aborting.",
                   reference_link_name.c_str());
      auto res = std::make_shared<FollowJointTrajectory::Result>();
      res->error_code = res->INVALID_GOAL;
      gh->abort(res);
      return false;
    }

    model_ = reference_link_->GetParentModel();
    RCLCPP_DEBUG(ros_node_->get_logger(),
                 "Update model pose by keeping link [%s] stationary inertially",
                 reference_link_->GetName().c_str());
  }

  // copy joint configuration into a map
  auto points_size = static_cast<unsigned int>(msg.points.size());
  {
    std::lock_guard<std::mutex> scoped_lock(lock_);
    auto chain_size = static_cast<unsigned int>(msg.joint_names.size());
    joints_.resize(chain_size);
    for (unsigned int i = 0; i < chain_size; ++i)
    {
      joints_[i] = model_->GetJoint(msg.joint_names[i]);
      if (!joints_[i])
      {
        RCLCPP_ERROR(ros_node_->get_logger(), "Joint [%s] not found. Trajectory not set.", msg.joint_names[i].c_str());
        auto res = std::make_shared<FollowJointTrajectory::Result>();
        res->error_code = res->INVALID_GOAL;
        gh->abort(res);
        return false;
      }
    }

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

  rclcpp::Duration traj_dur(msg.points.back().time_from_start);

  RCLCPP_INFO(ros_node_->get_logger(),
              "Executing trajectory with %lu points and duration of %f seconds",
              points_size,
              traj_dur.seconds());
  decltype(result_.error_code) error_code = result_.error_code;
  while (rclcpp::ok())
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(check_pause));
    if (!has_trajectory_)
    {
      break;
    }

    publishFeedback(gh);

    // getting error code
    {
      std::lock_guard<std::mutex> lock(result_mtx);
      error_code = result_.error_code;
    }

    if (error_code != result_.SUCCESSFUL)
    {
      std::shared_ptr<FollowJointTrajectory::Result> res;
      {
        std::lock_guard<std::mutex> lock(result_mtx);
        res = std::make_shared<FollowJointTrajectory::Result>(result_);
      }
      has_trajectory_ = false;
      gh->abort(res);
      RCLCPP_ERROR(ros_node_->get_logger(), "Failure %s", res->error_string.c_str());
      return false;
    }

    if (gh->is_canceling() && gh->is_executing())
    {
      std::shared_ptr<FollowJointTrajectory::Result> res = std::make_shared<FollowJointTrajectory::Result>();
      has_trajectory_ = false;
      res->error_code = res->INVALID_GOAL;
      res->error_string = "action canceled";
      gh->canceled(res);
      RCLCPP_ERROR(ros_node_->get_logger(), "Failure %s", res->error_string.c_str());
      return false;
    }
  }

  std::shared_ptr<FollowJointTrajectory::Result> res = std::make_shared<FollowJointTrajectory::Result>();
  if (gh->is_executing() && trajectory_index_ >= points_size - 1)
  {
    has_trajectory_ = false;
    res->error_code = res->SUCCESSFUL;
    RCLCPP_INFO(ros_node_->get_logger(), "Completed trajectory");
    gh->succeed(res);
  }
  else
  {
    res->error_code = res->PATH_TOLERANCE_VIOLATED;
    res->error_string = "unknown error, didn't complete trajectory";
    RCLCPP_ERROR(ros_node_->get_logger(), "Failure %s", res->error_string.c_str());
    gh->abort(res);
    return false;
  }
  return true;
}

GZ_REGISTER_MODEL_PLUGIN(JointPoseTrajectory)
}  // namespace crs_gazebo_plugins
