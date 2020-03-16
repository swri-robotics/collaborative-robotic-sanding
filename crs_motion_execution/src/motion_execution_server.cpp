#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>

#include <crs_msgs/action/cartesian_compliance_trajectory.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <tesseract/tesseract.h>
#include <tesseract_common/types.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_rosutils/utils.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_storage.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

static const std::string RESOURCES_PACKAGE_NAME = "crs_support";
static const std::string DEFAULT_URDF_PATH = "urdf/crs.urdf";
static const std::string DEFAULT_SRDF_PATH = "urdf/ur10e_robot.srdf";
static const std::string ENVIRONMENT_UPDATE_TOPIC_NAME = "monitored_tesseract";
static const std::string ENVIRONMENT_ID = "crs";

static const std::string POSITION_COMPLIANCE_TOPIC = "position_controllers/CartesianComplianceController";
static const std::string VELOCITY_COMPLIANCE_TOPIC = "velocity_interface/CartesianComplianceController";
static const std::string DESIRED_WRENCH_TOPIC = "my_cartesian_compliance_controller/target_wrench";
static const std::string MOTION_EXECUTION_ACTION_TOPIC = "execute_surface_motion";

namespace param_names
{
static const std::string URDF_PATH = "urdf_path";
static const std::string SRDF_PATH = "srdf_path";
static const std::string ROOT_LINK_FRAME = "base_link_frame";
static const std::string WORLD_FRAME = "world_frame";
static const std::string TOOL0_FRAME = "tool0_frame";
static const std::string TCP_FRAME = "tcp_frame";
static const std::string MANIPULATOR_GROUP = "manipulator_group";
}  // namespace param_names

void findCartPoseArrayFromTraj(const trajectory_msgs::msg::JointTrajectory& joint_trajectory,
                               const tesseract::Tesseract::Ptr tesseract_local,
                               const std::string manipulator,
                               geometry_msgs::msg::PoseArray& cartesian_poses)
{
    const std::shared_ptr<const tesseract_environment::Environment> env = tesseract_local->getEnvironmentConst();
    tesseract_common::TransformMap curr_transforms = env->getCurrentState()->transforms;

    tesseract_kinematics::ForwardKinematics::ConstPtr kin =
        tesseract_local->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manipulator);

    for (auto joint_pose : joint_trajectory.points)
    {
        Eigen::VectorXd joint_positions(joint_pose.positions.size());
        for (size_t i = 0; i < joint_pose.positions.size(); i++)
        {
            joint_positions[static_cast<int>(i)] = joint_pose.positions[i];
        }
        Eigen::Isometry3d eig_pose;
        kin->calcFwdKin(eig_pose, joint_positions);
        Eigen::Isometry3d world_to_base_link;
        world_to_base_link = curr_transforms.find("base_link")->second;
        eig_pose = world_to_base_link * eig_pose;
        geometry_msgs::msg::Pose curr_cart_pose = tf2::toMsg(eig_pose);
        cartesian_poses.poses.push_back(curr_cart_pose);
    }
    cartesian_poses.header.frame_id = "world";
}

class MotionExecutionServer : public rclcpp::Node
{
public:
  using ProcessExecution = crs_msgs::action::CartesianComplianceTrajectory;
  using ServerGoalHandleProcessExecution = rclcpp_action::ServerGoalHandle<ProcessExecution>;

  explicit MotionExecutionServer()
  : Node("ComplianceMotionExecutionServer"),
    clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)),
    tf_buffer_(clock_),
    tf_listener_(tf_buffer_)
  {
    namespace fs = boost::filesystem;

    // ROS parameters
    this->declare_parameter(
        param_names::URDF_PATH,
        (fs::path(ament_index_cpp::get_package_share_directory(RESOURCES_PACKAGE_NAME)) / fs::path(DEFAULT_URDF_PATH))
            .string());

    this->declare_parameter(
        param_names::SRDF_PATH,
        (fs::path(ament_index_cpp::get_package_share_directory(RESOURCES_PACKAGE_NAME)) / fs::path(DEFAULT_SRDF_PATH))
            .string());

    this->declare_parameter(param_names::ROOT_LINK_FRAME, "base_link");
    this->declare_parameter(param_names::WORLD_FRAME, "world");
    this->declare_parameter(param_names::TOOL0_FRAME, "tool0");
    this->declare_parameter(param_names::TCP_FRAME, "sander_center_link");
    this->declare_parameter(param_names::MANIPULATOR_GROUP, "manipulator");

    action_server_ = rclcpp_action::create_server<ProcessExecution>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      MOTION_EXECUTION_ACTION_TOPIC,
      std::bind(&MotionExecutionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MotionExecutionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MotionExecutionServer::handle_accepted, this, std::placeholders::_1));

    compliance_position_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(POSITION_COMPLIANCE_TOPIC, 1);
    compliance_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(VELOCITY_COMPLIANCE_TOPIC, 1);
    compliance_wrench_publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(DESIRED_WRENCH_TOPIC, 1);

    env_state_sub_ = this->create_subscription<tesseract_msgs::msg::TesseractState>(
        ENVIRONMENT_UPDATE_TOPIC_NAME, 10, std::bind(&MotionExecutionServer::envCallback, this, std::placeholders::_1));


    // openning files
    std::string urdf_path, srdf_path;
    urdf_path = this->get_parameter(param_names::URDF_PATH).as_string();
    srdf_path = this->get_parameter(param_names::SRDF_PATH).as_string();
    std::vector<std::string> file_paths = { urdf_path, srdf_path };
    std::vector<std::string> file_string_contents;
    for (const auto& f : file_paths)
    {
      std::ifstream ifs(f);
      if (!ifs.is_open())
      {
        throw std::runtime_error(boost::str(boost::format("File '%s' could not be opened") % f));
      }
      std::stringstream ss;
      ss << ifs.rdbuf();
      file_string_contents.push_back(ss.str());
    }

    const std::string urdf_content = file_string_contents[0];
    const std::string srdf_content = file_string_contents[1];

    tesseract_local_ = std::make_shared<tesseract::Tesseract>();
    tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    tesseract_local_->init(urdf_content, srdf_content, locator);

    world_frame_ = this->get_parameter(param_names::WORLD_FRAME).as_string();
    robot_base_frame_ = this->get_parameter(param_names::ROOT_LINK_FRAME).as_string();
    tool0_frame_ = this->get_parameter(param_names::TOOL0_FRAME).as_string();
    tcp_frame_ = this->get_parameter(param_names::TCP_FRAME).as_string();
    manipulator_ = this->get_parameter(param_names::MANIPULATOR_GROUP).as_string();
  }

private:

  void envCallback(const tesseract_msgs::msg::TesseractState::SharedPtr msg)
  {
    const tesseract_environment::Environment::Ptr env = tesseract_local_->getEnvironment();
    if (!tesseract_rosutils::processMsg(env, *msg))
      RCLCPP_ERROR(this->get_logger(), "Failed to update local Tesseract state");
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ProcessExecution::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal trajectory to execute");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<ServerGoalHandleProcessExecution> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute(const std::shared_ptr<ServerGoalHandleProcessExecution> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Rate loop_rate(1);

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ProcessExecution::Feedback>();
    auto result = std::make_shared<ProcessExecution::Result>();

    trajectory_msgs::msg::JointTrajectory joint_traj = goal->trajectory;
    double speed = goal->speed;
    double force = goal->force;
    double tolerance = goal->tolerance;
    geometry_msgs::msg::PoseArray cart_pose_array;

    findCartPoseArrayFromTraj(joint_traj, tesseract_local_, manipulator_, cart_pose_array);

    const std::shared_ptr<const tesseract_environment::Environment> env = tesseract_local_->getEnvironmentConst();
    tesseract_common::TransformMap curr_transforms = env->getCurrentState()->transforms;
    Eigen::Isometry3d world_to_base_link, world_to_sander, world_to_tool0, tool0_to_sander;
    world_to_base_link = curr_transforms.find(robot_base_frame_)->second;
    world_to_sander = curr_transforms.find(tcp_frame_)->second;
    world_to_tool0 = curr_transforms.find(tool0_frame_)->second;
    tool0_to_sander = world_to_tool0.inverse() * world_to_sander;

    Eigen::Isometry3d force_vector = Eigen::Isometry3d::Identity();
    force_vector.translation().z() = force;
    force_vector = force_vector * tool0_to_sander;
    geometry_msgs::msg::WrenchStamped wrench_msg;
    wrench_msg.header.frame_id = tool0_frame_;
    wrench_msg.wrench.force.x = force_vector.translation().x();
    wrench_msg.wrench.force.y = force_vector.translation().y();
    wrench_msg.wrench.force.z = force_vector.translation().z();

    compliance_wrench_publisher_->publish(wrench_msg); // Publish requested force
    int count = 1;

    for (auto pose : cart_pose_array.poses)
    {
      std::cout << "Targetting point number: " << count << std::endl;
      geometry_msgs::msg::PoseStamped curr_cart_goal;
      curr_cart_goal.header.frame_id = world_frame_;
      curr_cart_goal.pose = pose;
      feedback->waypoint = curr_cart_goal;
      feedback->status = "Executing";
      goal_handle->publish_feedback(feedback);
      geometry_msgs::msg::TransformStamped world_to_sander_tf;
      try
      {
        world_to_sander_tf = tf_buffer_.lookupTransform(world_frame_, tcp_frame_, tf2::TimePointZero, tf2::Duration(std::chrono::seconds(2)));
      }
      catch (tf2::LookupException &e)
      {
        geometry_msgs::msg::WrenchStamped zero_wrench;
        zero_wrench.header.frame_id = tool0_frame_;
        compliance_wrench_publisher_->publish(zero_wrench);
        result->err_msg = "Failed TF Lookup";
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      Eigen::Isometry3d world_to_sander_eig, curr_cart_goal_eig;
      geometry_msgs::msg::Pose world_to_sander_pose;
      world_to_sander_eig = tf2::transformToEigen(world_to_sander_tf.transform);
      tf2::fromMsg(curr_cart_goal.pose, curr_cart_goal_eig);
      std::cout << "GOAL:\n" << curr_cart_goal_eig.matrix() << "\nCURR:\n" << world_to_sander_eig.matrix() << std::endl;
      double diff = (curr_cart_goal_eig.translation() - world_to_sander_eig.translation()).norm();
      std::cout << "DIFF: " << diff << std::endl;
      while (diff > tolerance)
      {
        compliance_position_publisher_->publish(curr_cart_goal);
        try
        {
          world_to_sander_tf = tf_buffer_.lookupTransform(world_frame_, tcp_frame_, tf2::TimePointZero, tf2::Duration(std::chrono::seconds(2)));
        }
        catch (tf2::LookupException &e)
        {
          geometry_msgs::msg::WrenchStamped zero_wrench;
          zero_wrench.header.frame_id = tool0_frame_;
          compliance_wrench_publisher_->publish(zero_wrench);
          result->err_msg = "Failed TF Lookup";
          result->success = false;
          goal_handle->abort(result);
          return;
        }
        world_to_sander_eig = tf2::transformToEigen(world_to_sander_tf.transform);
        diff = (curr_cart_goal_eig = world_to_sander_eig).translation().norm();
      }
      count++;
    }
    wrench_msg.wrench.force.x = -force_vector.translation().x();
    wrench_msg.wrench.force.y = -force_vector.translation().y();
    wrench_msg.wrench.force.z = -force_vector.translation().z();

    compliance_wrench_publisher_->publish(wrench_msg); // Publish inverse of requested force to make robot lift up

    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }

  void handle_accepted(const std::shared_ptr<ServerGoalHandleProcessExecution> goal_handle)
  {
    std::thread{std::bind(&MotionExecutionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  rclcpp_action::Server<ProcessExecution>::SharedPtr action_server_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr compliance_position_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr compliance_velocity_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr compliance_wrench_publisher_;

  rclcpp::Subscription<tesseract_msgs::msg::TesseractState>::SharedPtr env_state_sub_;

  tesseract::Tesseract::Ptr tesseract_local_;

  std::string world_frame_, robot_base_frame_, tool0_frame_, tcp_frame_, manipulator_;

  std::shared_ptr<rclcpp::Clock> clock_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};  // class MotionExecutionServer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<MotionExecutionServer>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
