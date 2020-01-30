#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <rclcpp/rclcpp.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <crs_msgs/srv/call_freespace_motion.hpp>

#include <tf2/transform_storage.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_default_config.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_freespace_config.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>

#include <trajopt/problem_description.hpp>

#include <tesseract_rosutils/utils.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

static const std::vector<double> DEFAULT_COEFFICIENTS {10, 10, 10, 10, 10, 10};
static const std::string RESOURCES_PACKAGE_NAME = "crs_support";
static const std::string DEFAULT_URDF_PATH = "urdf/crs_v3.urdf";
static const std::string DEFAULT_SRDF_PATH = "urdf/ur10e_robot.srdf";

static const std::string DEFAULT_FREESPACE_MOTION_PLANNING_SERVICE = "plan_freespace_motion";
static const std::string DEFAULT_FREESPACE_TRAJECTORY_TOPIC = "planned_freespace_trajectory";
static const std::string JOINT_STATES_TOPIC = "joint_states";

namespace param_names
{
  static const std::string URDF_PATH = "urdf_path";
  static const std::string SRDF_PATH = "srdf_path";
  static const std::string ROOT_LINK_FRAME = "base_link_frame";
  static const std::string MANIPULATOR_GROUP = "manipulator_group";
  static const std::string NUM_STEPS = "num_steps";
  static const std::string CART_WEIGHT_COEFFS = "cartesian_coeffs";
  static const std::string FREESPACE_MOTION_PLANNING_SERVICE = "freespace_motion_service";
  static const std::string FREESPACE_TRAJECTORY_TOPIC = "trajectory_topic";
}

void tesseractRosutilsToMsg(trajectory_msgs::msg::JointTrajectory& traj_msg,
                              const std::vector<std::string>& joint_names,
                              const Eigen::Ref<const tesseract_common::TrajArray>& traj)
{
  assert(joint_names.size() == static_cast<unsigned>(traj.cols()));

  // Initialze the whole traject with the current state.
  std::map<std::string, int> jn_to_index;
  traj_msg.joint_names.resize(joint_names.size());
  traj_msg.points.resize(static_cast<size_t>(traj.rows()));

  for (int i = 0; i < traj.rows(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint jtp;
    jtp.positions.resize(static_cast<size_t>(traj.cols()));

    for (int j = 0; j < traj.cols(); ++j)
    {
      if (i == 0)
        traj_msg.joint_names[static_cast<size_t>(j)] = joint_names[static_cast<size_t>(j)];

      jtp.positions[static_cast<size_t>(j)] = traj(i, j);
    }

    jtp.time_from_start = rclcpp::Duration(i, 0);
    traj_msg.points[static_cast<size_t>(i)] = jtp;
  }
}

class PlanningServer: public rclcpp::Node
{
public:
  PlanningServer()
    : Node("planning_server_node")
  {
    namespace fs = boost::filesystem;

    // ROS parameters
    this->declare_parameter(param_names::URDF_PATH, (fs::path(ament_index_cpp::get_package_share_directory(RESOURCES_PACKAGE_NAME)) /
       fs::path(DEFAULT_URDF_PATH)).string());

    this->declare_parameter(param_names::SRDF_PATH, (fs::path(ament_index_cpp::get_package_share_directory(RESOURCES_PACKAGE_NAME)) /
                            fs::path(DEFAULT_SRDF_PATH)).string());

    this->declare_parameter(param_names::ROOT_LINK_FRAME, "world");
    this->declare_parameter(param_names::MANIPULATOR_GROUP, "manipulator");
    this->declare_parameter(param_names::NUM_STEPS, 200);
    this->declare_parameter(param_names::CART_WEIGHT_COEFFS, DEFAULT_COEFFICIENTS);
    this->declare_parameter(param_names::FREESPACE_MOTION_PLANNING_SERVICE, DEFAULT_FREESPACE_MOTION_PLANNING_SERVICE);
    this->declare_parameter(param_names::FREESPACE_TRAJECTORY_TOPIC, DEFAULT_FREESPACE_TRAJECTORY_TOPIC);


    std::string freespace_motion_service = this->get_parameter(param_names::FREESPACE_MOTION_PLANNING_SERVICE).as_string();
    std::string freespace_trajectory_topic = this->get_parameter(param_names::FREESPACE_TRAJECTORY_TOPIC).as_string();

    // ROS communications
    plan_service_ = this->create_service<crs_msgs::srv::CallFreespaceMotion>(freespace_motion_service,
                                                                             std::bind(&PlanningServer::planService,
                                                                             this, std::placeholders::_1, std::placeholders::_2));
    traj_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(freespace_trajectory_topic,10);
    joint_state_listener_ = this->create_subscription<sensor_msgs::msg::JointState>(JOINT_STATES_TOPIC, 2,
                                                                              std::bind(&PlanningServer::jointCallback,
                                                                              this, std::placeholders::_1));

    // openning files
    std::string urdf_path, srdf_path;
    urdf_path = this->get_parameter(param_names::URDF_PATH).as_string();
    srdf_path = this->get_parameter(param_names::SRDF_PATH).as_string();
    std::vector<std::string> file_paths = {urdf_path, srdf_path};
    std::vector<std::string> file_string_contents;
    for(const auto& f : file_paths)
    {
      std::ifstream ifs(f);
      if(!ifs.is_open())
      {
        throw std::runtime_error(boost::str( boost::format("File '%s' could not be opened" ) % f));
      }
      std::stringstream ss;
      ss << ifs.rdbuf();
      file_string_contents.push_back(ss.str());
    }

    const std::string urdf_content = file_string_contents[0];
    const std::string srdf_content = file_string_contents[1];

    // initializing planning
    tesseract_local_ = std::make_shared<tesseract::Tesseract>();
    tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    tesseract_local_->init(urdf_content, srdf_content, locator);

    base_link_frame_ = this->get_parameter(param_names::ROOT_LINK_FRAME).as_string();
    manipulator_ = this->get_parameter(param_names::MANIPULATOR_GROUP).as_string();
    default_num_steps_ = this->get_parameter(param_names::NUM_STEPS).as_int();
    cart_weight_coeffs_ = this->get_parameter(param_names::CART_WEIGHT_COEFFS).as_double_array();

    // verifying joints
    std::vector<std::string> joint_names = tesseract_local_->getFwdKinematicsManagerConst()->getFwdKinematicSolver(
        manipulator_)->getJointNames();
    if(cart_weight_coeffs_.size() < 6)
    {
      throw std::runtime_error(boost::str(boost::format("cart coeffs array size (%lu) is less than 6") %
                                          cart_weight_coeffs_.size() ));
    }

  }
private:
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr joint_msg)
  {
    curr_joint_state_ = *joint_msg;
  }
  void planService(std::shared_ptr<crs_msgs::srv::CallFreespaceMotion::Request> request,
                    std::shared_ptr<crs_msgs::srv::CallFreespaceMotion::Response> response)
  {
    std::string target_frame = request->target_link;

    Eigen::Isometry3d tcp_eigen;
    tcp_eigen.setIdentity();
    auto traj_pc = std::make_shared<tesseract_motion_planners::TrajOptPlannerFreespaceConfig>(tesseract_local_, manipulator_, target_frame, tcp_eigen);

    std::vector<std::string> joint_names = tesseract_local_->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manipulator_)->getJointNames();
    // Initialize vector of target waypoints
    std::vector<tesseract_motion_planners::Waypoint::Ptr> trgt_wypts;

    // Define initial waypoint
    tesseract_motion_planners::JointWaypoint::Ptr joint_init_waypoint;
    if(request->start_position.position.empty())
    {
      //use current position when it was not specified in the request
      joint_init_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(
          curr_joint_state_.position, curr_joint_state_.name);
    }
    else
    {
      if(request->start_position.position.size() < joint_names.size())
      {
        response->message = boost::str(boost::format("start position (%lu) has fewer joints than the required number of %lu") %
                                       request->start_position.position.size() % joint_names.size());
        response->success = false;
        RCLCPP_ERROR(this->get_logger(),response->message.c_str());
        return;
      }
      joint_init_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(
          request->start_position.position, request->start_position.name);
    }

    joint_init_waypoint->setIsCritical(true);

    // Define goal waypoint
    tesseract_motion_planners::Waypoint::Ptr goal_waypoint;
    if(request->goal_position.position.empty())
    {
      Eigen::Vector3d goal_pose(request->goal_pose.translation.x, request->goal_pose.translation.y, request->goal_pose.translation.z);
      Eigen::Quaterniond goal_ori(request->goal_pose.rotation.w, request->goal_pose.rotation.x, request->goal_pose.rotation.y, request->goal_pose.rotation.z);
      tesseract_motion_planners::CartesianWaypoint::Ptr cart_goal_waypoint = std::make_shared<tesseract_motion_planners::CartesianWaypoint>(goal_pose, goal_ori);

      cart_goal_waypoint->setIsCritical(false);

      Eigen::VectorXd coeffs = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(cart_weight_coeffs_.data(), 6);
      cart_goal_waypoint->setCoefficients(coeffs);
      goal_waypoint = cart_goal_waypoint;
      RCLCPP_INFO(this->get_logger(),"Planning FreeSpace motion to cartesian goal");
    }
    else
    {
      goal_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(
                request->goal_position.position, request->goal_position.name);
      RCLCPP_INFO(this->get_logger(),"Planning FreeSpace motion to joint goal");
    }

    // Add waypoints
    trgt_wypts.push_back(joint_init_waypoint);
    trgt_wypts.push_back(goal_waypoint);

    // Write target waypoints to trajopt planning config
    traj_pc->target_waypoints = trgt_wypts;

    // Various freespace configuration settings
    traj_pc->num_steps = request->num_steps == 0 ? default_num_steps_ :  request->num_steps;;
    traj_pc->smooth_velocities = true;
    traj_pc->smooth_accelerations = true;
    traj_pc->smooth_jerks = true;
    traj_pc->longest_valid_segment_length = 0.05;
    traj_pc->init_type = trajopt::InitInfo::STATIONARY;

    // TODO: Make these values into configurable parameters read from yaml
    tesseract_motion_planners::CollisionConstraintConfig tesseract_collision_const_config;
    tesseract_collision_const_config.safety_margin = 0.01;
    tesseract_collision_const_config.coeff = 20;
    tesseract_collision_const_config.enabled = true;
    traj_pc->collision_constraint_config = tesseract_collision_const_config;
    tesseract_motion_planners::CollisionCostConfig tesseract_collision_cost_config;
    tesseract_collision_cost_config.enabled = false;
    traj_pc->collision_cost_config = tesseract_collision_cost_config;

    // Setup and solve trajopt motion plan
    tesseract_motion_planners::TrajOptMotionPlanner traj_motion_planner;
    tesseract_motion_planners::PlannerResponse plan_resp;
    traj_motion_planner.setConfiguration(traj_pc);
    traj_motion_planner.solve(plan_resp, false);

    // Store generated trajectory
    tesseract_common::TrajArray traj_array = plan_resp.joint_trajectory.trajectory;
    Eigen::MatrixXd cumulative_trajectory(traj_array.rows(), traj_array.cols());
    cumulative_trajectory << traj_array;

    // Convert trajectory to ROSmsg
    trajectory_msgs::msg::JointTrajectory cumulative_joint_trajectory;
    tesseractRosutilsToMsg(cumulative_joint_trajectory, joint_names, cumulative_trajectory);
    cumulative_joint_trajectory.header.frame_id = base_link_frame_;

    // Modify time
    for (int i = 0; i < traj_pc->num_steps; ++i)
    {
      cumulative_joint_trajectory.points[static_cast<size_t>(i)].time_from_start.sec = 0;
      cumulative_joint_trajectory.points[static_cast<size_t>(i)].time_from_start.nanosec = 0;//2e8;
    }

    response->output_trajectory = cumulative_joint_trajectory;
    response->message = plan_resp.status.message();
    if (plan_resp.status.value() == 0)
    {
      response->success = true;
    }
    else
    {
      response->success = false;
      RCLCPP_ERROR(this->get_logger(),response->message.c_str());
    }

    if (request->execute && response->success)
    {
      // Publish trajectory if desired
      traj_publisher_->publish(cumulative_joint_trajectory);
    }
  }

  rclcpp::Service<crs_msgs::srv::CallFreespaceMotion>::SharedPtr plan_service_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_listener_;
  tesseract::Tesseract::Ptr tesseract_local_;

  sensor_msgs::msg::JointState curr_joint_state_;

  std::string base_link_frame_;
  std::string manipulator_;
  int default_num_steps_;
  std::vector<double> cart_weight_coeffs_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningServer>());
  rclcpp::shutdown();
  return 0;
}
