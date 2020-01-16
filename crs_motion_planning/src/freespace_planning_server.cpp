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

    jtp.time_from_start = rclcpp::Duration(i, 0);  // TODO: re-enable with rclcpp::Duration
    traj_msg.points[static_cast<size_t>(i)] = jtp;
  }
}

class PlanningServer: public rclcpp::Node
{
public:
  PlanningServer()
    : Node("planning_server_node")
  {
    plan_service_ = this->create_service<crs_msgs::srv::CallFreespaceMotion>("test_plan", std::bind(&PlanningServer::planService, this, std::placeholders::_1, std::placeholders::_2));
    traj_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("set_trajectory_test",10);
    joint_state_listener_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 2, std::bind(&PlanningServer::jointCallback, this, std::placeholders::_1));

    joint_name_list_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    std::string urdf_path, srdf_path;
    urdf_path = ament_index_cpp::get_package_share_directory("crs_support") + "/urdf/crs.urdf";
    srdf_path = ament_index_cpp::get_package_share_directory("crs_support") + "/urdf/ur10e_robot.srdf";
    std::stringstream urdf_xml_string, srdf_xml_string;
    std::ifstream urdf_in(urdf_path);
    urdf_xml_string << urdf_in.rdbuf();
    std::ifstream srdf_in(srdf_path);
    srdf_xml_string << srdf_in.rdbuf();

    tesseract_local_ = std::make_shared<tesseract::Tesseract>();
    tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    tesseract_local_->init(urdf_xml_string.str(), srdf_xml_string.str(), locator);
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
    std::string base_link_frame = "world";

    Eigen::Isometry3d tcp_eigen;
    tcp_eigen.setIdentity();

    auto traj_pc = std::make_shared<tesseract_motion_planners::TrajOptPlannerFreespaceConfig>(tesseract_local_, "manipulator", target_frame, tcp_eigen);

    // Initialize vector of target waypoints
    std::vector<tesseract_motion_planners::Waypoint::Ptr> trgt_wypts;

    // Define initial waypoint
    Eigen::VectorXd init_joint_pose(6);
    init_joint_pose << curr_joint_state_.position[0], curr_joint_state_.position[1], curr_joint_state_.position[2], curr_joint_state_.position[3], curr_joint_state_.position[4], curr_joint_state_.position[5];
    tesseract_motion_planners::JointWaypoint::Ptr joint_init_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(init_joint_pose, joint_name_list_);
    joint_init_waypoint->setIsCritical(false);

    // Define goal waypoint
    Eigen::Vector3d goal_pose(request->goal_pose.translation.x, request->goal_pose.translation.y, request->goal_pose.translation.z);
    Eigen::Quaterniond goal_ori(request->goal_pose.rotation.w, request->goal_pose.rotation.x, request->goal_pose.rotation.y, request->goal_pose.rotation.z);
    tesseract_motion_planners::CartesianWaypoint::Ptr goal_waypoint = std::make_shared<tesseract_motion_planners::CartesianWaypoint>(goal_pose, goal_ori);

    goal_waypoint->setIsCritical(true);
    Eigen::VectorXd coeffs(6);
    coeffs << 10, 10, 10, 10, 10, 10;
    goal_waypoint->setCoefficients(coeffs);

    // Add waypoints
    trgt_wypts.push_back(joint_init_waypoint);
    trgt_wypts.push_back(goal_waypoint);

    // Write target waypoints to trajopt planning config
    traj_pc->target_waypoints = trgt_wypts;

    // Various freespace configuration settings
    traj_pc->num_steps = 200;
    traj_pc->smooth_velocities = true;
    traj_pc->smooth_accelerations = true;
    traj_pc->smooth_jerks = true;
    traj_pc->collision_check = true;
    traj_pc->collision_coeff = 20;
    traj_pc->longest_valid_segment_length = 0.05;
    traj_pc->init_type = trajopt::InitInfo::STATIONARY;


    // Setup and solve trajopt motoin plannner
    tesseract_motion_planners::TrajOptMotionPlanner traj_motion_planner;
    tesseract_motion_planners::PlannerResponse plan_resp;
    traj_motion_planner.setConfiguration(traj_pc);
    traj_motion_planner.solve(plan_resp, true);

    // Store generated trajectory
    tesseract_common::TrajArray traj_array = plan_resp.joint_trajectory.trajectory;
    Eigen::MatrixXd cumulative_trajectory(traj_array.rows(), traj_array.cols());
    cumulative_trajectory << traj_array;

    // Convert trajectory to ROSmsg
    trajectory_msgs::msg::JointTrajectory cumulative_joint_trajectory;
    tesseractRosutilsToMsg(cumulative_joint_trajectory, joint_name_list_, cumulative_trajectory);
    cumulative_joint_trajectory.header.frame_id = base_link_frame;

    // Modify time
    for (int i = 0; i < traj_pc->num_steps; ++i)
    {
      cumulative_joint_trajectory.points[static_cast<size_t>(i)].time_from_start.sec = 0;
      cumulative_joint_trajectory.points[static_cast<size_t>(i)].time_from_start.nanosec = 0;//2e8;
    }

    response->output_trajectory = cumulative_joint_trajectory;
    if (plan_resp.status.value() == 0)
    {
      response->success = true;
    }
    else
    {
      response->success = false;
    }
    response->message = plan_resp.status.message();

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

  std::vector<std::string> joint_name_list_;
  sensor_msgs::msg::JointState curr_joint_state_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningServer>());
  rclcpp::shutdown();
  return 0;
}
