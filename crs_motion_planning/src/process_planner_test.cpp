#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <crs_motion_planning/path_planning_utils.h>

#include <crs_msgs/srv/call_freespace_motion.hpp>
#include <crs_msgs/srv/plan_process_motions.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <crs_msgs/srv/load_part.hpp>

#include <tf2/transform_storage.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <boost/format.hpp>

#include <std_srvs/srv/set_bool.hpp>

static const std::string RESOURCES_PACKAGE_NAME = "crs_support";
static const std::string DEFAULT_URDF_PATH = "urdf/crs.urdf";
static const std::string DEFAULT_SRDF_PATH = "urdf/ur10e_robot.srdf";
static const std::string ENVIRONMENT_UPDATE_TOPIC_NAME = "monitored_tesseract";
static const std::string ENVIRONMENT_ID = "crs";

static const std::string POSITION_COMPLIANCE_TOPIC = "cartesian_compliance_controller/target_frame";
static const std::string VELOCITY_COMPLIANCE_TOPIC = "velocity_interface/CartesianComplianceController";
static const std::string DESIRED_WRENCH_TOPIC = "cartesian_compliance_controller/target_wrench";
static const double WAIT_SERVER_TIMEOUT = 10.0;  // seconds
static const std::string FOLLOW_JOINT_TRAJECTORY_ACTION = "follow_joint_trajectory";
static const std::string MOTION_EXECUTION_ACTION_TOPIC = "execute_surface_motion";
static const std::string CONTROLLER_CHANGER_SERVICE = "test_serv";

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

void generateFakeToolPath(const double length_x,
                          const double length_y,
                          const double spacing_x,
                          const double spacing_y,
                          std::vector<geometry_msgs::msg::PoseArray>& toolpath)
{
  double curr_x = -length_x / 2;
  int side_mult = 1;
  while (curr_x < length_x / 2)
  {
    geometry_msgs::msg::PoseArray curr_tp;
    curr_tp.header.frame_id = "part";
    double curr_y = -length_y / 2;
    while (curr_y < length_y / 2)
    {
      geometry_msgs::msg::Pose curr_pose;
      curr_pose.position.y = curr_x;
      curr_pose.position.x = curr_y * side_mult;
      curr_pose.position.z = -0.002;
      curr_pose.orientation.w = 0;
      curr_pose.orientation.x = 1;
      curr_pose.orientation.y = 0;
      curr_pose.orientation.z = 0;
      curr_tp.poses.push_back(curr_pose);
      curr_y += spacing_y;
    }
    side_mult *= -1;
    toolpath.push_back(curr_tp);
    curr_x += spacing_x;
  }
  std::cout << "Vec length: " << toolpath.size() << std::endl;
}

class ProcessPlannerTestServer  // : public rclcpp::Node
{
public:
  ProcessPlannerTestServer(std::shared_ptr<rclcpp::Node> node)
    //  ProcessPlannerTestServer()
    : node_(node)
    , pnode_(std::make_shared<rclcpp::Node>(std::string(node_->get_name()) + "_private"))
    , clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME))
    , tf_buffer_(clock_)
    , tf_listener_(tf_buffer_)
  {
    //    call_process_plan_client_ = second_server.call_process_plan_client_;
    //    second_server_ = second_server;
    //    private_node_ = std::make_shared<rclcpp::Node>("private_node");

    namespace fs = boost::filesystem;

    // ROS parameters
    node_->declare_parameter(
        param_names::URDF_PATH,
        (fs::path(ament_index_cpp::get_package_share_directory(RESOURCES_PACKAGE_NAME)) / fs::path(DEFAULT_URDF_PATH))
            .string());

    node_->declare_parameter(
        param_names::SRDF_PATH,
        (fs::path(ament_index_cpp::get_package_share_directory(RESOURCES_PACKAGE_NAME)) / fs::path(DEFAULT_SRDF_PATH))
            .string());

    node_->declare_parameter(param_names::ROOT_LINK_FRAME, "base_link");
    node_->declare_parameter(param_names::WORLD_FRAME, "world");
    node_->declare_parameter(param_names::TOOL0_FRAME, "tool0");
    node_->declare_parameter(param_names::TCP_FRAME, "sander_center_link");
    node_->declare_parameter(param_names::MANIPULATOR_GROUP, "manipulator");
    // ROS communications
    test_process_planner_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "test_process_planner",
        std::bind(&ProcessPlannerTestServer::planService, this, std::placeholders::_1, std::placeholders::_2));

    trajectory_exec_client_cbgroup_ =
        node_->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    trajectory_exec_client_ =
        rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node_->get_node_base_interface(),
                                                                                  node_->get_node_graph_interface(),
                                                                                  node_->get_node_logging_interface(),
                                                                                  node_->get_node_waitables_interface(),
                                                                                  FOLLOW_JOINT_TRAJECTORY_ACTION,
                                                                                  trajectory_exec_client_cbgroup_);

    surface_traj_exec_client_cbgroup_ =
        node_->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    //    surface_traj_exec_client_ =
    //        rclcpp_action::create_client<crs_msgs::action::CartesianComplianceTrajectory>(this->get_node_base_interface(),
    //                                                                                      this->get_node_graph_interface(),
    //                                                                                      this->get_node_logging_interface(),
    //                                                                                      this->get_node_waitables_interface(),
    //                                                                                      MOTION_EXECUTION_ACTION_TOPIC,
    //                                                                                      trajectory_exec_client_cbgroup_);
    surface_traj_exec_client_ =
        rclcpp_action::create_client<cartesian_trajectory_msgs::action::CartesianComplianceTrajectory>(
            node_->get_node_base_interface(),
            node_->get_node_graph_interface(),
            node_->get_node_logging_interface(),
            node_->get_node_waitables_interface(),
            MOTION_EXECUTION_ACTION_TOPIC,
            trajectory_exec_client_cbgroup_);

    joint_state_listener_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 1, std::bind(&ProcessPlannerTestServer::jointCallback, this, std::placeholders::_1));

    env_state_sub_ = node_->create_subscription<tesseract_msgs::msg::TesseractState>(
        ENVIRONMENT_UPDATE_TOPIC_NAME,
        10,
        std::bind(&ProcessPlannerTestServer::envCallback, this, std::placeholders::_1));

    call_process_plan_client_ = pnode_->create_client<crs_msgs::srv::PlanProcessMotions>("plan_process_motion");

    part_loader_client_ = node_->create_client<crs_msgs::srv::LoadPart>("load_part_tesseract_env");

    controller_changer_client_ = node_->create_client<std_srvs::srv::SetBool>(CONTROLLER_CHANGER_SERVICE);

    toolpath_filepath_ = ament_index_cpp::get_package_share_directory("crs_support") + "/toolpaths/scanned_part1/"
                                                                                       "job_90degrees.yaml";

    test_part_loader_client_ = node_->create_service<std_srvs::srv::Trigger>(
        "test_part_loader",
        std::bind(&ProcessPlannerTestServer::testPartLoader, this, std::placeholders::_1, std::placeholders::_2));

    part_filepath_ = ament_index_cpp::get_package_share_directory("crs_support") + "/meshes/Parts/visual/"
                                                                                   "part1_ch.stl";
    // waiting for server
    //    if (!trajectory_exec_client_->wait_for_action_server(std::chrono::duration<double>(WAIT_SERVER_TIMEOUT)))
    //    {
    //      std::string err_msg =
    //          boost::str(boost::format("Failed to find action server %s") % FOLLOW_JOINT_TRAJECTORY_ACTION);
    //      RCLCPP_ERROR(node_->get_logger(), "%s", err_msg.c_str());
    //      throw std::runtime_error(err_msg);
    //    }
    //    RCLCPP_INFO(node_->get_logger(), "%s action client found2", FOLLOW_JOINT_TRAJECTORY_ACTION.c_str());

    //    // waiting for server
    //    if (!surface_traj_exec_client_->wait_for_action_server(std::chrono::duration<double>(WAIT_SERVER_TIMEOUT)))
    //    {
    //      std::string err_msg =
    //          boost::str(boost::format("Failed to find surface action server %s") % MOTION_EXECUTION_ACTION_TOPIC);
    //      RCLCPP_ERROR(node_->get_logger(), "%s", err_msg.c_str());
    //      throw std::runtime_error(err_msg);
    //    }
    //    RCLCPP_INFO(node_->get_logger(), "%s surface action client found", MOTION_EXECUTION_ACTION_TOPIC.c_str());

    // openning files
    std::string urdf_path, srdf_path;
    urdf_path = node_->get_parameter(param_names::URDF_PATH).as_string();
    srdf_path = node_->get_parameter(param_names::SRDF_PATH).as_string();
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
  }

  void get_priv_node(rclcpp::Node::SharedPtr& priv_node) { priv_node = pnode_; }

  void get_priv_node(rclcpp::Node::SharedPtr& priv_node) { priv_node = pnode_; }

private:
  bool change_controller(const std_srvs::srv::SetBool::Request::SharedPtr req)
  {
    using namespace std_srvs::srv;
    if (req->data)
      RCLCPP_ERROR(node_->get_logger(), "REQUESTING CART");
    else
      RCLCPP_ERROR(node_->get_logger(), "REQUESTING JOINT");
    std::shared_future<SetBool::Response::SharedPtr> result_future =
        controller_changer_client_->async_send_request(req);

    std::future_status status = result_future.wait_for(std::chrono::seconds(15));
    if (status != std::future_status::ready)
    {
      RCLCPP_ERROR(node_->get_logger(), "change controller service error or timeout");
      return false;
    }

    if (!result_future.get()->success)
    {
      RCLCPP_ERROR(node_->get_logger(), "change controller service failed");
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "change controller service succeeded");
    return true;
  }

  void envCallback(const tesseract_msgs::msg::TesseractState::SharedPtr msg)
  {
    const tesseract_environment::Environment::Ptr env = tesseract_local_->getEnvironment();
    if (!tesseract_rosutils::processMsg(env, *msg))
      RCLCPP_ERROR(node_->get_logger(), "Failed to update local Tesseract state");
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr joint_msg) { curr_joint_state_ = *joint_msg; }
  void planService(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(node_->get_logger(), "Planning now");
    // Load rasters and get them in usable form
    std::string waypoint_origin_frame = "part";
    std::vector<geometry_msgs::msg::PoseArray> raster_strips;
    generateFakeToolPath(0.06, 0.5, 0.05, 0.025, raster_strips);
    //    crs_motion_planning::parsePathFromFile(toolpath_filepath_, waypoint_origin_frame, raster_strips);
    geometry_msgs::msg::PoseArray strip_of_interset;
    for (auto strip : raster_strips)
    {
      strip_of_interset.poses.insert(strip_of_interset.poses.end(), strip.poses.begin(), strip.poses.end());
    }

    // Get transform between world and part
    tf2::TimePoint time_point = tf2::TimePointZero;
    geometry_msgs::msg::TransformStamped world_to_goal_frame;
    try
    {
      world_to_goal_frame = tf_buffer_.lookupTransform("world", waypoint_origin_frame, time_point);
    }
    catch (tf2::LookupException& e)
    {
      response->success = false;
      response->message = "TF lookup failed: " + std::string(e.what());
      return;
    }

    std::vector<geometry_msgs::msg::PoseArray> raster_strips_world_frame;
    for (auto strip : raster_strips)
    {
      geometry_msgs::msg::PoseArray curr_strip, ar_strip;
      for (size_t i = 0; i < strip.poses.size(); ++i)
      {
        geometry_msgs::msg::PoseStamped surface_pose_world_frame, surface_pose_og_frame;
        surface_pose_og_frame.pose = strip.poses[i];
        surface_pose_og_frame.header = strip.header;
        tf2::doTransform(surface_pose_og_frame, surface_pose_world_frame, world_to_goal_frame);
        geometry_msgs::msg::Pose sf_pose_wf = surface_pose_world_frame.pose;
        curr_strip.poses.push_back(std::move(sf_pose_wf));
      }
      raster_strips_world_frame.push_back(curr_strip);
    }

    auto proc_req = std::make_shared<crs_msgs::srv::PlanProcessMotions::Request>();
    proc_req->tool_link = "sander_center_link";
    proc_req->tool_speed = 0.05;
    proc_req->approach_dist = 0.025;
    proc_req->retreat_dist = 0.025;
    proc_req->start_position = curr_joint_state_;
    proc_req->end_position = curr_joint_state_;
    Eigen::Isometry3d tool_offset_req = Eigen::Isometry3d::Identity();
    geometry_msgs::msg::Pose geom_tool_offset;
    tesseract_rosutils::toMsg(geom_tool_offset, tool_offset_req);
    geom_tool_offset.position.z = 0.0;
    proc_req->tool_offset = geom_tool_offset;
    std::vector<crs_msgs::msg::ToolProcessPath> path_requests;
    crs_msgs::msg::ToolProcessPath path_wf;
    path_wf.rasters = raster_strips_world_frame;
    path_requests.push_back(path_wf);
    proc_req->process_paths = path_requests;

    auto process_plan_cb = std::bind(&ProcessPlannerTestServer::processPlanCallback, this, std::placeholders::_1);
    std::shared_future<crs_msgs::srv::PlanProcessMotions::Response::SharedPtr> result_future =
        call_process_plan_client_->async_send_request(proc_req, process_plan_cb);

    std::future_status status = result_future.wait_for(std::chrono::seconds(20));
    if (status != std::future_status::ready)
    {
      RCLCPP_ERROR(node_->get_logger(), "plan service error or timeout");
    }

    if (!result_future.get()->succeeded)
    {
      RCLCPP_ERROR(node_->get_logger(), "plan service failed");
    }

    RCLCPP_INFO(node_->get_logger(), "plan service succeeded");

    response->success = true;
    response->message = "Plan Sent";
  }

  void processPlanCallback(const rclcpp::Client<crs_msgs::srv::PlanProcessMotions>::SharedFuture future)
  {
    using namespace crs_motion_planning;
    bool success = future.get()->succeeded;

    if (success)
    {
      std_srvs::srv::SetBool::Request::SharedPtr trig_req = std::make_shared<std_srvs::srv::SetBool::Request>();
      bool successful_controller_change = false;

      std::vector<crs_msgs::msg::ProcessMotionPlan> process_plans = future.get()->plans;
      crs_motion_planning::cartesianTrajectoryConfig traj_config;
      traj_config.tesseract_local = tesseract_local_;
      traj_config.manipulator = "manipulator";
      traj_config.base_frame = "base_link";
      traj_config.tcp_frame = "sander_center_link";
      Eigen::Vector3d path_pose_tolerance = Eigen::Vector3d::Ones() * 0.01;
      path_pose_tolerance(2) = 0.02;
      Eigen::Vector3d path_ori_tolerance = Eigen::Vector3d::Ones() * 0.1;
      Eigen::Vector3d goal_pose_tolerance = Eigen::Vector3d::Ones() * 0.015;
      goal_pose_tolerance(2) = 0.025;
      Eigen::Vector3d goal_ori_tolerance = Eigen::Vector3d::Ones() * 0.05;
      Eigen::Vector3d force_tolerance = Eigen::Vector3d::Ones() * 20;
      force_tolerance(0) = 50;
      force_tolerance(1) = 50;
      traj_config.path_pose_tolerance = tf2::toMsg(path_pose_tolerance, traj_config.path_pose_tolerance);
      traj_config.path_ori_tolerance = tf2::toMsg(path_ori_tolerance, traj_config.path_ori_tolerance);
      traj_config.goal_pose_tolerance = tf2::toMsg(goal_pose_tolerance, traj_config.goal_pose_tolerance);
      traj_config.goal_ori_tolerance = tf2::toMsg(goal_ori_tolerance, traj_config.goal_ori_tolerance);
      traj_config.force_tolerance = tf2::toMsg(force_tolerance, traj_config.force_tolerance);
      traj_config.target_force = 50;
      traj_config.target_speed = 0.15;
      for (size_t j = 0; j < process_plans.size(); ++j)
      {
        if (!rclcpp::ok())
        {
          return;
        }
        RCLCPP_INFO(node_->get_logger(), "EXECUTING PROCESS\t%i OF %i", j + 1, process_plans.size());
        trajectory_msgs::msg::JointTrajectory start_traj = process_plans[j].start;
        trajectory_msgs::msg::JointTrajectory end_traj = process_plans[j].end;
        std::vector<trajectory_msgs::msg::JointTrajectory> process_motions = process_plans[j].process_motions;
        std::vector<trajectory_msgs::msg::JointTrajectory> freespace_motions = process_plans[j].free_motions;
        std::vector<cartesian_trajectory_msgs::msg::CartesianTrajectory> cart_process_motions =
            process_plans[j].force_controlled_process_motions;
        if (start_traj.points.size() > 0)
        {
          trig_req->data = false;
          successful_controller_change = change_controller(trig_req);
          RCLCPP_ERROR(node_->get_logger(), "EXECUTING FIRST FREESPACE");
          if (!successful_controller_change ||
              !execTrajectory(trajectory_exec_client_, node_->get_logger(), start_traj))
          {
            return;
          }
        }

        for (size_t i = 0; i < freespace_motions.size(); ++i)
        {
          RCLCPP_INFO(node_->get_logger(), "EXECUTING SURFACE TRAJECTORY\t%i OF %i", i + 1, process_motions.size());
          //          if (!execTrajectory(trajectory_exec_client_, this->get_logger(), process_motions[i]))
          //          {
          //            return;
          //          }
          trig_req->data = true;
          //          trig_req->data = false;
          successful_controller_change = change_controller(trig_req);
          //          if (!successful_controller_change || !execSurfaceTrajectory(surface_traj_exec_client_,
          //          node_->get_logger(), process_motions[i], traj_config))
          if (!successful_controller_change ||
              !execSurfaceTrajectory(
                  surface_traj_exec_client_, node_->get_logger(), cart_process_motions[i], traj_config))
          //          if (!successful_controller_change || !execTrajectory(trajectory_exec_client_, node_->get_logger(),
          //          process_motions[i]))
          {
            return;
          }

          trig_req->data = false;
          successful_controller_change = change_controller(trig_req);
          RCLCPP_INFO(node_->get_logger(), "EXECUTING FREESPACE TRAJECTORY\t%i OF %i", i + 1, freespace_motions.size());
          if (!successful_controller_change ||
              !execTrajectory(trajectory_exec_client_, node_->get_logger(), freespace_motions[i]))
          {
            return;
          }
        }

        RCLCPP_INFO(node_->get_logger(),
                    "EXECUTING SURFACE TRAJECTORY\t%i OF %i",
                    process_motions.size(),
                    process_motions.size());
        trig_req->data = true;
        //        trig_req->data = false;
        successful_controller_change = change_controller(trig_req);
        //        if (!successful_controller_change || !execSurfaceTrajectory(surface_traj_exec_client_,
        //        node_->get_logger(), process_motions.back(), traj_config))
        if (!successful_controller_change ||
            !execSurfaceTrajectory(
                surface_traj_exec_client_, node_->get_logger(), cart_process_motions.back(), traj_config))
        //        if (!successful_controller_change || !execTrajectory(trajectory_exec_client_, node_->get_logger(),
        //        process_motions.back()))
        {
          return;
        }
        if (end_traj.points.size() > 0)
        {
          trig_req->data = false;
          successful_controller_change = change_controller(trig_req);
          RCLCPP_INFO(node_->get_logger(), "EXECUTING FINAL FREESPACE");
          if (!successful_controller_change || !execTrajectory(trajectory_exec_client_, node_->get_logger(), end_traj))
          {
            return;
          }
        }
      }

      std::cout << "ALL DONE" << std::endl;
    }
    else
    {
      std::cout << future.get()->err_msg << std::endl;
    }
  }

  void testPartLoader(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    geometry_msgs::msg::Pose part_pose;
    part_pose.position.x = -0.53;
    part_pose.position.y = -0.14;
    part_pose.position.z = 1.1;

    part_pose.orientation.x = 0.5;
    part_pose.orientation.y = -0.5;
    part_pose.orientation.z = 0.5;
    part_pose.orientation.w = 0.5;

    auto load_part_request = std::make_shared<crs_msgs::srv::LoadPart::Request>();
    load_part_request->path_to_part = part_filepath_;
    load_part_request->part_origin = part_pose;

    part_loader_client_->async_send_request(load_part_request);
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_process_planner_service_;
  rclcpp::Client<crs_msgs::srv::PlanProcessMotions>::SharedPtr call_process_plan_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_exec_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_listener_;
  rclcpp::callback_group::CallbackGroup::SharedPtr trajectory_exec_client_cbgroup_;

  rclcpp_action::Client<cartesian_trajectory_msgs::action::CartesianComplianceTrajectory>::SharedPtr
      surface_traj_exec_client_;
  rclcpp::callback_group::CallbackGroup::SharedPtr surface_traj_exec_client_cbgroup_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_part_loader_client_;
  rclcpp::Client<crs_msgs::srv::LoadPart>::SharedPtr part_loader_client_;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr controller_changer_client_;

  std::shared_ptr<rclcpp::Clock> clock_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  sensor_msgs::msg::JointState curr_joint_state_;

  std::string toolpath_filepath_;
  std::string part_filepath_;

  rclcpp::Subscription<tesseract_msgs::msg::TesseractState>::SharedPtr env_state_sub_;

  tesseract::Tesseract::Ptr tesseract_local_;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> pnode_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("process_planner_test_node");
  executor.add_node(node);
  ProcessPlannerTestServer exec(node);
  std::shared_ptr<rclcpp::Node> pnode;
  exec.get_priv_node(pnode);
  executor.add_node(pnode);
  //  ProcessPlannerTestServer::SharedPtr node_test = std::make_shared<ProcessPlannerTestServer>();
  //  rclcpp::Node::SharedPtr node_private;
  //  get_priv_node(node_private);
  //  rclcpp::Node::SharedPtr node = std::make_shared<ProcessPlannerTestServer>();
  //  executor.add_node(node_test);
  executor.spin();
  //  while (rclcpp::ok())
  //  {
  //    executor.spin_some(rclcpp::Duration::from_seconds(15.0).to_chrono<std::chrono::nanoseconds>());
  //    executor.spin_some();
  //  }
  rclcpp::shutdown();
  return 0;
}
