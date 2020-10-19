#include <rclcpp/rclcpp.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <crs_motion_planning/path_planning_utils.h>
#include <crs_motion_planning/path_processing_utils.h>

#include <crs_msgs/srv/call_freespace_motion.hpp>
#include <crs_msgs/srv/plan_process_motions.hpp>
#include <crs_msgs/srv/load_part.hpp>
#include <crs_msgs/srv/robot_positioner.hpp>

#include <tesseract_msgs/srv/modify_environment.hpp>
#include <tesseract_msgs/msg/tesseract_state.hpp>
#include <tesseract_geometry/mesh_parser.h>

#include <tf2/transform_storage.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/pose.hpp>

static const std::string RESOURCES_PACKAGE_NAME = "crs_support";
static const std::string DEFAULT_URDF_PATH = "urdf/crs.urdf";
static const std::string DEFAULT_SRDF_PATH = "urdf/ur10e_robot.srdf";

static const std::string DEFAULT_PROCESS_MOTION_PLANNING_SERVICE = "plan_process_motion";
static const std::string DEFAULT_FREESPACE_MOTION_PLANNING_SERVICE = "plan_freespace_motion";
static const std::string JOINT_STATES_TOPIC = "joint_states";
static const std::string LOADED_RASTER_PATHS_TOPIC = "original_raster_paths";
static const std::string FINAL_RASTER_PATHS_TOPIC = "fixed_raster_paths";
static const std::string UNREACHABLE_VERTICES_TOPIC = "failed_vertices";
static const std::string FOLLOW_JOINT_TRAJECTORY_ACTION = "follow_joint_trajectory";

static const std::string LOAD_PART_SERVICE = "load_part_tesseract_env";
static const std::string REMOVE_PART_SERVICE = "remove_part_tesseract_env";
static const std::string MOVE_ROBOT_BASE_SERVICE = "move_robot_base";
static const std::string MODIFY_ENVIRONMENT_SERVICE_NAME = "modify_tesseract";
static const std::string ENVIRONMENT_UPDATE_TOPIC_NAME = "monitored_tesseract";
static const std::string ENVIRONMENT_ID = "crs";

static const std::string LOADED_PART_LINK_NAME = "part_link";
static const std::string LOADED_PART_JOINT_NAME = "world_to_part_joint";
static const std::string ROBOT_BASE_JOINT_NAME = "robot_base_joint";

namespace param_names
{
static const std::string MOTION_PLANNING_CONFIG = "motion_planning_config";
static const std::string URDF_PATH = "urdf_path";
static const std::string SRDF_PATH = "srdf_path";
static const std::string PROCESS_MOTION_PLANNING_SERVICE = "process_planner_service";
static const std::string FREESPACE_MOTION_PLANNING_SERVICE = "freespace_motion_service";
static const std::string WORLD_FRAME = "world_frame";
static const std::string NUM_FREEPSACE_STEPS = "num_steps";
}  // namespace param_names

class MotionPlanningServer : public rclcpp::Node
{
public:
  MotionPlanningServer(rclcpp::NodeOptions options)
    : Node("motion_planning_server_node", options)
    , tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
    , tf_listener_(tf_buffer_)
  {
    namespace fs = boost::filesystem;

    // ROS parameters
    std::string process_planner_service = this->get_parameter(param_names::PROCESS_MOTION_PLANNING_SERVICE).as_string();
    std::string freespace_motion_service =
        this->get_parameter(param_names::FREESPACE_MOTION_PLANNING_SERVICE).as_string();

    // ROS communications
    plan_process_service_ = this->create_service<crs_msgs::srv::PlanProcessMotions>(
        process_planner_service,
        std::bind(&MotionPlanningServer::planProcess, this, std::placeholders::_1, std::placeholders::_2));
    plan_freespace_service_ = this->create_service<crs_msgs::srv::CallFreespaceMotion>(
        freespace_motion_service,
        std::bind(&MotionPlanningServer::planFreespace, this, std::placeholders::_1, std::placeholders::_2));
    original_path_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(LOADED_RASTER_PATHS_TOPIC, 1);
    corrected_path_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(FINAL_RASTER_PATHS_TOPIC, 1);
    failed_vertex_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(UNREACHABLE_VERTICES_TOPIC, 1);
    joint_state_listener_ = this->create_subscription<sensor_msgs::msg::JointState>(
        JOINT_STATES_TOPIC, 1, std::bind(&MotionPlanningServer::jointCallback, this, std::placeholders::_1));

    traj_exec_node_ = std::make_shared<rclcpp::Node>("trajectory_exec");
    trajectory_exec_client_cbgroup_ =
        traj_exec_node_->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    trajectory_exec_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        traj_exec_node_->get_node_base_interface(),
        traj_exec_node_->get_node_graph_interface(),
        traj_exec_node_->get_node_logging_interface(),
        traj_exec_node_->get_node_waitables_interface(),
        FOLLOW_JOINT_TRAJECTORY_ACTION,
        trajectory_exec_client_cbgroup_);

    load_part_service_ = this->create_service<crs_msgs::srv::LoadPart>(
        LOAD_PART_SERVICE,
        std::bind(&MotionPlanningServer::loadPartTesseract, this, std::placeholders::_1, std::placeholders::_2));
    remove_part_service_ = this->create_service<std_srvs::srv::Trigger>(
        REMOVE_PART_SERVICE,
        std::bind(&MotionPlanningServer::removePartTesseract, this, std::placeholders::_1, std::placeholders::_2));
    move_ur_base_service_ = this->create_service<crs_msgs::srv::RobotPositioner>(
        MOVE_ROBOT_BASE_SERVICE,
        std::bind(&MotionPlanningServer::moveURBase, this, std::placeholders::_1, std::placeholders::_2));
    modify_env_client_ = this->create_client<tesseract_msgs::srv::ModifyEnvironment>(MODIFY_ENVIRONMENT_SERVICE_NAME);
    env_state_sub_ = this->create_subscription<tesseract_msgs::msg::TesseractState>(
        ENVIRONMENT_UPDATE_TOPIC_NAME, 10, std::bind(&MotionPlanningServer::envCallback, this, std::placeholders::_1));

    // openning robot files
    std::string urdf_path, srdf_path;
    urdf_path = this->get_parameter(param_names::URDF_PATH).as_string();
    srdf_path = this->get_parameter(param_names::SRDF_PATH).as_string();
    std::vector<std::string> robot_file_paths = { urdf_path, srdf_path };
    std::vector<std::string> robot_file_contents;
    for (const auto& f : robot_file_paths)
    {
      std::ifstream ifs(f);
      if (!ifs.is_open())
      {
        throw std::runtime_error(boost::str(boost::format("File '%s' could not be opened") % f));
      }
      std::stringstream ss;
      ss << ifs.rdbuf();
      robot_file_contents.push_back(ss.str());
    }

    const std::string urdf_content = robot_file_contents[0];
    const std::string srdf_content = robot_file_contents[1];

    tesseract_local_ = std::make_shared<tesseract::Tesseract>();
    tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    tesseract_local_->init(urdf_content, srdf_content, locator);

    // motion planning config file
    std::string motion_planing_config = this->get_parameter(param_names::MOTION_PLANNING_CONFIG).as_string();
  }

private:
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr joint_msg) { curr_joint_state_ = *joint_msg; }
  void envCallback(const tesseract_msgs::msg::TesseractState::SharedPtr msg)
  {
    tesseract_revision_ = msg->revision;
    const tesseract_environment::Environment::Ptr env = tesseract_local_->getEnvironment();
    if (!tesseract_rosutils::processMsg(env, *msg))
      RCLCPP_ERROR(this->get_logger(), "Failed to update local Tesseract state");
  }
  void planProcess(std::shared_ptr<crs_msgs::srv::PlanProcessMotions::Request> request,
                   std::shared_ptr<crs_msgs::srv::PlanProcessMotions::Response> response)
  {
    namespace fs = boost::filesystem;
    crs_motion_planning::pathPlanningConfig::Ptr motion_planner_config =
        std::make_unique<crs_motion_planning::pathPlanningConfig>();
    std::string motion_planing_config = this->get_parameter(param_names::MOTION_PLANNING_CONFIG).as_string();
    crs_motion_planning::loadPathPlanningConfig(motion_planing_config, *motion_planner_config);
    motion_planner_config->tesseract_local = tesseract_local_;

    // Setup planner config with requested process planner service request data
    motion_planner_config->tcp_frame = request->tool_link;
    motion_planner_config->approach_distance = request->approach_dist;
    motion_planner_config->retreat_distance = request->retreat_dist;
    motion_planner_config->tool_speed = request->tool_speed;
    if (request->start_position.position.size() > 0)
    {
      motion_planner_config->use_start = true;
      motion_planner_config->start_pose = std::make_shared<tesseract_motion_planners::JointWaypoint>(
          request->start_position.position, request->start_position.name);
    }
    if (request->end_position.position.size() > 0)
    {
      motion_planner_config->use_end = true;
      motion_planner_config->end_pose = std::make_shared<tesseract_motion_planners::JointWaypoint>(
          request->end_position.position, request->end_position.name);
    }
    tesseract_rosutils::fromMsg(motion_planner_config->tool_offset, request->tool_offset);

    std::vector<crs_msgs::msg::ProcessMotionPlan> returned_plans;
    bool success = false;
    std::vector<trajectory_msgs::msg::JointTrajectory> trajopt_trajectories;

    // Clear old visualizations
    visualization_msgs::msg::Marker marker_eraser;
    visualization_msgs::msg::MarkerArray marker_eraser_array;
    marker_eraser.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_eraser_array.markers.push_back(marker_eraser);
    original_path_publisher_->publish(marker_eraser_array);
    corrected_path_publisher_->publish(marker_eraser_array);
    failed_vertex_publisher_->publish(marker_eraser);

    std::vector<bool> successes;

    for (size_t i = 0; i < request->process_paths.size(); ++i)
    {
      std::cout << "Planning " << i + 1 << " process of " << request->process_paths.size() << std::endl;
      if (i != 0 && success)
      {
        size_t last_success_i = 0;
        for (size_t j = 0; j < successes.size(); ++j)
        {
          if (successes[j])
            last_success_i = j;
        }
        std::vector<double> last_pose = returned_plans[last_success_i].process_motions.back().points.back().positions;
        std::vector<std::string> last_joint_names = returned_plans[last_success_i].process_motions.back().joint_names;
        motion_planner_config->use_start = true;
        motion_planner_config->start_pose =
            std::make_shared<tesseract_motion_planners::JointWaypoint>(last_pose, last_joint_names);
        returned_plans[last_success_i].end.points.clear();
        returned_plans[last_success_i].end.joint_names.clear();
      }

      // Load in current rasters
      motion_planner_config->rasters.clear();
      motion_planner_config->rasters = request->process_paths[i].rasters;

      // Create marker array for original raster visualization
      visualization_msgs::msg::MarkerArray mark_array_msg;
      crs_motion_planning::rasterStripsToMarkerArray(motion_planner_config->rasters,
                                                     motion_planner_config->world_frame,
                                                     mark_array_msg,
                                                     { 1.0, 0.0, 0.0, 1.0 },
                                                     -0.01);
      original_path_publisher_->publish(mark_array_msg);

      // Create crsMotionPlanner class
      crs_motion_planning::crsMotionPlanner crs_motion_planner(*motion_planner_config, this->get_logger());

      // Run process planner
      auto path_plan_results = std::make_unique<crs_motion_planning::pathPlanningResults>();

      bool curr_success =
          crs_motion_planner.generateProcessPlan(path_plan_results);  // Check if current plan was successful
      successes.push_back(curr_success);                              // Update list of successes
      success = curr_success || success;                              // Update whether any has been successful

      // Create marker array for processed raster visualization
      visualization_msgs::msg::MarkerArray temp_mark_array_msg, pub_mark_array_msg;
      crs_motion_planning::rasterStripsToMarkerArray(path_plan_results->solved_rasters,
                                                     motion_planner_config->world_frame,
                                                     temp_mark_array_msg,
                                                     { 1.0, 0.0, 1.0, 0.0 },
                                                     -0.025);
      crs_motion_planning::rasterStripsToMarkerArray(path_plan_results->failed_rasters,
                                                     motion_planner_config->world_frame,
                                                     temp_mark_array_msg,
                                                     { 1.0, 1.0, 0.0, 0.0 },
                                                     -0.025);
      crs_motion_planning::rasterStripsToMarkerArray(path_plan_results->skipped_rasters,
                                                     motion_planner_config->world_frame,
                                                     temp_mark_array_msg,
                                                     { 1.0, 1.0, 1.0, 0.0 },
                                                     -0.025);
      pub_mark_array_msg.markers.insert(
          pub_mark_array_msg.markers.end(), temp_mark_array_msg.markers.begin(), temp_mark_array_msg.markers.end());
      corrected_path_publisher_->publish(pub_mark_array_msg);

      // Create marker array for unreachable vertices
      visualization_msgs::msg::Marker failed_vertex_markers;
      crs_motion_planning::failedEdgesToMarkerArray(path_plan_results->unreachable_waypoints,
                                                    motion_planner_config->world_frame,
                                                    failed_vertex_markers,
                                                    { 1.0, 0.0, 1.0, 1.0 },
                                                    0.01);
      failed_vertex_publisher_->publish(failed_vertex_markers);

      // Store trajectories for service response
      trajopt_trajectories = path_plan_results->final_trajectories;
      crs_msgs::msg::ProcessMotionPlan resulting_process;
      if (path_plan_results->final_start_trajectory.points.size() > 0)
      {
        resulting_process.start = path_plan_results->final_start_trajectory;
      }
      if (path_plan_results->final_end_trajectory.points.size() > 0)
      {
        resulting_process.end = path_plan_results->final_end_trajectory;
      }
      resulting_process.free_motions = path_plan_results->final_freespace_trajectories;
      resulting_process.process_motions = path_plan_results->final_raster_trajectories;

      crs_motion_planning::cartesianTrajectoryConfig cart_traj_config;
      cart_traj_config.tcp_frame = motion_planner_config->tcp_frame;
      cart_traj_config.base_frame = motion_planner_config->robot_base_frame;
      cart_traj_config.manipulator = motion_planner_config->manipulator;
      cart_traj_config.tool_frame = motion_planner_config->tool0_frame;
      cart_traj_config.tesseract_local = motion_planner_config->tesseract_local;
      cart_traj_config.target_force = request->target_force;
      cart_traj_config.target_speed = motion_planner_config->tool_speed;
      motion_planner_config->tool_speed = request->tool_speed;
      for (size_t j = 0; j < resulting_process.process_motions.size(); ++j)
      {
        cartesian_trajectory_msgs::msg::CartesianTrajectory curr_cart_traj;
        crs_motion_planning::genCartesianTrajectory(
            resulting_process.process_motions[j], cart_traj_config, curr_cart_traj);
        resulting_process.force_controlled_process_motions.push_back(curr_cart_traj);
      }

      returned_plans.push_back(resulting_process);
      path_plan_results.reset();
    }
    // Populate response
    response->plans = returned_plans;

    if (success)
    {
      response->succeeded = true;
      response->err_msg = "TRAJECTORIES PUBLISHED";
    }
    else
    {
      response->succeeded = false;
      response->err_msg = "Failed to generate preplan";
    }
    motion_planner_config.reset();
  }

  void planFreespace(std::shared_ptr<crs_msgs::srv::CallFreespaceMotion::Request> request,
                     std::shared_ptr<crs_msgs::srv::CallFreespaceMotion::Response> response)
  {
    using namespace crs_motion_planning;
    namespace fs = boost::filesystem;

    crs_motion_planning::pathPlanningConfig::Ptr motion_planner_config =
        std::make_unique<crs_motion_planning::pathPlanningConfig>();
    std::string motion_planing_config = this->get_parameter(param_names::MOTION_PLANNING_CONFIG).as_string();
    crs_motion_planning::loadPathPlanningConfig(motion_planing_config, *motion_planner_config);
    motion_planner_config->tesseract_local = tesseract_local_;

    std::cout << "GOT REQUEST" << std::endl;
    motion_planner_config->tcp_frame = request->target_link;
    std::cout << "SET TARGET LINK" << std::endl;
    if (request->num_steps != 0)
    {
      motion_planner_config->ompl_config.n_output_states = request->num_steps;
    }
    else
    {
      motion_planner_config->ompl_config.n_output_states =
          this->get_parameter(param_names::NUM_FREEPSACE_STEPS).as_int();
    }
    std::cout << "SET N STEPS" << std::endl;

    // Create crsMotionPlanner class
    crs_motion_planning::crsMotionPlanner crs_motion_planner(*motion_planner_config, this->get_logger());
    std::cout << "INITIALIZED CLASS" << std::endl;

    // Define initial waypoint
    tesseract_motion_planners::JointWaypoint::Ptr joint_start_waypoint;
    if (request->start_position.position.empty())
    {
      // use current position when it was not specified in the request
      joint_start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(curr_joint_state_.position,
                                                                                        curr_joint_state_.name);
    }
    else
    {
      if (request->start_position.position.size() < curr_joint_state_.name.size())
      {
        response->message = boost::str(boost::format("start position (%lu) has fewer joints than the required number "
                                                     "of %lu") %
                                       request->start_position.position.size() % curr_joint_state_.name.size());
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), response->message.c_str());
        return;
      }
      joint_start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(
          request->start_position.position, request->start_position.name);
    }
    std::cout << "DEFINED START" << std::endl;

    // Define goal waypoint
    bool success;
    if (request->goal_position.position.empty())
    {
      Eigen::Vector3d goal_pose(
          request->goal_pose.translation.x, request->goal_pose.translation.y, request->goal_pose.translation.z);
      Eigen::Quaterniond goal_ori(request->goal_pose.rotation.w,
                                  request->goal_pose.rotation.x,
                                  request->goal_pose.rotation.y,
                                  request->goal_pose.rotation.z);
      tesseract_motion_planners::CartesianWaypoint::Ptr goal_waypoint =
          std::make_shared<tesseract_motion_planners::CartesianWaypoint>(goal_pose, goal_ori);

      success =
          crs_motion_planner.generateFreespacePlan(joint_start_waypoint, goal_waypoint, response->output_trajectory);
    }
    else
    {
      tesseract_motion_planners::JointWaypoint::Ptr goal_waypoint =
          std::make_shared<tesseract_motion_planners::JointWaypoint>(request->goal_position.position,
                                                                     request->goal_position.name);
      std::cout << "goal_pose_name " << request->goal_position.name.size() << std::endl;
      RCLCPP_INFO(this->get_logger(), "Planning FreeSpace motion to joint goal");

      success =
          crs_motion_planner.generateFreespacePlan(joint_start_waypoint, goal_waypoint, response->output_trajectory);
    }

    motion_planner_config.reset();

    if (success && response->output_trajectory.points.size() > 0)
    {
      response->success = true;
    }
    else
    {
      response->success = false;
      RCLCPP_ERROR(this->get_logger(), response->message.c_str());
      return;
    }

    if (request->execute && success)
    {
      // Publish trajectory if desired
      std::cout << "EXECUTING TRAJECTORY" << std::endl;
      // traj_publisher_->publish(response->output_trajectory);
      if (!execTrajectory(trajectory_exec_client_, this->get_logger(), response->output_trajectory, traj_exec_node_))
      {
        return;
      }
    }
  }

  void loadPartTesseract(std::shared_ptr<crs_msgs::srv::LoadPart::Request> request,
                         std::shared_ptr<crs_msgs::srv::LoadPart::Response> response)
  {
    std::vector<tesseract_geometry::Mesh::Ptr> meshes =
        tesseract_geometry::createMeshFromPath<tesseract_geometry::Mesh>(request->path_to_part);

    std::shared_ptr<tesseract_common::VectorVector3d> conv_hull_vertices(new tesseract_common::VectorVector3d);
    std::shared_ptr<Eigen::VectorXi> conv_hull_faces(new Eigen::VectorXi);

    tesseract_collision::createConvexHull(*conv_hull_vertices, *conv_hull_faces, *meshes[0]->getVertices());
    tesseract_geometry::ConvexMesh conv_mesh = tesseract_geometry::ConvexMesh(conv_hull_vertices, conv_hull_faces);

    tesseract_msgs::msg::Geometry link_geom;
    //    tesseract_rosutils::toMsg(link_geom, conv_mesh);
    tesseract_rosutils::toMsg(link_geom, *meshes[0]);

    std::vector<tesseract_msgs::msg::VisualGeometry> vis_geom_vector;
    tesseract_msgs::msg::VisualGeometry link_vis_geom;
    link_vis_geom.geometry = link_geom;
    link_vis_geom.origin = tf2::toMsg(Eigen::Isometry3d::Identity());
    vis_geom_vector.push_back(link_vis_geom);

    std::vector<tesseract_msgs::msg::CollisionGeometry> coll_geom_vector;
    tesseract_msgs::msg::CollisionGeometry link_coll_geom;
    link_coll_geom.geometry = link_geom;
    link_vis_geom.origin = tf2::toMsg(Eigen::Isometry3d::Identity());
    coll_geom_vector.push_back(link_coll_geom);

    tesseract_msgs::msg::Link added_link;
    added_link.name = LOADED_PART_LINK_NAME;
    added_link.visual = vis_geom_vector;
    added_link.collision = coll_geom_vector;
    added_link.inertial.empty = true;

    tesseract_msgs::msg::Joint added_joint;
    added_joint.name = LOADED_PART_JOINT_NAME;
    added_joint.type = tesseract_msgs::msg::Joint::FIXED;
    added_joint.child_link_name = LOADED_PART_LINK_NAME;
    added_joint.parent_link_name = this->get_parameter(param_names::WORLD_FRAME).as_string();
    added_joint.parent_to_joint_origin_transform = request->part_origin;
    added_joint.limits.empty = true;

    std::vector<tesseract_msgs::msg::EnvironmentCommand> env_command_vector;
    tesseract_msgs::msg::EnvironmentCommand env_command;
    env_command.command = tesseract_msgs::msg::EnvironmentCommand::ADD;
    env_command.add_link = added_link;
    env_command.add_joint = added_joint;

    env_command_vector.push_back(env_command);

    auto mod_env_request = std::make_shared<tesseract_msgs::srv::ModifyEnvironment::Request>();
    mod_env_request->id = ENVIRONMENT_ID;
    mod_env_request->revision = tesseract_revision_;
    mod_env_request->commands = env_command_vector;

    if (!modify_env_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "ModifyEnvironment Service unavailable");
      response->success = false;
      response->error = "Unable to reach ModifyEnvironment Service";
      return;
    }
    auto result_future = modify_env_client_->async_send_request(mod_env_request);

    RCLCPP_INFO(this->get_logger(), "Part successfully added to tesseract environment");
    response->success = true;
    response->error = "No Errors";
    return;
  }

  void removePartTesseract(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::vector<tesseract_msgs::msg::EnvironmentCommand> env_command_vector;
    tesseract_msgs::msg::EnvironmentCommand env_remove_joint_command, env_remove_part_command;
    env_remove_joint_command.command = tesseract_msgs::msg::EnvironmentCommand::REMOVE_JOINT;
    env_remove_joint_command.remove_joint = LOADED_PART_JOINT_NAME;

    env_remove_part_command.command = tesseract_msgs::msg::EnvironmentCommand::REMOVE_LINK;
    env_remove_part_command.remove_link = LOADED_PART_LINK_NAME;

    env_command_vector.push_back(env_remove_part_command);

    auto mod_env_request = std::make_shared<tesseract_msgs::srv::ModifyEnvironment::Request>();
    mod_env_request->id = ENVIRONMENT_ID;
    mod_env_request->revision = tesseract_revision_;
    mod_env_request->commands = env_command_vector;

    if (!modify_env_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "ModifyEnvironment Service unavailable");
      response->success = false;
      response->message = "Unable to reach ModifyEnvironment Service";
      return;
    }

    auto result_future = modify_env_client_->async_send_request(mod_env_request);

    RCLCPP_INFO(this->get_logger(), "Part successfully removed from tesseract environment");
    response->success = true;
    response->message = "No Errors";
    return;
  }

  void moveURBase(std::shared_ptr<crs_msgs::srv::RobotPositioner::Request> request,
                  std::shared_ptr<crs_msgs::srv::RobotPositioner::Response> response)
  {
    Eigen::Isometry3d ur_base_pose = Eigen::Isometry3d::Identity();
    ur_base_pose.rotate(Eigen::Quaterniond(0.5, -0.5, -0.5, -0.5));
    if (request->robot_rail == crs_msgs::srv::RobotPositioner::Request::RAIL1)
    {
      ur_base_pose.translation().x() = -0.148896;
    }
    else
    {
      ur_base_pose.translation().x() = -1.030659;
    }
    if (request->robot_position == crs_msgs::srv::RobotPositioner::Request::POSITION1)
    {
      ur_base_pose.translation().z() = 0.7977;
    }
    else if (request->robot_position == crs_msgs::srv::RobotPositioner::Request::POSITION2)
    {
      ur_base_pose.translation().z() = 0.1532;
    }
    else
    {
      ur_base_pose.translation().z() = -0.6677;
    }
    ur_base_pose.translation().y() = -2.037406;

    std::vector<tesseract_msgs::msg::EnvironmentCommand> env_command_vector;
    tesseract_msgs::msg::EnvironmentCommand move_ur_joint_command;
    move_ur_joint_command.command = tesseract_msgs::msg::EnvironmentCommand::CHANGE_JOINT_ORIGIN;
    move_ur_joint_command.change_joint_origin_name = ROBOT_BASE_JOINT_NAME;
    move_ur_joint_command.change_joint_origin_pose = tf2::toMsg(ur_base_pose);

    env_command_vector.push_back(move_ur_joint_command);

    auto mod_env_request = std::make_shared<tesseract_msgs::srv::ModifyEnvironment::Request>();
    mod_env_request->id = ENVIRONMENT_ID;
    mod_env_request->revision = tesseract_revision_;
    mod_env_request->commands = env_command_vector;

    if (!modify_env_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "ModifyEnvironment Service unavailable");
      response->success = false;
      response->error = "Unable to reach ModifyEnvironment Service";
      return;
    }

    auto result_future = modify_env_client_->async_send_request(mod_env_request);

    RCLCPP_INFO(this->get_logger(), "Part successfully moved in tesseract environment");
    response->success = true;
    response->error = "No Errors";
    return;
  }

  rclcpp::Service<crs_msgs::srv::PlanProcessMotions>::SharedPtr plan_process_service_;
  rclcpp::Service<crs_msgs::srv::CallFreespaceMotion>::SharedPtr plan_freespace_service_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr original_path_publisher_,
      corrected_path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr failed_vertex_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_listener_;

  rclcpp::Service<crs_msgs::srv::LoadPart>::SharedPtr load_part_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remove_part_service_;
  rclcpp::Service<crs_msgs::srv::RobotPositioner>::SharedPtr move_ur_base_service_;
  rclcpp::Client<tesseract_msgs::srv::ModifyEnvironment>::SharedPtr modify_env_client_;
  rclcpp::Subscription<tesseract_msgs::msg::TesseractState>::SharedPtr env_state_sub_;

  rclcpp::Node::SharedPtr traj_exec_node_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_exec_client_;
  rclcpp::callback_group::CallbackGroup::SharedPtr trajectory_exec_client_cbgroup_;

  tesseract::Tesseract::Ptr tesseract_local_;

  sensor_msgs::msg::JointState curr_joint_state_;
  std::size_t tesseract_revision_ = 0;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = std::make_shared<MotionPlanningServer>(node_options);
  executor.add_node(node);
  executor.spin();
  return 0;
}
