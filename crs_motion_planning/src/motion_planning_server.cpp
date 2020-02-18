#include <rclcpp/rclcpp.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <crs_motion_planning/path_planning_utils.h>

#include <crs_msgs/srv/call_freespace_motion.hpp>
#include <crs_msgs/srv/plan_process_motions.hpp>

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
static const std::string DEFAULT_FREESPACE_TRAJECTORY_TOPIC = "planned_freespace_trajectory";
static const std::string JOINT_STATES_TOPIC = "crs/joint_states";
static const std::string LOADED_RASTER_PATHS_TOPIC = "original_raster_paths";
static const std::string FINAL_RASTER_PATHS_TOPIC = "fixed_raster_paths";
static const std::string UNREACHABLE_VERTICES_TOPIC = "failed_vertices";

namespace param_names
{
static const std::string URDF_PATH = "urdf_path";
static const std::string SRDF_PATH = "srdf_path";
static const std::string PROCESS_MOTION_PLANNING_SERVICE = "process_planner_service";
static const std::string FREESPACE_MOTION_PLANNING_SERVICE = "freespace_motion_service";
static const std::string FREESPACE_TRAJECTORY_TOPIC = "trajectory_topic";
static const std::string ROOT_LINK_FRAME = "base_link_frame";
static const std::string WORLD_FRAME = "world_frame";
static const std::string TOOL0_FRAME = "tool0_frame";
static const std::string MANIPULATOR_GROUP = "manipulator_group";
static const std::string NUM_FREEPSACE_STEPS = "num_steps";
static const std::string MAX_JOINT_VELOCITY = "max_joint_velocity";
static const std::string MIN_RASTER_LENGTH = "min_raster_length";
static const std::string GAZEBO_SIM_TIMING = "use_gazebo_simulation_time";
static const std::string TRAJOPT_VERBOSE = "set_trajopt_verbose";
}  // namespace param_names

class MotionPlanningServer : public rclcpp::Node
{
public:
  MotionPlanningServer() : Node("motion_planning_server_node")
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

    this->declare_parameter(param_names::FREESPACE_MOTION_PLANNING_SERVICE, DEFAULT_FREESPACE_MOTION_PLANNING_SERVICE);
    this->declare_parameter(param_names::FREESPACE_TRAJECTORY_TOPIC, DEFAULT_FREESPACE_TRAJECTORY_TOPIC);
    this->declare_parameter(param_names::PROCESS_MOTION_PLANNING_SERVICE, DEFAULT_PROCESS_MOTION_PLANNING_SERVICE);
    this->declare_parameter(param_names::ROOT_LINK_FRAME, "base_link");
    this->declare_parameter(param_names::WORLD_FRAME, "world");
    this->declare_parameter(param_names::TOOL0_FRAME, "tool0");
    this->declare_parameter(param_names::MANIPULATOR_GROUP, "manipulator");
    this->declare_parameter(param_names::NUM_FREEPSACE_STEPS, 20);
    this->declare_parameter(param_names::MAX_JOINT_VELOCITY, 0.5);
    this->declare_parameter(param_names::MIN_RASTER_LENGTH, 3);
    this->declare_parameter(param_names::GAZEBO_SIM_TIMING, false);
    this->declare_parameter(param_names::TRAJOPT_VERBOSE, false);

    std::string process_planner_service = this->get_parameter(param_names::PROCESS_MOTION_PLANNING_SERVICE).as_string();
    std::string freespace_motion_service =
        this->get_parameter(param_names::FREESPACE_MOTION_PLANNING_SERVICE).as_string();
    std::string freespace_trajectory_topic = this->get_parameter(param_names::FREESPACE_TRAJECTORY_TOPIC).as_string();

    // ROS communications
    plan_process_service_ = this->create_service<crs_msgs::srv::PlanProcessMotions>(
        process_planner_service,
        std::bind(&MotionPlanningServer::planProcess, this, std::placeholders::_1, std::placeholders::_2));
    plan_freespace_service_ = this->create_service<crs_msgs::srv::CallFreespaceMotion>(
        freespace_motion_service,
        std::bind(&MotionPlanningServer::planFreespace, this, std::placeholders::_1, std::placeholders::_2));
    traj_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(freespace_trajectory_topic, 1);
    original_path_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(LOADED_RASTER_PATHS_TOPIC, 1);
    corrected_path_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(FINAL_RASTER_PATHS_TOPIC, 1);
    failed_vertex_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(UNREACHABLE_VERTICES_TOPIC, 1);
    joint_state_listener_ = this->create_subscription<sensor_msgs::msg::JointState>(
        JOINT_STATES_TOPIC, 1, std::bind(&MotionPlanningServer::jointCallback, this, std::placeholders::_1));

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

    tesseract::Tesseract::Ptr tesseract_local = std::make_shared<tesseract::Tesseract>();
    tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    tesseract_local->init(urdf_content, srdf_content, locator);

    // Set up planning config variable
    crs_motion_planning::descartesConfig descartes_config;
    descartes_config.axial_step = 0.1;
    descartes_config.collision_safety_margin = 0.0075;

    crs_motion_planning::trajoptSurfaceConfig trajopt_surface_config;
    trajopt_surface_config.smooth_velocities = false;
    trajopt_surface_config.smooth_accelerations = false;
    trajopt_surface_config.smooth_jerks = false;
    tesseract_motion_planners::CollisionCostConfig coll_cost_config_srfc;
    coll_cost_config_srfc.enabled = false;
    trajopt_surface_config.coll_cst_cfg = coll_cost_config_srfc;
    tesseract_motion_planners::CollisionConstraintConfig coll_cnt_config_srfc;
    coll_cnt_config_srfc.enabled = true;
    coll_cnt_config_srfc.safety_margin = 0.001;
    trajopt_surface_config.coll_cnt_cfg = coll_cnt_config_srfc;
    Eigen::VectorXd surface_coeffs(6);
    surface_coeffs << 10, 10, 10, 10, 10, 0;
    trajopt_surface_config.surface_coeffs = surface_coeffs;
    trajopt_surface_config.waypoints_critical = true;
    trajopt_surface_config.longest_valid_segment_fraction = 0.001;

    crs_motion_planning::omplConfig ompl_config;
    ompl_config.collision_safety_margin = 0.0175;
    ompl_config.planning_time = 5;
    ompl_config.n_output_states = this->get_parameter(param_names::NUM_FREEPSACE_STEPS).as_int();
    ompl_config.simplify = true;
    ompl_config.longest_valid_segment_fraction = 0.005;

    crs_motion_planning::trajoptFreespaceConfig trajopt_freespace_config;
    tesseract_motion_planners::CollisionCostConfig coll_cost_config_fs;
    coll_cost_config_fs.enabled = true;
    coll_cost_config_fs.buffer_margin = 0.05;
    coll_cost_config_fs.coeff = 15;
    tesseract_motion_planners::CollisionConstraintConfig coll_cnt_config_fs;
    coll_cnt_config_fs.enabled = true;
    coll_cnt_config_fs.safety_margin = 0.005;
//    coll_cnt_config_fs.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
    trajopt_freespace_config.coll_cst_cfg = coll_cost_config_fs;
    trajopt_freespace_config.coll_cnt_cfg = coll_cnt_config_fs;
    trajopt_freespace_config.longest_valid_segment_fraction = 0.001;
    trajopt_freespace_config.contact_test_type = tesseract_collision::ContactTestType::ALL;
//    trajopt_freespace_config.contact_test_type = tesseract_collision::ContactTestType::CLOSEST;
    trajopt_freespace_config.smooth_velocities = true;
    trajopt_freespace_config.smooth_accelerations = true;
    trajopt_freespace_config.smooth_jerks = true;

    motion_planner_config_ = std::make_shared<crs_motion_planning::pathPlanningConfig>();
    motion_planner_config_->tesseract_local = tesseract_local;
    motion_planner_config_->descartes_config = descartes_config;
    motion_planner_config_->trajopt_surface_config = trajopt_surface_config;
    motion_planner_config_->ompl_config = ompl_config;
    motion_planner_config_->trajopt_freespace_config = trajopt_freespace_config;
    motion_planner_config_->manipulator = this->get_parameter(param_names::MANIPULATOR_GROUP).as_string();
    motion_planner_config_->robot_base_frame = this->get_parameter(param_names::ROOT_LINK_FRAME).as_string();
    motion_planner_config_->world_frame = this->get_parameter(param_names::WORLD_FRAME).as_string();
    motion_planner_config_->tool0_frame = this->get_parameter(param_names::TOOL0_FRAME).as_string();
    motion_planner_config_->max_joint_vel =
        this->get_parameter(param_names::MAX_JOINT_VELOCITY).as_double();  // 5.0;//1.5;
    motion_planner_config_->minimum_raster_length = this->get_parameter(param_names::MIN_RASTER_LENGTH).as_int();
    motion_planner_config_->add_approach_and_retreat = true;
    motion_planner_config_->required_tool_vel = true;
    motion_planner_config_->use_gazebo_sim_timing = this->get_parameter(param_names::GAZEBO_SIM_TIMING).as_bool();
    motion_planner_config_->trajopt_verbose_output = this->get_parameter(param_names::TRAJOPT_VERBOSE).as_bool();
    motion_planner_config_->simplify_start_end_freespace = true;
    motion_planner_config_->use_trajopt_freespace = true;
    motion_planner_config_->combine_strips = true;
    motion_planner_config_->global_descartes = true;
  }

private:
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr joint_msg) { curr_joint_state_ = *joint_msg; }
  void planProcess(std::shared_ptr<crs_msgs::srv::PlanProcessMotions::Request> request,
                   std::shared_ptr<crs_msgs::srv::PlanProcessMotions::Response> response)
  {
    // Setup planner config with requested process planner service request data
    motion_planner_config_->tcp_frame = request->tool_link;
    motion_planner_config_->approach_distance = request->approach_dist;
    motion_planner_config_->retreat_distance = request->retreat_dist;
    motion_planner_config_->tool_speed = request->tool_speed;
    if (request->start_position.position.size() > 0)
    {
      motion_planner_config_->use_start = true;
      motion_planner_config_->start_pose = std::make_shared<tesseract_motion_planners::JointWaypoint>(
          request->start_position.position, request->start_position.name);
    }
    if (request->end_position.position.size() > 0)
    {
      motion_planner_config_->use_end = true;
      motion_planner_config_->end_pose = std::make_shared<tesseract_motion_planners::JointWaypoint>(
          request->end_position.position, request->end_position.name);
    }
    tesseract_rosutils::fromMsg(motion_planner_config_->tool_offset, request->tool_offset);

    std::vector<crs_msgs::msg::ProcessMotionPlan> returned_plans;
    bool success;
    std::vector<trajectory_msgs::msg::JointTrajectory> trajopt_trajectories;
    auto path_plan_results = std::make_shared<crs_motion_planning::pathPlanningResults>();

    for (size_t i = 0; i < request->process_paths.size(); ++i)
    {
      // Load in current rasters
      motion_planner_config_->rasters = request->process_paths[i].rasters;

      // Create marker array for original raster visualization
      visualization_msgs::msg::MarkerArray mark_array_msg;
      crs_motion_planning::rasterStripsToMarkerArray(
          motion_planner_config_->rasters, "world", mark_array_msg, { 1.0, 0.0, 0.0, 1.0 }, -0.01);
      original_path_publisher_->publish(mark_array_msg);

      // Create crsMotionPlanner class
      crs_motion_planning::crsMotionPlanner crs_motion_planner(motion_planner_config_);

      // Run process planner
      success = crs_motion_planner.generateProcessPlan(path_plan_results);

      // Create marker array for processed raster visualization
      visualization_msgs::msg::MarkerArray temp_mark_array_msg, pub_mark_array_msg;
      crs_motion_planning::rasterStripsToMarkerArray(path_plan_results->solved_rasters,
                                                     motion_planner_config_->world_frame,
                                                     temp_mark_array_msg,
                                                     { 1.0, 0.0, 1.0, 0.0 },
                                                     -0.025);
      crs_motion_planning::rasterStripsToMarkerArray(path_plan_results->failed_rasters,
                                                     motion_planner_config_->world_frame,
                                                     temp_mark_array_msg,
                                                     { 1.0, 1.0, 0.0, 0.0 },
                                                     -0.025);
      crs_motion_planning::rasterStripsToMarkerArray(path_plan_results->skipped_rasters,
                                                     motion_planner_config_->world_frame,
                                                     temp_mark_array_msg,
                                                     { 1.0, 1.0, 1.0, 0.0 },
                                                     -0.025);
      pub_mark_array_msg.markers.insert(
          pub_mark_array_msg.markers.end(), temp_mark_array_msg.markers.begin(), temp_mark_array_msg.markers.end());
      corrected_path_publisher_->publish(pub_mark_array_msg);

      // Create marker array for unreachable vertices
      visualization_msgs::msg::Marker failed_vertex_markers;
      crs_motion_planning::failedEdgesToMarkerArray(path_plan_results->unreachable_waypoints,
                                                    motion_planner_config_->world_frame,
                                                    failed_vertex_markers,
                                                    { 1.0, 0.0, 1.0, 1.0 },
                                                    0.01);
      failed_vertex_publisher_->publish(failed_vertex_markers);

      // Store trajectories for service response
      trajopt_trajectories = path_plan_results->final_trajectories;
      crs_msgs::msg::ProcessMotionPlan resulting_process;
      if (path_plan_results->final_start_end_trajectories.size() > 0)
      {
        resulting_process.start = path_plan_results->final_start_end_trajectories[0];
      }
      if (path_plan_results->final_start_end_trajectories.size() > 1)
      {
        resulting_process.end = path_plan_results->final_start_end_trajectories[1];
      }
      resulting_process.free_motions = path_plan_results->final_freespace_trajectories;
      resulting_process.process_motions = path_plan_results->final_raster_trajectories;
      returned_plans.push_back(resulting_process);
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
  }

  void planFreespace(std::shared_ptr<crs_msgs::srv::CallFreespaceMotion::Request> request,
                     std::shared_ptr<crs_msgs::srv::CallFreespaceMotion::Response> response)
  {
    std::cout << "GOT REQUEST" << std::endl;
    motion_planner_config_->tcp_frame = request->target_link;
    std::cout << "SET TARGET LINK" << std::endl;
    if (request->num_steps != 0)
    {
      motion_planner_config_->ompl_config.n_output_states = request->num_steps;
    }
    else
    {
      motion_planner_config_->ompl_config.n_output_states =
          this->get_parameter(param_names::NUM_FREEPSACE_STEPS).as_int();
    }
    std::cout << "SET N STEPS" << std::endl;

    // Create crsMotionPlanner class
    crs_motion_planning::crsMotionPlanner crs_motion_planner(motion_planner_config_);
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
      RCLCPP_INFO(this->get_logger(), "Planning FreeSpace motion to joint goal");

      success =
          crs_motion_planner.generateFreespacePlan(joint_start_waypoint, goal_waypoint, response->output_trajectory);
    }

    // Modify time
    for (size_t i = 0; i < response->output_trajectory.points.size(); ++i)
    {
      response->output_trajectory.points[i].time_from_start.sec = 0;
      response->output_trajectory.points[i].time_from_start.nanosec = 0;  // 2e8;
    }

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
      traj_publisher_->publish(response->output_trajectory);
    }
  }

  rclcpp::Service<crs_msgs::srv::PlanProcessMotions>::SharedPtr plan_process_service_;
  rclcpp::Service<crs_msgs::srv::CallFreespaceMotion>::SharedPtr plan_freespace_service_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr original_path_publisher_,
      corrected_path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr failed_vertex_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_listener_;

  crs_motion_planning::pathPlanningConfig::Ptr motion_planner_config_;

  sensor_msgs::msg::JointState curr_joint_state_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionPlanningServer>());
  rclcpp::shutdown();
  return 0;
}
