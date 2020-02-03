#include <rclcpp/rclcpp.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <crs_motion_planning/path_processing_utils.h>
#include <crs_motion_planning/path_planning_utils.h>

#include <tf2/transform_storage.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_default_config.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_freespace_config.h>
#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>

#include <trajopt/problem_description.hpp>

#include <tesseract_rosutils/utils.h>
#include <tesseract_rosutils/conversions.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/pose.hpp>

//#include <descartes_light/descartes_light.h>
//#include <descartes_samplers/evaluators/distance_edge_evaluator.h>
//#include <descartes_samplers/evaluators/euclidean_distance_edge_evaluator.h>
//#include <descartes_samplers/samplers/axial_symmetric_sampler.h>
//#include <descartes_samplers/samplers/fixed_joint_pose_sampler.h>
//#include <ur_ikfast_kinematics/descartes_ikfast_ur10e.h>

static const std::vector<double> COEFFICIENTS {10, 10, 10, 10, 10, 10};



class SurfaceServer: public rclcpp::Node
{
public:
  SurfaceServer()
    : Node("surface_server_node"),
      clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)),
      tf_buffer_(clock_),
      tf_listener_(tf_buffer_)
  {
    // ROS parameters
    this->declare_parameter("urdf_path", ament_index_cpp::get_package_share_directory("crs_support") + "/urdf/crs_v3.urdf");
    this->declare_parameter("srdf_path", ament_index_cpp::get_package_share_directory("crs_support") + "/urdf/ur10e_robot.srdf");
    this->declare_parameter("base_link_frame", "world");
    this->declare_parameter("manipulator_group", "manipulator");

    // ROS communications
    load_paths_service_ = this->create_service<std_srvs::srv::Trigger>("load_paths", std::bind(&SurfaceServer::planService, this, std::placeholders::_1, std::placeholders::_2));
    traj_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("set_trajectory_test",10);
    original_path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("original_raster_paths",10);
    corrected_path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("fixed_raster_paths",10);
    failed_vertex_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("failed_vertices",10);
    joint_state_listener_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 2, std::bind(&SurfaceServer::jointCallback, this, std::placeholders::_1));

    std::string urdf_path, srdf_path;
    urdf_path = this->get_parameter("urdf_path").as_string();
    srdf_path = this->get_parameter("srdf_path").as_string();
    std::stringstream urdf_xml_string, srdf_xml_string;
    std::ifstream urdf_in(urdf_path);
    urdf_xml_string << urdf_in.rdbuf();
    std::ifstream srdf_in(srdf_path);
    srdf_xml_string << srdf_in.rdbuf();

    tesseract_local_ = std::make_shared<tesseract::Tesseract>();
    tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    tesseract_local_->init(urdf_xml_string.str(), srdf_xml_string.str(), locator);

    base_link_frame_ = this->get_parameter("base_link_frame").as_string();
    manipulator_ = this->get_parameter("manipulator_group").as_string();
    toolpath_filepath_ = ament_index_cpp::get_package_share_directory("crs_support") + "/toolpaths/scanned_part1/job_90degrees.yaml";
  }
private:
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr joint_msg)
  {
    curr_joint_state_ = *joint_msg;
  }
  void planService(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {

      // Load rasters and get them in usable form
      std::string waypoint_origin_frame = "part";
      std::vector<geometry_msgs::msg::PoseArray> raster_strips;
      bool success = crs_motion_planning::parsePathFromFile(toolpath_filepath_, waypoint_origin_frame, raster_strips);
      geometry_msgs::msg::PoseArray strip_of_interset;
      for (auto strip : raster_strips)
      {
          strip_of_interset.poses.insert(strip_of_interset.poses.end(), strip.poses.begin(), strip.poses.end());
      }

      // Display rasters on part
      visualization_msgs::msg::MarkerArray mark_array_msg;
      crs_motion_planning::rasterStripsToMarkerArray(strip_of_interset, waypoint_origin_frame, mark_array_msg, {1.0, 0.0, 0.0, 1.0}, -0.01);
      original_path_publisher_->publish(mark_array_msg);

      // Get transform between world and part
      tf2::TimePoint time_point = tf2::TimePointZero;
      geometry_msgs::msg::TransformStamped world_to_goal_frame;
      try
      {
        world_to_goal_frame = tf_buffer_.lookupTransform("world", "part", time_point);
      }
      catch (tf2::LookupException &e)
      {
        response->success = false;
        response->message = "TF lookup failed: " + std::string(e.what());
        return;
      }

      // Convert rasters into world frame
      geometry_msgs::msg::PoseArray strip_of_interest_world_frame;
      for (size_t i = 0; i < strip_of_interset.poses.size(); ++i)
      {
          geometry_msgs::msg::PoseStamped surface_pose_world_frame, surface_pose_og_frame;
          surface_pose_og_frame.pose = strip_of_interset.poses[i];
          surface_pose_og_frame.header = strip_of_interset.header;
          tf2::doTransform(surface_pose_og_frame, surface_pose_world_frame, world_to_goal_frame);
          geometry_msgs::msg::Pose sf_pose_wf = surface_pose_world_frame.pose;
          strip_of_interest_world_frame.poses.push_back(std::move(sf_pose_wf));
          Eigen::Isometry3d strip_og_eigen, strip_converted_eigen;
          tf2::fromMsg(surface_pose_og_frame.pose, strip_og_eigen);
          tf2::fromMsg(surface_pose_world_frame.pose, strip_converted_eigen);
      }

      // Set up planning config variable
      auto path_plan_config_ptr = std::make_shared<crs_motion_planning::pathPlanningConfig>();
      path_plan_config_ptr->tesseract_local = tesseract_local_;
      path_plan_config_ptr->manipulator = manipulator_;
      path_plan_config_ptr->world_frame = "world";
      path_plan_config_ptr->robot_base_frame = "base_link";
      path_plan_config_ptr->tool0_frame = "tool0";
      path_plan_config_ptr->tcp_frame = "sander_center_link";

      // Create crsMotionPlanner class
      crs_motion_planning::crsMotionPlanner crs_motion_planner(path_plan_config_ptr);

      // Determine reachability of all rasters using descartes
      std::vector<size_t> failed_edges, failed_vertices;
      trajectory_msgs::msg::JointTrajectory joint_traj_msg_out_init, joint_traj_msg_out_final;
      Eigen::MatrixXd joint_eigen_out1;

      bool gen_preplan = crs_motion_planner.generateDescartesSeed(strip_of_interest_world_frame,
                                                                  0.1,
                                                                  false,
                                                                  0.0075,
                                                                  failed_edges,
                                                                  failed_vertices,
                                                                  joint_traj_msg_out_init,
                                                                  joint_eigen_out1);


      std::vector<geometry_msgs::msg::PoseArray> resplit_rasters;
      geometry_msgs::msg::PoseArray reachable_rasters;
      Eigen::MatrixXd joint_eigen_out;
      // Check if all rasters reachable
      if (!gen_preplan)
      {
          // Split up raster based on where planning failures occurred
          geometry_msgs::msg::PoseArray failed_vertex_poses;
          crs_motion_planning::cleanRasterStrip(strip_of_interest_world_frame,
                                                failed_vertices,
                                                resplit_rasters,
                                                failed_vertex_poses);

          // Display failed vertices
          visualization_msgs::msg::Marker failed_vertex_markers;
          crs_motion_planning::failedEdgesToMarkerArray(failed_vertex_poses,
                                                        "world",
                                                        failed_vertex_markers,
                                                        {1.0, 1.0, 0.0, 0.0},
                                                        0.01);
          failed_vertex_publisher_->publish(failed_vertex_markers);


          // Combine raster into a continous strip
          for (auto strip : resplit_rasters)
          {
              reachable_rasters.poses.insert(reachable_rasters.poses.end(), strip.poses.begin(), strip.poses.end());
          }

          // Display now reachable raster points
          visualization_msgs::msg::MarkerArray mark_array_fixed_msg;
          crs_motion_planning::rasterStripsToMarkerArray(reachable_rasters, "world", mark_array_fixed_msg, {1.0, 0.0, 1.0, 0.0}, -0.025);
          corrected_path_publisher_->publish(mark_array_fixed_msg);

          // Generate Descartes preplan
          gen_preplan = crs_motion_planner.generateDescartesSeed(reachable_rasters,
                                                                 0.05,
                                                                 false,
                                                                 0.0075,
                                                                 failed_edges,
                                                                 failed_vertices,
                                                                 joint_traj_msg_out_final,
                                                                 joint_eigen_out);
      }
      else
      {
          joint_traj_msg_out_final = joint_traj_msg_out_init;
          reachable_rasters = strip_of_interest_world_frame;
      }


      // Check if successfully generated preplan with descartes
      if (gen_preplan)
      {
          // Split rasters based on signigicant joint motions
          std::vector<trajectory_msgs::msg::JointTrajectory> split_traj;
          std::vector<geometry_msgs::msg::PoseArray> reresplit_rasters;
          double desired_ee_val = 0.5, max_joint_vel = 4.0;
          std::vector<std::vector<double>> time_steps;
          crs_motion_planning::splitRastersByJointDist(joint_traj_msg_out_final,
                                                       reachable_rasters,
                                                       desired_ee_val,
                                                       max_joint_vel,
                                                       split_traj,
                                                       reresplit_rasters,
                                                       time_steps);

          // Run trajectories through trajopt
          std::string target_frame = "sander_center_link";
          Eigen::Isometry3d tcp_eigen;
          tcp_eigen.setIdentity();
          auto traj_pc = std::make_shared<tesseract_motion_planners::TrajOptPlannerDefaultConfig>(tesseract_local_, manipulator_, target_frame, tcp_eigen);
          // TODO: FINISH TRAJOPT TRAJECTORY OPTIMIZATION OF DESCARTES PREPLAN
          tesseract_motion_planners::CollisionCostConfig coll_cost_config;
          coll_cost_config.enabled = false;
          traj_pc->collision_cost_config = coll_cost_config;

          tesseract_motion_planners::CollisionConstraintConfig coll_cnt_config;
          coll_cnt_config.enabled = true;
          coll_cnt_config.safety_margin = 0.003;
          traj_pc->collision_constraint_config = coll_cnt_config;

          traj_pc->init_type = trajopt::InitInfo::GIVEN_TRAJ;
          traj_pc->longest_valid_segment_fraction = 0.01;

          traj_pc->smooth_velocities = false;
          traj_pc->smooth_accelerations = false;
          traj_pc->smooth_jerks = false;

          Eigen::VectorXd surface_coeffs(6);
          surface_coeffs << 10, 10, 10, 10, 10, 0;

          std::vector<std::vector<tesseract_motion_planners::Waypoint::Ptr>> waypoints;
          bool waypoints_critical = true;
          for (size_t i = 0; i < reresplit_rasters.size(); ++i)
          {
              std::vector<tesseract_motion_planners::Waypoint::Ptr> curr_raster;
              for (auto waypoint : reresplit_rasters[i].poses)
              {
                  Eigen::Vector3d surface_pose(waypoint.position.x, waypoint.position.y, waypoint.position.z);
                  Eigen::Quaterniond surface_ori(waypoint.orientation.w, waypoint.orientation.x, waypoint.orientation.y, waypoint.orientation.z);
                  tesseract_motion_planners::CartesianWaypoint::Ptr surface_waypoint = std::make_shared<tesseract_motion_planners::CartesianWaypoint>(surface_pose, surface_ori);
                  surface_waypoint->setCoefficients(surface_coeffs);
                  surface_waypoint->setIsCritical(waypoints_critical);
                  curr_raster.push_back(std::move(surface_waypoint));
              }
              waypoints.push_back(curr_raster);
          }
          traj_pc->target_waypoints = waypoints[0];

          Eigen::MatrixXd joint_eigen_from_jt = tesseract_rosutils::toEigen(split_traj[0],split_traj[0].joint_names);

          traj_pc->seed_trajectory = joint_eigen_from_jt;

          trajectory_msgs::msg::JointTrajectory trajopt_result_traj;
          tesseract_motion_planners::PlannerResponse planner_resp;
          tesseract_motion_planners::TrajOptMotionPlanner traj_surface_planner;
          traj_surface_planner.setConfiguration(traj_pc);
          std::cout << "Solving" << std::endl;
          traj_surface_planner.solve(planner_resp);
          std::cout << "DONE" << std::endl;

          if (planner_resp.status.value() < 0)
          {
              response->success = false;
              response->message = planner_resp.status.message();
              return;
          }
          Eigen::MatrixXd result_traj(planner_resp.joint_trajectory.trajectory.rows(), planner_resp.joint_trajectory.trajectory.cols());
          result_traj << planner_resp.joint_trajectory.trajectory;
          crs_motion_planning::tesseractRosutilsToMsg(trajopt_result_traj, split_traj[0].joint_names, result_traj);

          split_traj.push_back(trajopt_result_traj);
          // Assign trajectory timestamps for motion execution
          std::vector<double> traj_times;
          for (size_t i = 0; i < split_traj.size(); ++i)
          {
              std::cout << "Raster: " << i << " " << split_traj[i].points.size() << std::endl;
              split_traj[i].header.frame_id  = "world";
              size_t i2;
              if (i == 2)
              {
                  i2 = 0;
              }
              else{
                i2 = i;
              }
              double curr_traj_time = 0;
              for (size_t j = 1; j < split_traj[i].points.size(); ++j)
              {
                  split_traj[i].points[j-1].time_from_start.sec = static_cast<int>(floor(time_steps[i2][j]));
                  split_traj[i].points[j-1].time_from_start.nanosec = static_cast<uint>(1e9 * (time_steps[i2][j] - floor(time_steps[i2][j])));
                  curr_traj_time += time_steps[i2][j];
              }
              std::cout << "Done" << std::endl;
              split_traj[i].points.end()->time_from_start.sec = 0;
              traj_times.push_back(curr_traj_time);
          }

          // Publish trajectories with a 2 second pause in between
          for (size_t i = 0; i < split_traj.size(); ++i)
          {
              traj_publisher_->publish(split_traj[i]);
              std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(ceil(traj_times[i]))+2));
          }
          response->success = success;
          response->message = "Loaded";
      }
      else
      {
          response->success = false;
          response->message = "Failed to generate preplan";
      }

  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_paths_service_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr original_path_publisher_, corrected_path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr failed_vertex_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_listener_;
  tesseract::Tesseract::Ptr tesseract_local_;

  std::shared_ptr<rclcpp::Clock> clock_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  sensor_msgs::msg::JointState curr_joint_state_;

  std::string base_link_frame_;
  std::string manipulator_;
  std::string toolpath_filepath_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SurfaceServer>());
  rclcpp::shutdown();
  return 0;
}
