#include <rclcpp/rclcpp.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

//#include <crs_motion_planning/path_processing_utils.h>
#include <crs_motion_planning/path_planning_utils.h>

#include <tf2/transform_storage.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

//#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
//#include <tesseract_motion_planners/trajopt/config/trajopt_planner_default_config.h>
//#include <tesseract_motion_planners/trajopt/config/trajopt_planner_freespace_config.h>
//#include <tesseract_motion_planners/descartes/descartes_collision.h>
//#include <tesseract_environment/core/environment.h>
//#include <tesseract_environment/core/utils.h>

//#include <trajopt/problem_description.hpp>

//#include <tesseract_rosutils/utils.h>
//#include <tesseract_rosutils/conversions.h>

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
    traj_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("set_trajectory_test",1);
    original_path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("original_raster_paths",1);
    corrected_path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("fixed_raster_paths",1);
    failed_vertex_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("failed_vertices",1);
    joint_state_listener_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1, std::bind(&SurfaceServer::jointCallback, this, std::placeholders::_1));

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
      crs_motion_planning::parsePathFromFile(toolpath_filepath_, waypoint_origin_frame, raster_strips);
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

      std::vector<geometry_msgs::msg::PoseArray> raster_strips_world_frame;
      for (auto strip : raster_strips)
      {
          geometry_msgs::msg::PoseArray curr_strip;
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

      // Set up planning config variable
      crs_motion_planning::descartesConfig descartes_config;
      descartes_config.axial_step = 0.1;
      descartes_config.collision_safety_margin = 0.0075;

      crs_motion_planning::trajoptSurfaceConfig trajopt_surface_config;
      trajopt_surface_config.smooth_velocities = false;
      trajopt_surface_config.smooth_accelerations = false;
      trajopt_surface_config.smooth_jerks = false;
      tesseract_motion_planners::CollisionCostConfig coll_cost_config;
      coll_cost_config.enabled = false;
      trajopt_surface_config.coll_cst_cfg = coll_cost_config;
      tesseract_motion_planners::CollisionConstraintConfig coll_cnt_config;
      coll_cnt_config.enabled = true;
      coll_cnt_config.safety_margin = 0.001;
      trajopt_surface_config.coll_cnt_cfg = coll_cnt_config;
      Eigen::VectorXd surface_coeffs(6);
      surface_coeffs << 10, 10, 10, 10, 10, 0;
      trajopt_surface_config.surface_coeffs = surface_coeffs;

      Eigen::Isometry3d tool_offset;
      tool_offset.setIdentity();

      auto path_plan_config = std::make_shared<crs_motion_planning::pathPlanningConfig>();
      path_plan_config->tesseract_local = tesseract_local_;
      path_plan_config->descartes_config = descartes_config;
      path_plan_config->trajopt_surface_config = trajopt_surface_config;
      path_plan_config->manipulator = manipulator_;
      path_plan_config->world_frame = "world";
      path_plan_config->robot_base_frame = "base_link";
      path_plan_config->tool0_frame = "tool0";
      path_plan_config->tcp_frame = "sander_center_link";
      path_plan_config->rasters = raster_strips_world_frame;
      path_plan_config->smooth_velocities = false;
      path_plan_config->smooth_accelerations = false;
      path_plan_config->smooth_jerks = false;
      path_plan_config->max_joint_vel = 4.0;
      path_plan_config->tool_speed = 0.5;
      path_plan_config->tool_offset = tool_offset;

      // Create crsMotionPlanner class
      crs_motion_planning::crsMotionPlanner crs_motion_planner(path_plan_config);

      auto path_plan_results = std::make_shared<crs_motion_planning::pathPlanningResults>();
      bool success = crs_motion_planner.generateProcessPlan(path_plan_results);

      std::vector<trajectory_msgs::msg::JointTrajectory> trajopt_trajectories;
      trajopt_trajectories = path_plan_results->final_raster_trajectories;

      if (success)
      {
          for (size_t i = 0; i < trajopt_trajectories.size(); ++i)
          {
              std::cout << "PUBLISHING TRAJECTORY " << i+1 << " OF " << trajopt_trajectories.size() << std::endl;
              traj_publisher_->publish(trajopt_trajectories[i]);
              std::this_thread::sleep_for(std::chrono::seconds(2));
          }
          std::cout << "ALL DONE" << std::endl;
          response->success = true;
          response->message = "TRAJECTORIES PUBLISHED";
      }
      else
      {
          response->success = false;
          response->message = "Failed to generate preplan";
      }
//      // Determine reachability of all rasters using descartes
//      trajectory_msgs::msg::JointTrajectory joint_traj_msg_out_init, joint_traj_msg_out_final;
//      std::cout << "RUNNING FIRST DESCARTES" << std::endl;

//      bool gen_preplan;
//      std::vector<geometry_msgs::msg::PoseArray> split_reachable_rasters; // HAS FAILED
//      std::vector<trajectory_msgs::msg::JointTrajectory> split_traj;
//      bool any_successes = false;
//      size_t count_strips = 0;
//      geometry_msgs::msg::PoseArray failed_vertex_poses;

//      for (auto strip : raster_strips_world_frame)
//      {
//          std::vector<size_t> failed_edges, failed_vertices;
//          gen_preplan = crs_motion_planner.generateDescartesSeed(strip,
//                                                                 failed_edges,
//                                                                 failed_vertices,
//                                                                 joint_traj_msg_out_init);
//          path_plan_config->descartes_config.axial_step = 0.05;
//          crs_motion_planner.updateConfiguration(path_plan_config);
//          std::cout << "DONE" << std::endl;


//          // Check if all rasters reachable
//          if (!gen_preplan)
//          {
//              std::vector<geometry_msgs::msg::PoseArray> split_rasters;
//              // Split up raster based on where planning failures occurred
//              geometry_msgs::msg::PoseArray curr_failed_vertex_poses;
//              std::cout << "CLEANING" << std::endl;
//              crs_motion_planning::cleanRasterStrip(strip,
//                                                    failed_vertices,
//                                                    split_rasters,
//                                                    curr_failed_vertex_poses);

//              // Display failed vertices
//              failed_vertex_poses.poses.insert(failed_vertex_poses.poses.end(), curr_failed_vertex_poses.poses.begin(), curr_failed_vertex_poses.poses.end());
//              visualization_msgs::msg::Marker failed_vertex_markers;
//              std::cout << "PUBLISHING MARKER ARRAY" << std::endl;
//              crs_motion_planning::failedEdgesToMarkerArray(failed_vertex_poses,
//                                                            "world",
//                                                            failed_vertex_markers,
//                                                            {1.0, 1.0, 0.0, 0.0},
//                                                            0.01);
//              failed_vertex_publisher_->publish(failed_vertex_markers);

//              for (auto split_strip : split_rasters)
//              {
//                  // Generate Descartes preplan
//                  std::cout << "DESCARTES ROUND 2 ELECTRIC BOOGALO" << std::endl;
//                  if (crs_motion_planner.generateDescartesSeed(split_strip,
//                                                               failed_edges,
//                                                               failed_vertices,
//                                                               joint_traj_msg_out_final) && split_strip.poses.size()>1)
//                  {
//                    split_traj.push_back(joint_traj_msg_out_final);
//                    split_reachable_rasters.push_back(std::move(split_strip));
//                    any_successes = true;
//                    std::cout << "SUCCESS" << std::endl;
//                  }
//                  std::cout << "DONE" << std::endl;
//              }
//          }
//          else
//          {
//              std::cout << "SUCCESS" << std::endl;
//              split_traj.push_back(joint_traj_msg_out_init);
//              split_reachable_rasters.push_back(std::move(strip));
//          }
//          std::cout << "Strip " << ++count_strips << " of " << raster_strips_world_frame.size() << std::endl;
//      }
//      // Display now reachable raster points
//      visualization_msgs::msg::MarkerArray mark_array_fixed_msg;
//      crs_motion_planning::rasterStripsToMarkerArray(split_reachable_rasters, "world", mark_array_fixed_msg, {1.0, 0.0, 1.0, 0.0}, -0.025);
//      corrected_path_publisher_->publish(mark_array_fixed_msg);

//      // Check if successfully generated preplan with descartes
//      if (any_successes)
//      {
//          std::vector<trajectory_msgs::msg::JointTrajectory> final_split_traj;
//          std::vector<geometry_msgs::msg::PoseArray> final_split_rasters;
//          std::vector<std::vector<double>> final_time_steps;
//          size_t raster_n = 0;
//          std::cout << "CHECKING FOR SPLITS IN " << split_traj.size() << " TRAJECTORIES" << std::endl;
//          for (auto curr_joint_traj : split_traj)
//          {
//              // Split rasters based on signigicant joint motions
//              std::vector<trajectory_msgs::msg::JointTrajectory> double_split_traj;
//              std::vector<geometry_msgs::msg::PoseArray> resplit_rasters;
//              double desired_ee_val = 0.5, max_joint_vel = 4.0;
//              std::vector<std::vector<double>> time_steps;
//              if(crs_motion_planning::splitRastersByJointDist(curr_joint_traj,
//                                                           split_reachable_rasters[raster_n],
//                                                           desired_ee_val,
//                                                           max_joint_vel,
//                                                           double_split_traj,
//                                                           resplit_rasters,
//                                                           time_steps))
//              {
//                  std::cout << "FOUND A SPLIT" << std::endl;
//                  for (size_t i = 0; i < resplit_rasters.size(); ++i)
//                  {
//                      geometry_msgs::msg::PoseArray modified_raster;
//                      double approach = 0.01, retreat = 0.01;
//                      std::vector<double> modified_time_steps = time_steps[i];
//                      if (i == 0)
//                      {
//                          // Add departure to end of resplit_rasters[i]
//                          crs_motion_planning::addApproachAndRetreat(resplit_rasters[i], 0, retreat, modified_raster);
//                          // ADD TIME STEPS
//                      }
//                      else if (i == double_split_traj.size() - 1)
//                      {
//                          // Add entrance to end of resplit_rasters[i]
//                          crs_motion_planning::addApproachAndRetreat(resplit_rasters[i], approach, retreat, modified_raster);
//                          // ADD TIME STEPS
//                      }
//                      else
//                      {
//                          // Add entrance and departure to resplit_rasters[i]
//                          crs_motion_planning::addApproachAndRetreat(resplit_rasters[i], approach, 0, modified_raster);
//                          // ADD TIME STEPS
//                      }

//                      trajectory_msgs::msg::JointTrajectory modified_joint_traj_msg_out;
//                      // Generate Descartes preplan
//                      std::cout << "DESCARTES ROUND 3, DESCARTES WITH A VEGENCE" << std::endl;
//                      std::vector<size_t> failed_edges, failed_vertices;

//                      if (crs_motion_planner.generateDescartesSeed(modified_raster,
//                                                                   failed_edges,
//                                                                   failed_vertices,
//                                                                   modified_joint_traj_msg_out))
//                      {
//                          std::cout << "SUCCESS" << std::endl;
//                          for (int j = 0; j < 1; ++j)
//                          {
//                              final_split_rasters.push_back(modified_raster);
//                              final_split_traj.push_back(modified_joint_traj_msg_out);
//                              final_time_steps.push_back(modified_time_steps);
//                          }
//                      }
//                      std::cout << "DONE" << std::endl;
//                  }
//              }
//              else
//              {
//                  std::cout << "NO SPLITS FOR YOU" << std::endl;
//                  for (int j = 0; j < 1; ++j)
//                  {
//                      final_split_rasters.push_back(split_reachable_rasters[raster_n]);
//                      final_split_traj.push_back(std::move(curr_joint_traj));
//                      final_time_steps.push_back(time_steps[0]);
//                  }
//              }
//              raster_n++;
//          }
//          std::cout << "TIME TO OPTIMIZE" << std::endl;

//          //**************************************************TRAJOPT***************************************************
//          // Run trajectories through trajopt
//          std::string target_frame = "sander_center_link";
//          Eigen::Isometry3d tcp_eigen;
//          tcp_eigen.setIdentity();

//          Eigen::VectorXd surface_coeffs(6);
//          surface_coeffs << 10, 10, 10, 10, 10, 0;
//          std::cout << "BUILT CONFIG SETTINGS" << std::endl;

//          std::vector<trajectory_msgs::msg::JointTrajectory> trajopt_trajectories;
//          std::vector<bool> trajopt_solved;
//          bool waypoints_critical = true;
//          for (size_t i = 0; i < final_split_rasters.size(); ++i)
//          {
//              std::vector<tesseract_motion_planners::Waypoint::Ptr> curr_raster;
//              std::cout << "BUILDING WAYPOINT SET " << i+1 << " OF " << final_split_rasters.size() << std::endl;
//              for (auto waypoint : final_split_rasters[i].poses)
//              {
//                  Eigen::Vector3d surface_pose(waypoint.position.x, waypoint.position.y, waypoint.position.z);
//                  Eigen::Quaterniond surface_ori(waypoint.orientation.w, waypoint.orientation.x, waypoint.orientation.y, waypoint.orientation.z);
//                  tesseract_motion_planners::CartesianWaypoint::Ptr surface_waypoint = std::make_shared<tesseract_motion_planners::CartesianWaypoint>(surface_pose, surface_ori);
//                  surface_waypoint->setCoefficients(surface_coeffs);
//                  surface_waypoint->setIsCritical(waypoints_critical);
//                  curr_raster.push_back(std::move(surface_waypoint));
//              }

//              auto traj_pc = std::make_shared<tesseract_motion_planners::TrajOptPlannerDefaultConfig>(tesseract_local_, manipulator_, target_frame, tcp_eigen);

//              tesseract_motion_planners::CollisionCostConfig coll_cost_config;
//              coll_cost_config.enabled = false;
//              traj_pc->collision_cost_config = coll_cost_config;

//              tesseract_motion_planners::CollisionConstraintConfig coll_cnt_config;
//              coll_cnt_config.enabled = true;
//              coll_cnt_config.safety_margin = 0.001;
//              traj_pc->collision_constraint_config = coll_cnt_config;

//              traj_pc->init_type = trajopt::InitInfo::GIVEN_TRAJ;
//              traj_pc->longest_valid_segment_fraction = 0.01;

//              traj_pc->smooth_velocities = false;
//              traj_pc->smooth_accelerations = false;
//              traj_pc->smooth_jerks = false;
//              traj_pc->target_waypoints = curr_raster;

//              Eigen::MatrixXd joint_eigen_from_jt;
////              joint_eigen_from_jt = tesseract_rosutils::toEigen(final_split_traj[i],final_split_traj[i].joint_names);

//              traj_pc->seed_trajectory = joint_eigen_from_jt;

////              path_plan_config->trajopt_surface_config = traj_pc;

//              trajectory_msgs::msg::JointTrajectory trajopt_result_traj;
//              tesseract_motion_planners::PlannerResponse planner_resp;
//              tesseract_motion_planners::TrajOptMotionPlanner traj_surface_planner;
//              traj_surface_planner.setConfiguration(traj_pc);
//              std::cout << "Solving raster: " << i+1 << " of " << final_split_rasters.size() << std::endl;
//              traj_surface_planner.solve(planner_resp);

//              if (planner_resp.status.value() < 0)
//              {
//                  std::cout << "FAILED: " << planner_resp.status.message() << std::endl;
//                  trajopt_solved.push_back(false);
//              }
//              else
//              {
//                  std::cout << "SUCCEEDED" << std::endl;
//                  Eigen::MatrixXd result_traj(planner_resp.joint_trajectory.trajectory.rows(), planner_resp.joint_trajectory.trajectory.cols());
//                  result_traj << planner_resp.joint_trajectory.trajectory;
//                  crs_motion_planning::tesseractRosutilsToMsg(trajopt_result_traj, final_split_traj[i].joint_names, result_traj);

//                  trajopt_trajectories.push_back(std::move(trajopt_result_traj));
//                  trajopt_solved.push_back(true);
//              }
//          }
//          //**************************************************TRAJOPT***************************************************
//          std::cout << "TIME TO GET THIS WHOLE TIME THING DOWN" << std::endl;
//          // Assign trajectory timestamps for motion execution
//          std::vector<double> traj_times;
//          size_t trajopt_traj_n = 0;
//          for (size_t i = 0; i < final_split_traj.size(); ++i)
//          {
//              if (trajopt_solved[i])
//              {
//                  std::cout << "Raster: " << trajopt_traj_n + 1 << " of " << trajopt_trajectories.size() << " with " << trajopt_trajectories[trajopt_traj_n].points.size() << " waypoints " << std::endl;
//                  trajopt_trajectories[trajopt_traj_n].header.frame_id  = "world";
//                  double curr_traj_time = 0;
//                  for (size_t j = 1; j < trajopt_trajectories[trajopt_traj_n].points.size(); ++j)
//                  {
//                      trajopt_trajectories[trajopt_traj_n].points[j-1].time_from_start.sec = static_cast<int>(floor(final_time_steps[i][j]));
//                      trajopt_trajectories[trajopt_traj_n].points[j-1].time_from_start.nanosec = static_cast<uint>(1e9 * (final_time_steps[i][j] - floor(final_time_steps[i][j])));
//                      curr_traj_time += final_time_steps[i][j];
//                  }
//                  trajopt_trajectories[trajopt_traj_n].points.back().time_from_start.sec = 0;
//                  traj_times.push_back(std::move(curr_traj_time));
//                  trajopt_traj_n++;
//              }
//          }

//          std::cout << "TIMES UP WITH THE TIME THING... TIME TO SHOW OFF OUR TRAJECTORIES" << std::endl;

//          // Publish trajectories with a 2 second pause in between
//          for (size_t i = 0; i < trajopt_trajectories.size(); ++i)
//          {
//              std::cout << "PUBLISHING TRAJECTORY " << i+1 << " OF " << trajopt_trajectories.size() << std::endl;
//              traj_publisher_->publish(trajopt_trajectories[i]);
//              std::this_thread::sleep_for(std::chrono::seconds(2));
//          }
//          std::cout << "ALL DONE" << std::endl;
//          response->success = true;
//          response->message = "TRAJECTORIES PUBLISHED";
//      }
//      else
//      {
//          response->success = false;
//          response->message = "Failed to generate preplan";
//      }
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
