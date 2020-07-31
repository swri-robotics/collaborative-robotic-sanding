#ifndef CRS_MOTION_PLANNING_PATH_PLANNING_UTILS_H
#define CRS_MOTION_PLANNING_PATH_PLANNING_UTILS_H

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_storage.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

#include <tesseract_common/types.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_default_config.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_freespace_config.h>
#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/config/ompl_planner_freespace_config.h>
#include <tesseract_motion_planners/core/trajectory_validator.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_rosutils/utils.h>

#include <descartes_light/descartes_light.h>
#include <descartes_samplers/evaluators/distance_edge_evaluator.h>
#include <descartes_samplers/evaluators/euclidean_distance_edge_evaluator.h>
#include <descartes_samplers/samplers/axial_symmetric_sampler.h>
#include <descartes_samplers/samplers/cartesian_point_sampler.h>

#include <crs_motion_planning/path_processing_utils.h>

#include <ur_ikfast_kinematics/descartes_ikfast_ur10e.h>

#include <Eigen/Eigen>
#include <vector>

namespace crs_motion_planning
{
struct descartesConfig
{
  using Ptr = std::shared_ptr<descartesConfig>;

  double axial_step = M_PI / 36;
  double collision_safety_margin = 0.01;

  bool allow_collisions = false;

  Eigen::Isometry3d tool_offset = Eigen::Isometry3d::Identity();
};

struct trajoptSurfaceConfig
{
  bool smooth_velocities = true;
  bool smooth_accelerations = true;
  bool smooth_jerks = true;

  tesseract_motion_planners::CollisionCostConfig coll_cst_cfg;
  tesseract_motion_planners::CollisionConstraintConfig coll_cnt_cfg;

  trajopt::InitInfo::Type init_type = trajopt::InitInfo::GIVEN_TRAJ;

  double longest_valid_segment_fraction = 0.01;
  double longest_valid_segment_length = 0.5;

  Eigen::VectorXd surface_coeffs;  // Defaults to 6 10s

  bool waypoints_critical = true;

  std::vector<std::tuple<std::string, std::string, double, double>> special_collision_cost;
  std::vector<std::tuple<std::string, std::string, double, double>> special_collision_constraint;
};

struct omplConfig
{
  double collision_safety_margin = 0.01;
  double planning_time = 5.0;  // seconds

  bool simplify = true;
  bool collision_continuous = true;
  bool collision_check = true;

  double range = 0.25;

  Eigen::VectorXd weights;

  int num_threads = 1;
  int max_solutions = 10;

  int n_output_states = 20;

  double longest_valid_segment_fraction = 0.01;
  double longest_valid_segment_length = 0.5;
};

struct trajoptFreespaceConfig
{
  bool smooth_velocities = true;
  bool smooth_accelerations = true;
  bool smooth_jerks = true;

  tesseract_motion_planners::CollisionCostConfig coll_cst_cfg;
  tesseract_motion_planners::CollisionConstraintConfig coll_cnt_cfg;

  trajopt::InitInfo::Type init_type = trajopt::InitInfo::GIVEN_TRAJ;

  double longest_valid_segment_fraction = 0.01;
  double longest_valid_segment_length = 0.5;

  tesseract_collision::ContactTestType contact_test_type = tesseract_collision::ContactTestType::CLOSEST;

  std::vector<std::tuple<std::string, std::string, double, double>> special_collision_cost;
  std::vector<std::tuple<std::string, std::string, double, double>> special_collision_constraint;
};

struct pathPlanningConfig
{
  using Ptr = std::unique_ptr<pathPlanningConfig>;

  descartesConfig descartes_config;

  trajoptSurfaceConfig trajopt_surface_config;

  omplConfig ompl_config;

  trajoptFreespaceConfig trajopt_freespace_config;
  bool use_trajopt_freespace = true;
  bool use_trajopt_surface = true;
  bool default_to_descartes = false;

  tesseract::Tesseract::Ptr tesseract_local;

  bool use_start = false;
  tesseract_motion_planners::JointWaypoint::Ptr start_pose;
  bool use_end = false;
  tesseract_motion_planners::JointWaypoint::Ptr end_pose;
  bool simplify_start_end_freespace = true;

  std::vector<geometry_msgs::msg::PoseArray> rasters;  // In world frame

  std::string manipulator = "manipulator";

  std::string world_frame = "world";
  std::string robot_base_frame = "base_link";
  std::string tool0_frame = "tool0";
  std::string tcp_frame;

  Eigen::Isometry3d tool_offset = Eigen::Isometry3d::Identity();

  bool add_approach_and_retreat = false;
  double approach_distance = 0.05;
  double retreat_distance = 0.05;

  bool required_tool_vel = false;
  double tool_speed = 0.03;    // m/s
  double max_joint_vel = 0.2;  // rad/s
  double max_joint_acc = 0.5;  // rad/s^2

  size_t minimum_raster_length = 2;

  bool trajopt_verbose_output = false;

  bool use_gazebo_sim_timing = false;

  bool combine_strips = false;
  bool global_descartes = true;
};

struct pathPlanningResults
{
  using Ptr = std::unique_ptr<pathPlanningResults>;

  geometry_msgs::msg::PoseArray reachable_waypoints;
  geometry_msgs::msg::PoseArray unreachable_waypoints;

  std::vector<trajectory_msgs::msg::JointTrajectory> descartes_trajectory_results;

  std::vector<geometry_msgs::msg::PoseArray> skipped_rasters;
  std::vector<geometry_msgs::msg::PoseArray> solved_rasters;
  std::vector<geometry_msgs::msg::PoseArray> failed_rasters;

  std::vector<trajectory_msgs::msg::JointTrajectory> final_raster_trajectories;

  std::vector<trajectory_msgs::msg::JointTrajectory> ompl_trajectories;
  std::vector<trajectory_msgs::msg::JointTrajectory> final_freespace_trajectories;

  std::vector<trajectory_msgs::msg::JointTrajectory> ompl_start_end_trajectories;
  std::vector<trajectory_msgs::msg::JointTrajectory> final_start_end_trajectories;

  std::vector<trajectory_msgs::msg::JointTrajectory> final_trajectories;
  std::string msg_out;
};

bool loadPathPlanningConfig(const std::string& yaml_fp,
                            pathPlanningConfig::Ptr& motion_planner_config);

class crsMotionPlanner
{
public:
  crsMotionPlanner(pathPlanningConfig::Ptr config, rclcpp::Logger logger);
  crsMotionPlanner(pathPlanningConfig config, rclcpp::Logger logger);
  ~crsMotionPlanner() = default;

  void updateConfiguration(pathPlanningConfig::Ptr config);

  ///
  /// \brief generateDescartesSeed Creates a seed trajectory using descartes
  /// \param waypoints
  /// \return success
  ///
  bool generateDescartesSeed(const geometry_msgs::msg::PoseArray& waypoints,
                             std::vector<std::size_t>& failed_edges,
                             std::vector<std::size_t>& failed_vertices,
                             trajectory_msgs::msg::JointTrajectory& joint_trajectory);

  ///
  /// \brief generateSurfacePlans Creates surface trajectories given a set of rasters
  /// \return success
  ///
  bool generateSurfacePlans(pathPlanningResults::Ptr& results);

  ///
  /// \brief generateOMPLSeed Creates freespace trajectory given a start and end joint state
  /// \return success
  ///
  bool generateOMPLSeed(const tesseract_motion_planners::JointWaypoint::Ptr& start_pose,
                        const tesseract_motion_planners::JointWaypoint::Ptr& end_pose,
                        tesseract_common::JointTrajectory& seed_trajectory);

  ///
  /// \brief trajoptFreespace Uses ompl seed to generate a trajopt plan
  /// \return success
  ///
  bool trajoptFreespaceFromOMPL(const tesseract_motion_planners::JointWaypoint::Ptr& start_pose,
                                const tesseract_motion_planners::JointWaypoint::Ptr& end_pose,
                                const tesseract_common::JointTrajectory& seed_trajectory,
                                trajectory_msgs::msg::JointTrajectory& joint_trajectory);
  ///
  /// \brief generateFreespacePlans Creates freespace trajectories given a set of surface rasters
  /// \return success
  ///
  bool generateFreespacePlans(pathPlanningResults::Ptr& results);

  ///
  /// \brief generateProcessPlan Creates process plans given a set of rasters, including freespace and surface
  /// trajectories \return success
  ///
  bool generateProcessPlan(pathPlanningResults::Ptr& results);

  ///
  /// \brief generateFreespacePlan Creates a freespace plans given a start and end joint position
  /// \return success
  ///
  bool generateFreespacePlan(const tesseract_motion_planners::JointWaypoint::Ptr& start_pose,
                             const tesseract_motion_planners::JointWaypoint::Ptr& end_pose,
                             trajectory_msgs::msg::JointTrajectory& joint_trajectory);

  ///
  /// \brief generateFreespacePlan Creates a freespace plans given a start joint position and end cartesian position
  /// \return success
  ///
  bool generateFreespacePlan(const tesseract_motion_planners::JointWaypoint::Ptr& start_pose,
                             const tesseract_motion_planners::CartesianWaypoint::Ptr& end_pose,
                             trajectory_msgs::msg::JointTrajectory& joint_trajectory);

  ///
  /// \brief findClosestJointOrientation Finds the closest joint state given start pose at the end cartesian pose
  /// \return success
  ///
  bool findClosestJointOrientation(const tesseract_motion_planners::JointWaypoint::Ptr& start_pose,
                                   const tesseract_motion_planners::CartesianWaypoint::Ptr& end_pose,
                                   tesseract_motion_planners::JointWaypoint::Ptr& returned_pose,
                                   const double& axial_step = -1);

protected:
  pathPlanningConfig::Ptr config_;
  rclcpp::Logger logger_;
};

}  // namespace crs_motion_planning

#endif  // CRS_MOTION_PLANNING_PATH_PLANNING_UTILS_H
