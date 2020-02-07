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
#include <tesseract_motion_planners/ompl/ompl_freespace_planner.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_rosutils/utils.h>
//#include <tesseract_rosutils/conversions.h>

#include <descartes_light/descartes_light.h>
#include <descartes_samplers/evaluators/distance_edge_evaluator.h>
#include <descartes_samplers/evaluators/euclidean_distance_edge_evaluator.h>
#include <descartes_samplers/samplers/axial_symmetric_sampler.h>

#include <crs_motion_planning/path_processing_utils.h>

#include <ur_ikfast_kinematics/descartes_ikfast_ur10e.h>

#include <Eigen/Eigen>
#include <vector>

namespace crs_motion_planning
{
struct descartesConfig
{
    using Ptr = std::shared_ptr<descartesConfig>;

    double axial_step = M_PI/36;
    double collision_safety_margin = 0.01;

    bool allow_collisions = false;
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

    Eigen::VectorXd surface_coeffs; // Defaults to 6 10s

    bool waypoints_critical = true;
};

struct pathPlanningConfig
{
    using Ptr = std::shared_ptr<pathPlanningConfig>;

    descartesConfig descartes_config;

//    tesseract_motion_planners::TrajOptPlannerDefaultConfig::Ptr trajopt_surface_config;
    trajoptSurfaceConfig trajopt_surface_config;

    std::shared_ptr<tesseract_motion_planners::OMPLFreespacePlannerConfig<ompl::geometric::RRTConnect>> ompl_config;

    tesseract_motion_planners::TrajOptPlannerFreespaceConfig::Ptr trajopt_freespace_config;

    tesseract::Tesseract::Ptr tesseract_local;

    descartes_light::KinematicsInterfaceD::Ptr kin_interface;

    std::vector<geometry_msgs::msg::PoseArray> rasters; // In world frame

    std::string manipulator = "manipulator";

    std::string world_frame = "world";
    std::string robot_base_frame = "base_link";
    std::string tool0_frame = "tool0";
    std::string tcp_frame;
//    std::string waypoint_frame = "world";

    Eigen::Isometry3d tool_offset = Eigen::Isometry3d::Identity();

    bool smooth_velocities = true;
    bool smooth_accelerations = true;
    bool smooth_jerks = true;

    bool add_approach_and_retreate = false;
    double approach_distance = 0.05;
    double retreat_distance = 0.05;

    bool required_joint_vel = false;
    double tool_speed = 0.03; // m/s
    double max_joint_vel = 0.2; // rad/s

    size_t minimum_raster_length = 2;
};

struct pathPlanningResults
{
    using Ptr = std::shared_ptr<pathPlanningResults>;

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

class crsMotionPlanner
{
public:
    crsMotionPlanner(pathPlanningConfig::Ptr config);
    ~crsMotionPlanner() = default;

    void updateConfiguration(pathPlanningConfig::Ptr config);

    ///
    /// \brief generateDescartesSeed Creates a seed trajectory using descartes
    /// \param waypoints
    /// \return success
    ///
    bool generateDescartesSeed(const geometry_msgs::msg::PoseArray &waypoints,
                               std::vector<std::size_t>& failed_edges,
                               std::vector<std::size_t>& failed_vertices,
                               trajectory_msgs::msg::JointTrajectory& joint_trajectory);

    ///
    /// \brief generateSurfacePlans Creates surface trajectories given a set of rasters
    /// \return success
    ///
    bool generateSurfacePlans(pathPlanningResults::Ptr& results);

    ///
    /// \brief generateSurfacePlans Creates surface trajectories given a set of rasters
    /// \return success
    ///
    bool generateFreespacePlans(pathPlanningResults::Ptr& results);

    ///
    /// \brief generateProcessPlan Creates process plans given a set of rasters, including freespace and surface trajectories
    /// \return success
    ///
    bool generateProcessPlan(pathPlanningResults::Ptr& results);

protected:
    pathPlanningConfig::Ptr config_;
    pathPlanningResults::Ptr results_;

};

}

#endif // CRS_MOTION_PLANNING_PATH_PLANNING_UTILS_H
