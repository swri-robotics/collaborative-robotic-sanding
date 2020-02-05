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

struct pathPlanningConfig
{
    using Ptr = std::shared_ptr<pathPlanningConfig>;

    descartesConfig descartes_config;

    tesseract_motion_planners::TrajOptPlannerDefaultConfig::Ptr trajopt_surface_config;

    std::shared_ptr<tesseract_motion_planners::OMPLFreespacePlannerConfig<ompl::geometric::RRTConnect>> ompl_config;

    tesseract_motion_planners::TrajOptPlannerFreespaceConfig::Ptr trajopt_freespace_config;

    tesseract::Tesseract::Ptr tesseract_local;

    descartes_light::KinematicsInterfaceD::Ptr kin_interface;

    std::string manipulator = "manipulator";

    std::string world_frame = "world";
    std::string robot_base_frame = "base_link";
    std::string tool0_frame = "tool0";
    std::string tcp_frame;

    Eigen::Isometry3d tool_offset;

    bool smooth_velocities = true;
    bool smooth_accelerations = true;
    bool smooth_jerks = true;
};

///
/// \brief generateDescartesSeed Creates a seed trajectory using descartes
/// \param kin
/// \param env
/// \param waypoints
/// \param kin_interface
/// \param axial_step
/// \param allow_collisions
/// \return success
///
bool generateDescartesSeed(const tesseract_kinematics::ForwardKinematics::ConstPtr kin,
                           const std::shared_ptr<const tesseract_environment::Environment> env,
                           const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
                           const descartes_light::KinematicsInterfaceD::Ptr &kin_interface,
                           const double &axial_step,
                           const bool &allow_collisions,
                           const double &collision_safety_margin,
                           std::vector<std::size_t>& failed_edges,
                           std::vector<std::size_t>& failed_vertices,
                           trajectory_msgs::msg::JointTrajectory& joint_trajectory);
///
/// \brief generateDescartesSeed Creates a seed trajectory using descartes
/// \param pathPlanningConfig
/// \param waypoints
/// \param axial_step
/// \param allow_collisions
/// \return success
///
bool generateDescartesSeed(const pathPlanningConfig config,
                           const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
                           const double &axial_step,
                           const bool &allow_collisions,
                           const double &collision_safety_margin,
                           std::vector<std::size_t>& failed_edges,
                           std::vector<std::size_t>& failed_vertices,
                           trajectory_msgs::msg::JointTrajectory& joint_trajectory);

class crsMotionPlanner
{
public:
    crsMotionPlanner(pathPlanningConfig::Ptr config);
    ~crsMotionPlanner() = default;

    void updateConfiguration(pathPlanningConfig::Ptr config);

    ///
    /// \brief generateDescartesSeed Creates a seed trajectory using descartes
    /// \param pathPlanningConfig
    /// \param waypoints
    /// \param axial_step
    /// \param allow_collisions
    /// \return success
    ///
    bool generateDescartesSeed(const geometry_msgs::msg::PoseArray &waypoints,
                               const double &axial_step,
                               const bool &allow_collisions,
                               const double &collision_safety_margin,
                               std::vector<std::size_t>& failed_edges,
                               std::vector<std::size_t>& failed_vertices,
                               trajectory_msgs::msg::JointTrajectory& joint_trajectory);
    bool generateDescartesSeed(const geometry_msgs::msg::PoseArray &waypoints,
                               std::vector<std::size_t>& failed_edges,
                               std::vector<std::size_t>& failed_vertices,
                               trajectory_msgs::msg::JointTrajectory& joint_trajectory);

    bool genSurfacePlans(const std::vector<geometry_msgs::msg::PoseArray> rasters);

protected:
    pathPlanningConfig::Ptr config_;
//    descartesConfig::Ptr descartes_config_;

};

}

#endif // CRS_MOTION_PLANNING_PATH_PLANNING_UTILS_H
