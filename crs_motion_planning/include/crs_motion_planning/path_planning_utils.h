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
#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_rosutils/utils.h>

#include <descartes_light/descartes_light.h>
#include <descartes_samplers/evaluators/distance_edge_evaluator.h>
#include <descartes_samplers/evaluators/euclidean_distance_edge_evaluator.h>
#include <descartes_samplers/samplers/axial_symmetric_sampler.h>

#include <crs_motion_planning/path_processing_utils.h>

#include <Eigen/Eigen>
#include <vector>

namespace crs_motion_planning
{
struct pathPlanningConfig
{
    tesseract::Tesseract::Ptr tesseract_local_;

};

///
/// \brief generateDescartesSeed Creates a seed trajectory using descartes
/// \param tesseract_local
/// \param waypoints
/// \param world_to_robot_base
/// \param tool0_to_tip
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



}

#endif // CRS_MOTION_PLANNING_PATH_PLANNING_UTILS_H
