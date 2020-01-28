#ifndef CRS_MOTION_PLANNING_PATH_PROCESSING_UTILS_H
#define CRS_MOTION_PLANNING_PATH_PROCESSING_UTILS_H

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/iterator.h>
#include <yaml-cpp/node/impl.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_storage.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

#include <tesseract_common/types.h>

#include <Eigen/Eigen>
#include <vector>

namespace crs_motion_planning
{
///
/// \brief parsePathFromFile Creates a collection of raster strips from a yaml file
/// \param yaml_filepath
/// \param waypoint_origin_frame
/// \return success
///
bool parsePathFromFile(const std::string& yaml_filepath,
                       const std::string& waypoint_origin_frame,
                       std::vector<std::vector<geometry_msgs::msg::PoseStamped>> &raster_strips);

///
/// \brief tesseract_rosutils_toMsg converts tesserarct TrajArray to trajectory msg
/// \param traj_msg
/// \param joint_names
/// \param traj
///
void tesseractRosutilsToMsg(trajectory_msgs::msg::JointTrajectory& traj_msg,
                              const std::vector<std::string>& joint_names,
                              const Eigen::Ref<const tesseract_common::TrajArray>& traj);

///
/// \brief rasterStripsToMarkerArray Generates marker array from vector of poseStampeds
/// \param edges
/// \param frame
/// \param markers
/// \param color
/// \param size
///
void rasterStripsToMarkerArray(const std::vector<geometry_msgs::msg::PoseStamped>& strips,
                               const std::string& frame,
                               visualization_msgs::msg::MarkerArray &arrows,
                               std::vector<float> color = {1.0, 0.0, 1.0, 0.0},
                               double size = -0.025);

///
/// \brief rasterStripsToMarkerArray Generates marker array from vector of poseStampeds
/// \param edges
/// \param frame
/// \param markers
/// \param color
/// \param size
///
void rasterStripsToMarkerArray(const std::vector<std::vector<geometry_msgs::msg::PoseStamped>>& strips,
                               const std::string& frame,
                               visualization_msgs::msg::MarkerArray &arrows,
                               std::vector<float> color = {1.0, 0.0, 1.0, 0.0},
                               double size = -0.025);///

/// \brief rasterStripsToMarkerArray Generates marker array from vector of poseStampeds
/// \param edges
/// \param frame
/// \param markers
/// \param color
/// \param size
///
void failedEdgesToMarkerArray(const std::vector<geometry_msgs::msg::PoseStamped>& vertices,
                               const std::string& frame,
                               visualization_msgs::msg::Marker &markers,
                               std::vector<float> color = {1.0, 1.0, 0.0, 0.0},
                               double size = -0.025);

///
/// \brief cleanRasterStrip Removes unreachable points from raster strips
///
void cleanRasterStrip(const std::vector<geometry_msgs::msg::PoseStamped>& original_strip,
                      const std::vector<std::size_t>& failed_vertices,
                      std::vector<std::vector<geometry_msgs::msg::PoseStamped>> &fixed_strips,
                      std::vector<geometry_msgs::msg::PoseStamped> &failed_vertex_poses);

void splitRastersByJointDist(const trajectory_msgs::msg::JointTrajectory& given_traj,
                             const std::vector<geometry_msgs::msg::PoseStamped>& given_raster,
                             const double& desired_ee_vel,
                             const double& max_joint_vel,
                             std::vector<trajectory_msgs::msg::JointTrajectory> &split_traj,
                             std::vector<std::vector<geometry_msgs::msg::PoseStamped>> &split_rasters,
                             std::vector<std::vector<double>> &time_steps);
}

#endif // CRS_MOTION_PLANNING_PATH_PROCESSING_UTILS_H
