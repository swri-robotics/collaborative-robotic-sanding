#ifndef CRS_MOTION_PLANNING_PATH_PROCESSING_UTILS_H
#define CRS_MOTION_PLANNING_PATH_PROCESSING_UTILS_H

#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/iterator.h>
#include <yaml-cpp/node/impl.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_storage.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <crs_msgs/action/cartesian_compliance_trajectory.hpp>

#include <tesseract_common/types.h>

#include <Eigen/Eigen>
#include <vector>

#include <iterative_spline_parameterization/iterative_spline_parameterization.h>

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
                       std::vector<geometry_msgs::msg::PoseArray>& raster_strips);

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
void rasterStripsToMarkerArray(const geometry_msgs::msg::PoseArray& strips,
                               const std::string& frame,
                               visualization_msgs::msg::MarkerArray& arrows,
                               std::vector<float> color = { 1.0, 0.0, 1.0, 0.0 },
                               double size = -0.025);

///
/// \brief rasterStripsToMarkerArray Generates marker array from vector of poseStampeds
/// \param edges
/// \param frame
/// \param markers
/// \param color
/// \param size
///
void rasterStripsToMarkerArray(const std::vector<geometry_msgs::msg::PoseArray>& strips,
                               const std::string& frame,
                               visualization_msgs::msg::MarkerArray& arrows,
                               std::vector<float> color = { 1.0, 0.0, 1.0, 0.0 },
                               double size = -0.025);

/// \brief rasterStripsToMarkerArray Generates marker array from vector of poseStampeds
/// \param edges
/// \param frame
/// \param markers
/// \param color
/// \param size
///
void failedEdgesToMarkerArray(const geometry_msgs::msg::PoseArray& vertices,
                              const std::string& frame,
                              visualization_msgs::msg::Marker& markers,
                              std::vector<float> color = { 1.0, 1.0, 0.0, 0.0 },
                              double size = -0.025);

///
/// \brief cleanRasterStrip Removes unreachable points from raster strips
///
void cleanRasterStrip(const geometry_msgs::msg::PoseArray& original_strip,
                      const std::vector<std::size_t>& failed_vertices,
                      std::vector<geometry_msgs::msg::PoseArray>& fixed_strips,
                      geometry_msgs::msg::PoseArray& failed_vertex_poses);

bool splitRastersByJointDist(const trajectory_msgs::msg::JointTrajectory& given_traj,
                             const geometry_msgs::msg::PoseArray& given_raster,
                             const double& desired_ee_vel,
                             const double& max_joint_vel,
                             std::vector<trajectory_msgs::msg::JointTrajectory>& split_traj,
                             std::vector<geometry_msgs::msg::PoseArray>& split_rasters,
                             std::vector<std::vector<double>>& time_steps);

void addApproachAndRetreat(const geometry_msgs::msg::PoseArray& given_raster,
                           const double& approach_dist,
                           const double& retreat_dist,
                           geometry_msgs::msg::PoseArray& returned_raster);

bool timeParameterizeTrajectories(const trajectory_msgs::msg::JointTrajectory& given_traj,
                                  trajectory_msgs::msg::JointTrajectory& returned_traj,
                                  const bool gazebo_time = false);

bool execTrajectory(rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr ac,
                    const rclcpp::Logger& logger,
                    const trajectory_msgs::msg::JointTrajectory& traj);

bool execSurfaceTrajectory(rclcpp_action::Client<crs_msgs::action::CartesianComplianceTrajectory>::SharedPtr ac,
                           const rclcpp::Logger& logger,
                           const trajectory_msgs::msg::JointTrajectory& traj);

bool timeParameterizeFreespace(const trajectory_msgs::msg::JointTrajectory& given_traj,
                               const double& max_joint_vel,
                               const double& max_joint_acc,
                               trajectory_msgs::msg::JointTrajectory& returned_traj);

bool timeParameterizeFreespace(const std::vector<trajectory_msgs::msg::JointTrajectory>& given_traj,
                               const double& max_joint_vel,
                               const double& max_joint_acc,
                               std::vector<trajectory_msgs::msg::JointTrajectory>& returned_traj);

}  // namespace crs_motion_planning

#endif  // CRS_MOTION_PLANNING_PATH_PROCESSING_UTILS_H
