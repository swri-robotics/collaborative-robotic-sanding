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
#include <cartesian_trajectory_msgs/action/cartesian_compliance_trajectory.hpp>

#include <Eigen/Eigen>
#include <vector>

#include <iterative_spline_parameterization/iterative_spline_parameterization.h>

#include <tesseract/tesseract.h>

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

/**
 * @brief creates a marker from a mesh file
 * @param file_path The path to the file
 * @param ns        The marker namespace
 * @param frame_id  The frame id
 * @param color     The rgba color values
 * @return  A marker
 */
visualization_msgs::msg::Marker meshToMarker(const std::string& file_path,
                                             const std::string& ns,
                                             const std::string& frame_id,
                                             const std::array<float, 4> color = { 1.0, 0.8, 1.0, 1.0 });

/**
 * @brief Creates axis markers for each waypoint in the paths
 * @param path
 * @param frame_id
 * @param ns
 * @param start_id
 * @param axis_scale
 * @param axis_length
 * @param offset
 * @return  A marker
 */
visualization_msgs::msg::MarkerArray
convertToAxisMarkers(const std::vector<geometry_msgs::msg::PoseArray>& path,
                     const std::string& frame_id,
                     const std::string& ns,
                     const std::size_t& start_id = 1,
                     const double& axis_scale = 0.001,
                     const double& axis_length = 0.03,
                     const std::array<float, 6>& offset = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

/**
 * @brief creates lines that connect the waypoints
 * @param path
 * @param frame_id
 * @param ns
 * @param start_id
 * @param offset
 * @param line_width
 * @param point_size
 * @return A marker
 */
visualization_msgs::msg::MarkerArray convertToDottedLineMarker(
    const std::vector<geometry_msgs::msg::PoseArray>& path,
    const std::string& frame_id,
    const std::string& ns,
    const std::size_t& start_id = 1,
    const std::array<float, 6>& offset = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
    const float& line_width = 0.001,
    const float& point_size = 0.005,
    const std::tuple<float, float, float, float>& line_rgba = std::make_tuple(1.0, 1.0, 0.2, 1.0),
    const std::tuple<float, float, float, float>& point_rgba = std::make_tuple(0.1, .8, 0.2, 1.0));

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
                             const double& max_dist,
                             std::vector<trajectory_msgs::msg::JointTrajectory>& split_traj,
                             std::vector<geometry_msgs::msg::PoseArray>& split_rasters,
                             std::vector<std::vector<double>>& time_steps,
                             const double& max_rotation_rate = 3.0,
                             const double& joint_vel_mult = 1.0);

void addApproachAndRetreat(const geometry_msgs::msg::PoseArray& given_raster,
                           const double& approach_dist,
                           const double& retreat_dist,
                           geometry_msgs::msg::PoseArray& returned_raster);

bool timeParameterizeTrajectories(const trajectory_msgs::msg::JointTrajectory& given_traj,
                                  trajectory_msgs::msg::JointTrajectory& returned_traj,
                                  const bool gazebo_time = false);

bool execTrajectory(rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr ac,
                    const rclcpp::Logger& logger,
                    const trajectory_msgs::msg::JointTrajectory& traj,
                    rclcpp::Node::SharedPtr node = nullptr);

bool timeParameterizeFreespace(const trajectory_msgs::msg::JointTrajectory& given_traj,
                               const double& max_joint_vel,
                               const double& max_joint_acc,
                               trajectory_msgs::msg::JointTrajectory& returned_traj);

bool timeParameterizeFreespace(const std::vector<trajectory_msgs::msg::JointTrajectory>& given_traj,
                               const double& max_joint_vel,
                               const double& max_joint_acc,
                               std::vector<trajectory_msgs::msg::JointTrajectory>& returned_traj);

struct cartesianTrajectoryConfig
{
  using Ptr = std::shared_ptr<cartesianTrajectoryConfig>;

  tesseract::Tesseract::Ptr tesseract_local;
  std::string manipulator = "manipulator";

  std::string base_frame = "base_link";
  std::string tool_frame = "tool0";
  std::string tcp_frame = "sander_center_link";

  double target_force = 20;
  double target_speed = 0.05;

  geometry_msgs::msg::Vector3 path_pose_tolerance;
  geometry_msgs::msg::Vector3 path_ori_tolerance;

  geometry_msgs::msg::Vector3 goal_pose_tolerance;
  geometry_msgs::msg::Vector3 goal_ori_tolerance;

  geometry_msgs::msg::Vector3 force_tolerance;
};

void findCartPoseArrayFromTraj(const trajectory_msgs::msg::JointTrajectory& joint_trajectory,
                               const cartesianTrajectoryConfig traj_config,
                               geometry_msgs::msg::PoseArray& cartesian_poses);

void genCartesianTrajectory(const trajectory_msgs::msg::JointTrajectory& joint_trajectory,
                            const cartesianTrajectoryConfig traj_config,
                            cartesian_trajectory_msgs::msg::CartesianTrajectory& cartesian_trajectory);

void genCartesianTrajectoryGoal(
    const cartesian_trajectory_msgs::msg::CartesianTrajectory& cartesian_trajectory,
    const cartesianTrajectoryConfig traj_config,
    cartesian_trajectory_msgs::action::CartesianComplianceTrajectory::Goal& cartesian_trajectory_goal);

bool execSurfaceTrajectory(
    rclcpp_action::Client<cartesian_trajectory_msgs::action::CartesianComplianceTrajectory>::SharedPtr ac,
    const rclcpp::Logger& logger,
    const trajectory_msgs::msg::JointTrajectory& traj,
    const cartesianTrajectoryConfig& traj_config);

bool execSurfaceTrajectory(
    rclcpp_action::Client<cartesian_trajectory_msgs::action::CartesianComplianceTrajectory>::SharedPtr ac,
    const rclcpp::Logger& logger,
    const cartesian_trajectory_msgs::msg::CartesianTrajectory& traj,
    const cartesianTrajectoryConfig& traj_config);

///
/// \brief removeEdgeWaypoints remove buffer from either side of the waypoints
/// \return cropped waypoints
///
geometry_msgs::msg::PoseArray removeEdgeWaypoints(const geometry_msgs::msg::PoseArray& input_waypoints,
                                                  const double& buffer);

///
/// \brief removeEdgeWaypoints remove buffer from either side of the waypoints
/// \return cropped waypoints
///
std::vector<geometry_msgs::msg::PoseArray>
removeEdgeWaypoints(const std::vector<geometry_msgs::msg::PoseArray>& input_waypoints, const double& buffer);

///
/// \brief transformWaypoints Transform waypoints given transform
/// \return transformed waypoints
///
geometry_msgs::msg::PoseArray transformWaypoints(const geometry_msgs::msg::PoseArray& input_waypoints,
                                                 const geometry_msgs::msg::TransformStamped& transform,
                                                 const bool inverse = false);

///
/// \brief transformWaypoints Transform waypoints given transform
/// \return transformed waypoints
///
std::vector<geometry_msgs::msg::PoseArray>
transformWaypoints(const std::vector<geometry_msgs::msg::PoseArray>& input_waypoints,
                   const geometry_msgs::msg::TransformStamped& transform,
                   const bool inverse = false);

///
/// \brief filterReachabilitySphere Removes any points outside of sphere of reachability from robot base
/// \return success
///
bool filterReachabilitySphere(const geometry_msgs::msg::PoseArray& waypoints,
                              const double& radius,
                              geometry_msgs::msg::PoseArray& reachable_waypoints);

///
/// \brief filterReachabilitySphere Removes any points outside of sphere of reachability from robot base
/// \return success
///
bool filterReachabilitySphere(const std::vector<geometry_msgs::msg::PoseArray>& waypoints_vec,
                              const double& radius,
                              std::vector<geometry_msgs::msg::PoseArray>& reachable_waypoints_vec);

///
/// \brief filterSingularityCylinder Removes any points inside cylinder coming from UR base
/// \return filtered points
///
geometry_msgs::msg::PoseArray filterSingularityCylinder(const geometry_msgs::msg::PoseArray& waypoints,
                                                        const double& radius);

///
/// \brief filterSingularityCylinder Removes any points inside cylinder coming from UR base
/// \return filtered points
///
std::vector<geometry_msgs::msg::PoseArray>
filterSingularityCylinder(const std::vector<geometry_msgs::msg::PoseArray>& waypoints, const double& radius);

///
/// \brief calcPoseDist calculates distance between two geometry_msgs Pose msgs
/// \return distance between poses
///
double calcPoseDist(const geometry_msgs::msg::Pose& waypoint1, const geometry_msgs::msg::Pose& waypoint2);

///
/// \brief calcRotation calculates rotation between two geometry_msgs Pose msgs
/// \return rotation between poses
///
double calcRotation(const geometry_msgs::msg::Pose& waypoint1, const geometry_msgs::msg::Pose& waypoint2);

///
/// \brief organizeRasters Puts rasters in alternating order
/// \return organized waypoints
///
std::vector<geometry_msgs::msg::PoseArray>
organizeRasters(const std::vector<geometry_msgs::msg::PoseArray>& waypoints_vec);

///
/// \brief findRasterRotation finds the rotation rate of a raster
/// \return [max rotation in radians/m, avg rotation in radians/m]
///
std::vector<double> findRasterRotation(const geometry_msgs::msg::PoseArray& waypoints);

}  // namespace crs_motion_planning

#endif  // CRS_MOTION_PLANNING_PATH_PROCESSING_UTILS_H
