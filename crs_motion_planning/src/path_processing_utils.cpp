
static const double TRAJECTORY_TIME_TOLERANCE = 5.0;  // seconds
static const double WAIT_RESULT_TIMEOUT = 1.0;        // seconds

#include <crs_motion_planning/path_processing_utils.h>

bool crs_motion_planning::parsePathFromFile(const std::string& yaml_filepath,
                                            const std::string& waypoint_origin_frame,
                                            std::vector<geometry_msgs::msg::PoseArray>& raster_strips)
{
  std::vector<geometry_msgs::msg::PoseArray> temp_raster_strips;
  YAML::Node full_yaml_node = YAML::LoadFile(yaml_filepath);
  YAML::Node paths = full_yaml_node[0]["paths"];
  std::double_t offset_strip = 0.0;
  for (YAML::const_iterator path_it = paths.begin(); path_it != paths.end(); ++path_it)
  {
    std::vector<geometry_msgs::msg::PoseStamped> temp_poses;
    geometry_msgs::msg::PoseArray curr_pose_array;
    YAML::Node strip = (*path_it)["poses"];
    for (YAML::const_iterator pose_it = strip.begin(); pose_it != strip.end(); ++pose_it)
    {
      const YAML::Node& pose = *pose_it;
      try
      {
        geometry_msgs::msg::PoseStamped current_pose;

        float x = pose["position"]["x"].as<float>();
        float y = pose["position"]["y"].as<float>();
        float z = pose["position"]["z"].as<float>();

        float qx = pose["orientation"]["x"].as<float>();
        float qy = pose["orientation"]["y"].as<float>();
        float qz = pose["orientation"]["z"].as<float>();
        float qw = pose["orientation"]["w"].as<float>();

        current_pose.pose.position.x = x;
        current_pose.pose.position.y = y;
        current_pose.pose.position.z = z;

        current_pose.pose.orientation.x = qx;
        current_pose.pose.orientation.y = qy;
        current_pose.pose.orientation.z = qz;
        current_pose.pose.orientation.w = qw;

        current_pose.header.frame_id = waypoint_origin_frame;

        std::double_t offset_waypoint = offset_strip;

        Eigen::Isometry3d original_pose;
        tf2::fromMsg(current_pose.pose, original_pose);

        Eigen::Isometry3d offset_pose =
            original_pose * Eigen::Translation3d(0.0, 0.0, offset_waypoint) * Eigen::Quaterniond(0, 1, 0, 0);

        current_pose.pose = tf2::toMsg(offset_pose);
        curr_pose_array.poses.push_back(current_pose.pose);
        curr_pose_array.header = current_pose.header;

        temp_poses.push_back(tf2::toMsg(current_pose));
      }
      catch (YAML::InvalidNode& e)
      {
        continue;
      }
    }
    temp_raster_strips.push_back(curr_pose_array);
  }
  raster_strips.reserve(temp_raster_strips.size());
  raster_strips = temp_raster_strips;
  return true;
}

void crs_motion_planning::tesseractRosutilsToMsg(trajectory_msgs::msg::JointTrajectory& traj_msg,
                                                 const std::vector<std::string>& joint_names,
                                                 const Eigen::Ref<const tesseract_common::TrajArray>& traj)
{
  assert(joint_names.size() == static_cast<unsigned>(traj.cols()));

  // Initialze the whole traject with the current state.
  std::map<std::string, int> jn_to_index;
  traj_msg.joint_names.resize(joint_names.size());
  traj_msg.points.resize(static_cast<size_t>(traj.rows()));

  for (int i = 0; i < traj.rows(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint jtp;
    jtp.positions.resize(static_cast<size_t>(traj.cols()));

    for (int j = 0; j < traj.cols(); ++j)
    {
      if (i == 0)
        traj_msg.joint_names[static_cast<size_t>(j)] = joint_names[static_cast<size_t>(j)];

      jtp.positions[static_cast<size_t>(j)] = traj(i, j);
    }

    jtp.time_from_start = rclcpp::Duration(i, 0);
    traj_msg.points[static_cast<size_t>(i)] = jtp;
  }
}

void crs_motion_planning::rasterStripsToMarkerArray(const geometry_msgs::msg::PoseArray& strip,
                                                    const std::string& frame,
                                                    visualization_msgs::msg::MarkerArray& arrows,
                                                    std::vector<float> color,
                                                    double size)
{
  int arrow_count;
  if (arrows.markers.size() > 0)
  {
    arrow_count = static_cast<int>(arrows.markers.size()) - 1;
  }
  else
  {
    arrows.markers.clear();  // reset marker list
    arrow_count = 0;
  }

  for (auto pose : strip.poses)
  {
    Eigen::Affine3d arrow_pose;
    tf2::fromMsg(pose, arrow_pose);

    geometry_msgs::msg::Point p, p_tip;
    p.x = pose.position.x;
    p.y = pose.position.y;
    p.z = pose.position.z;

    Eigen::Vector3d tip = Eigen::Vector3d(p.x, p.y, p.z) + arrow_pose.linear() * Eigen::Vector3d(0, 0, size);
    p_tip = tf2::toMsg(tip);

    visualization_msgs::msg::Marker arrow;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.header.frame_id = frame;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.color.a = color[0];
    arrow.color.r = color[1];
    arrow.color.g = color[2];
    arrow.color.b = color[3];
    arrow.scale.x = 0.0025;
    arrow.scale.y = 0.005;
    arrow.scale.z = 0.005;
    arrow.id = arrow_count++;

    arrow.pose = tf2::toMsg(Eigen::Isometry3d::Identity());

    arrow.points.push_back(p);
    arrow.points.push_back(p_tip);
    arrows.markers.push_back(arrow);
  }
}

void crs_motion_planning::rasterStripsToMarkerArray(const std::vector<geometry_msgs::msg::PoseArray>& strips,
                                                    const std::string& frame,
                                                    visualization_msgs::msg::MarkerArray& arrows,
                                                    std::vector<float> color,
                                                    double size)
{
  std::vector<geometry_msgs::msg::PoseStamped> combined_strips;
  geometry_msgs::msg::PoseArray combined_pose_array;
  for (auto strip : strips)
  {
    combined_pose_array.poses.insert(combined_pose_array.poses.end(), strip.poses.begin(), strip.poses.end());
  }
  crs_motion_planning::rasterStripsToMarkerArray(combined_pose_array, frame, arrows, color, size);
}

void crs_motion_planning::failedEdgesToMarkerArray(const geometry_msgs::msg::PoseArray& vertices,
                                                   const std::string& frame,
                                                   visualization_msgs::msg::Marker& markers,
                                                   std::vector<float> color,
                                                   double size)
{
  markers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  markers.header.frame_id = frame;
  markers.action = visualization_msgs::msg::Marker::ADD;
  markers.color.a = color[0];
  markers.color.r = color[1];
  markers.color.g = color[2];
  markers.color.b = color[3];

  markers.scale.x = size;
  markers.scale.y = size;
  markers.scale.z = size;

  markers.id = 1001;
  markers.pose = tf2::toMsg(Eigen::Isometry3d::Identity());

  for (auto vertex : vertices.poses)
  {
    markers.points.push_back(vertex.position);
  }
}

void crs_motion_planning::cleanRasterStrip(const geometry_msgs::msg::PoseArray& original_strip,
                                           const std::vector<std::size_t>& failed_vertices,
                                           std::vector<geometry_msgs::msg::PoseArray>& fixed_strips,
                                           geometry_msgs::msg::PoseArray& failed_vertex_poses)
{
  geometry_msgs::msg::PoseArray curr_raster_pose_array;
  size_t j = 0;
  int min_len_raster = 0;
  std::vector<geometry_msgs::msg::PoseStamped> curr_raster;
  for (size_t i = 0; i < original_strip.poses.size(); ++i)
  {
    if (j <= failed_vertices.size() && i == failed_vertices[j])
    {
      if (curr_raster_pose_array.poses.size() > min_len_raster)
      {
        fixed_strips.push_back(curr_raster_pose_array);
      }
      j++;
      curr_raster_pose_array.poses.clear();
      failed_vertex_poses.poses.push_back(original_strip.poses[i]);
    }
    else
    {
      curr_raster_pose_array.poses.push_back(original_strip.poses[i]);
    }
  }
  if (curr_raster_pose_array.poses.size() > min_len_raster)
  {
    fixed_strips.push_back(curr_raster_pose_array);
  }
}

bool crs_motion_planning::splitRastersByJointDist(const trajectory_msgs::msg::JointTrajectory& given_traj,
                                                  const geometry_msgs::msg::PoseArray& given_raster,
                                                  const double& desired_ee_vel,
                                                  const double& max_joint_vel,
                                                  std::vector<trajectory_msgs::msg::JointTrajectory>& split_traj,
                                                  std::vector<geometry_msgs::msg::PoseArray>& split_rasters,
                                                  std::vector<std::vector<double>>& time_steps)
{
  bool split_exists = false;
  geometry_msgs::msg::PoseArray curr_raster_pose_array;
  std::vector<geometry_msgs::msg::PoseStamped> curr_raster;
  trajectory_msgs::msg::JointTrajectory curr_traj;
  std::vector<double> curr_time_steps;
  curr_traj.joint_names = given_traj.joint_names;
  curr_traj.points.push_back(given_traj.points[0]);
  curr_raster_pose_array.poses.push_back(given_raster.poses[0]);
  curr_time_steps.push_back(0);
  for (size_t i = 1; i < given_traj.points.size(); ++i)
  {
    // Find largest joint motion
    double max_joint_diff = 0;
    for (size_t j = 0; j < given_traj.points[i].positions.size(); ++j)
    {
      if (abs(given_traj.points[i].positions[j] - given_traj.points[i - 1].positions[j]) > max_joint_diff)
        max_joint_diff = abs(given_traj.points[i].positions[j] - given_traj.points[i - 1].positions[j]);
    }

    // Find cartesian distance traveled between points
    Eigen::Isometry3d curr_cart_pose, prev_cart_pose;
    Eigen::Vector3d cart_pose_diff;
    tf2::fromMsg(given_raster.poses[i], curr_cart_pose);
    tf2::fromMsg(given_raster.poses[i - 1], prev_cart_pose);
    cart_pose_diff = curr_cart_pose.translation() - prev_cart_pose.translation();
    double dist_traveled = cart_pose_diff.norm();

    // Calculate max required joint velocity
    double req_joint_vel = (desired_ee_vel * max_joint_diff) / dist_traveled;
    double curr_time_step = dist_traveled / desired_ee_vel;

    // If required joint velocity is higher than max allowable then split raster
    if (req_joint_vel > max_joint_vel)
    {
      split_exists = true;
      split_traj.push_back(curr_traj);
      curr_traj.points.clear();
      split_rasters.push_back(curr_raster_pose_array);
      curr_raster_pose_array.poses.clear();
      time_steps.push_back(curr_time_steps);
      curr_time_steps.clear();
    }
    // Add current point to running trajectory and raster list
    curr_traj.points.push_back(given_traj.points[i]);
    curr_raster_pose_array.poses.push_back(given_raster.poses[i]);
    curr_time_steps.push_back(curr_time_step);
  }
  split_traj.push_back(curr_traj);
  split_rasters.push_back(curr_raster_pose_array);
  time_steps.push_back(curr_time_steps);
  return split_exists;
}

void crs_motion_planning::addApproachAndRetreat(const geometry_msgs::msg::PoseArray& given_raster,
                                                const double& approach_dist,
                                                const double& retreat_dist,
                                                geometry_msgs::msg::PoseArray& returned_raster)
{
  Eigen::Isometry3d start_pose, end_pose, approach_pose, retreat_pose;
  tf2::fromMsg(given_raster.poses[0], start_pose);
  approach_pose = start_pose * Eigen::Translation3d(0.0, 0.0, -1 * approach_dist);
  tf2::fromMsg(given_raster.poses.back(), end_pose);
  retreat_pose = end_pose * Eigen::Translation3d(0.0, 0.0, -1 * retreat_dist);
  geometry_msgs::msg::Pose approach_pose_msg = tf2::toMsg(approach_pose);
  geometry_msgs::msg::Pose retreat_pose_msg = tf2::toMsg(retreat_pose);
  if (approach_dist != 0)
  {
    returned_raster.poses.push_back(approach_pose_msg);
  }
  returned_raster.poses.insert(returned_raster.poses.end(), given_raster.poses.begin(), given_raster.poses.end());
  if (retreat_dist != 0)
  {
    returned_raster.poses.push_back(retreat_pose_msg);
  }
}

bool crs_motion_planning::timeParameterizeTrajectories(const trajectory_msgs::msg::JointTrajectory& given_traj,
                                                       trajectory_msgs::msg::JointTrajectory& returned_traj,
                                                       const bool gazebo_time)
{
  double curr_time = 0, prev_time_diff = 0;
  size_t joint_num = given_traj.joint_names.size();
  std::vector<double> prev_pose(joint_num), prev_vel(joint_num), prev_accel(joint_num);
  returned_traj = given_traj;
  for (size_t i = 1; i < given_traj.points.size(); ++i)
  {
    if (gazebo_time)
    {
      curr_time = 0;
    }
    double time_diff = given_traj.points[i - 1].time_from_start.sec +
                       static_cast<double>(given_traj.points[i - 1].time_from_start.nanosec) / 1e9 - curr_time;
    for (size_t j = 0; j < joint_num; ++j)
    {
      double pose_diff = given_traj.points[i].positions[j] - given_traj.points[i - 1].positions[j];
      returned_traj.points[i - 1].velocities.push_back(pose_diff / time_diff);
    }
    if (i > 1)
    {
      for (size_t j = 0; j < joint_num; ++j)
      {
        double vel_diff = returned_traj.points[i - 1].velocities[j] - returned_traj.points[i - 2].velocities[j];
        returned_traj.points[i - 2].accelerations.push_back(vel_diff / prev_time_diff);
      }
    }
    curr_time += time_diff;
    prev_time_diff = time_diff;
  }
  std::vector<double> zeros_vec;
  for (int i = 0; i < 6; ++i)
    zeros_vec.push_back(0.0);
  returned_traj.points.back().velocities = zeros_vec;
  for (size_t j = 1; j < joint_num; ++j)
  {
    double vel_diff = returned_traj.points.rbegin()[0].velocities[j] - returned_traj.points.rbegin()[1].velocities[j];
    returned_traj.points.rbegin()[1].accelerations.push_back(vel_diff / prev_time_diff);
  }
  returned_traj.points.back().accelerations = zeros_vec;

  return true;
}

bool crs_motion_planning::execTrajectory(
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr ac,
    const rclcpp::Logger& logger,
    const trajectory_msgs::msg::JointTrajectory& traj)
{
  using namespace control_msgs::action;
  using namespace rclcpp_action;
  using GoalHandleT = Client<FollowJointTrajectory>::GoalHandle;

  rclcpp::Duration traj_dur(traj.points.back().time_from_start);
  bool res = false;
  std::string err_msg;

  auto print_traj_time = [&](const trajectory_msgs::msg::JointTrajectory& traj) {
    RCLCPP_ERROR(logger, "Trajectory with %lu points time data", traj.points.size());
    for (std::size_t i = 0; i < traj.points.size(); i++)
    {
      const auto& p = traj.points[i];
      RCLCPP_ERROR(logger, "\tPoint %lu : %f secs", rclcpp::Duration(p.time_from_start).seconds());
    }
  };

  FollowJointTrajectory::Goal goal;
  goal.trajectory = traj;
  auto goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  // TODO populate tolerances

  // send goal
  std::shared_future<GoalHandleT::SharedPtr> trajectory_exec_fut = ac->async_send_goal(goal);
  traj_dur = traj_dur + rclcpp::Duration(std::chrono::duration<double>(TRAJECTORY_TIME_TOLERANCE));

  // wait for goal acceptance
  std::future_status status = trajectory_exec_fut.wait_for(std::chrono::duration<double>(WAIT_RESULT_TIMEOUT));
  if (status != std::future_status::ready)
  {
    err_msg = "Action request was not accepted in time";
    RCLCPP_ERROR(logger, "%s", err_msg.c_str());
    ac->async_cancel_all_goals();
    return res;
  }

  auto gh = trajectory_exec_fut.get();
  if (!gh)
  {
    RCLCPP_ERROR(logger, "Goal was rejected by server");
    return res;
  }

  // getting result
  RCLCPP_INFO(logger, "Waiting %f seconds for goal", traj_dur.seconds());
  auto result_fut = ac->async_get_result(gh);
  status = result_fut.wait_for(traj_dur.to_chrono<std::chrono::seconds>());
  if (status != std::future_status::ready)
  {
    print_traj_time(traj);
    err_msg = "trajectory execution timed out";
    RCLCPP_ERROR(logger, "%s", err_msg.c_str());
    return res;
  }

  rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult wrapped_result = result_fut.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    err_msg = wrapped_result.result->error_string;
    RCLCPP_ERROR(logger, "Trajectory execution failed with error message: %s", err_msg.c_str());
    return res;
  }

  // reset future
  RCLCPP_INFO(logger, "Trajectory completed");
  return true;
}

bool crs_motion_planning::execSurfaceTrajectory(
    rclcpp_action::Client<crs_msgs::action::CartesianComplianceTrajectory>::SharedPtr ac,
    const rclcpp::Logger& logger,
    const trajectory_msgs::msg::JointTrajectory& traj)
{
  using namespace crs_msgs::action;
  using namespace rclcpp_action;
  using GoalHandleT = Client<CartesianComplianceTrajectory>::GoalHandle;

  rclcpp::Duration traj_dur(traj.points.back().time_from_start);
  bool res = false;
  std::string err_msg;

  auto print_traj_time = [&](const trajectory_msgs::msg::JointTrajectory& traj) {
    RCLCPP_ERROR(logger, "Trajectory with %lu points time data", traj.points.size());
    for (std::size_t i = 0; i < traj.points.size(); i++)
    {
      const auto& p = traj.points[i];
      RCLCPP_ERROR(logger, "\tPoint %lu : %f secs", rclcpp::Duration(p.time_from_start).seconds());
    }
  };

  CartesianComplianceTrajectory::Goal goal;
  goal.trajectory = traj;
  goal.force = 30;
  goal.tolerance = 0.2;
  goal.speed = 0.03;
  auto goal_options = rclcpp_action::Client<CartesianComplianceTrajectory>::SendGoalOptions();
  // TODO populate tolerances

  // send goal
  std::shared_future<GoalHandleT::SharedPtr> trajectory_exec_fut = ac->async_send_goal(goal);
  traj_dur = traj_dur + rclcpp::Duration(std::chrono::duration<double>(60));

  // wait for goal acceptance
  std::future_status status = trajectory_exec_fut.wait_for(std::chrono::duration<double>(WAIT_RESULT_TIMEOUT));
  if (status != std::future_status::ready)
  {
    err_msg = "Action request was not accepted in time";
    RCLCPP_ERROR(logger, "%s", err_msg.c_str());
    ac->async_cancel_all_goals();
    return res;
  }

  auto gh = trajectory_exec_fut.get();
  if (!gh)
  {
    RCLCPP_ERROR(logger, "Goal was rejected by server");
    return res;
  }

  // getting result
  RCLCPP_INFO(logger, "Waiting %f seconds for goal", traj_dur.seconds());
  auto result_fut = ac->async_get_result(gh);
  status = result_fut.wait_for(traj_dur.to_chrono<std::chrono::seconds>());
  if (status != std::future_status::ready)
  {
    print_traj_time(traj);
    err_msg = "trajectory execution timed out";
    RCLCPP_ERROR(logger, "%s", err_msg.c_str());
    return res;
  }

  rclcpp_action::ClientGoalHandle<CartesianComplianceTrajectory>::WrappedResult wrapped_result = result_fut.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    err_msg = wrapped_result.result->err_msg;
    RCLCPP_ERROR(logger, "Trajectory execution failed with error message: %s", err_msg.c_str());
    return res;
  }

  // reset future
  RCLCPP_INFO(logger, "Trajectory completed");
  return true;
}

bool crs_motion_planning::timeParameterizeFreespace(const trajectory_msgs::msg::JointTrajectory& given_traj,
                                                    const double& max_joint_vel,
                                                    const double& max_joint_acc,
                                                    trajectory_msgs::msg::JointTrajectory& returned_traj)
{
  returned_traj.joint_names = given_traj.joint_names;
  returned_traj.header = given_traj.header;

  if (given_traj.points.size() < 2)
  {
    returned_traj.points = given_traj.points;
    returned_traj.points.front().velocities =
        std::vector<std::double_t>(given_traj.points.front().positions.size(), 0.0);
    returned_traj.points.front().accelerations =
        std::vector<std::double_t>(given_traj.points.front().positions.size(), 0.0);
    return true;
  }

  std::vector<iterative_spline_parameterization::TrajectoryState> waypoints;
  for (auto point : given_traj.points)
  {
    Eigen::Matrix<double, 6, 1> joint_angles(point.positions.data());
    waypoints.push_back(iterative_spline_parameterization::TrajectoryState(
        joint_angles, Eigen::Matrix<double, 6, 1>::Zero(), Eigen::Matrix<double, 6, 1>::Zero(), 0.0));
  }

  bool add_points = false;
  if (waypoints.size() <= 3)
    add_points = true;

  iterative_spline_parameterization::IterativeSplineParameterization isp(add_points);
  isp.computeTimeStamps(waypoints, max_joint_vel, max_joint_acc);

  // Make sure not two adjacent points share the same timestep
  for (size_t i = 0; i < (waypoints.size() - 1); i++)
  {
    if ((waypoints[i + 1].time - waypoints[i].time) < 1e-8)
    {
      waypoints.erase(waypoints.begin() + i);
      i--;
    }
  }

  for (auto waypoint : waypoints)
  {
    trajectory_msgs::msg::JointTrajectoryPoint traj_point;
    traj_point.time_from_start = rclcpp::Duration(std::chrono::duration<double>(waypoint.time));

    traj_point.positions.resize(waypoint.positions.size());
    Eigen::VectorXd::Map(&traj_point.positions[0], waypoint.positions.size()) = waypoint.positions;

    traj_point.velocities.resize(waypoint.velocities.size());
    Eigen::VectorXd::Map(&traj_point.velocities[0], waypoint.velocities.size()) = waypoint.velocities;

    traj_point.accelerations.resize(waypoint.accelerations.size());
    Eigen::VectorXd::Map(&traj_point.accelerations[0], waypoint.accelerations.size()) = waypoint.accelerations;

    returned_traj.points.push_back(traj_point);
  }

  return true;
}

bool crs_motion_planning::timeParameterizeFreespace(
    const std::vector<trajectory_msgs::msg::JointTrajectory>& given_traj,
    const double& max_joint_vel,
    const double& max_joint_acc,
    std::vector<trajectory_msgs::msg::JointTrajectory>& returned_traj)
{
  for (auto traj : given_traj)
  {
    trajectory_msgs::msg::JointTrajectory curr_traj;
    crs_motion_planning::timeParameterizeFreespace(traj, max_joint_vel, max_joint_acc, curr_traj);
    returned_traj.push_back(curr_traj);
  }
  return true;
}
