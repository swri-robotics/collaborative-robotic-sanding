#include <rclcpp/logging.hpp>
#include <boost/format.hpp>
#include <crs_motion_planning/path_processing_utils.h>

static const double TRAJECTORY_TIME_TOLERANCE = 180.0;  // seconds
static const double WAIT_RESULT_TIMEOUT = 5.0;          // seconds
static const rclcpp::Logger LOGGER = rclcpp::get_logger("PATH_PROCESSING_UTILS");

static geometry_msgs::msg::Pose pose3DtoPoseMsg(const std::array<float, 6>& p)
{
  using namespace Eigen;
  geometry_msgs::msg::Pose pose_msg;
  Eigen::Affine3d eigen_pose = Translation3d(Vector3d(std::get<0>(p), std::get<1>(p), std::get<2>(p))) *
                               AngleAxisd(std::get<3>(p), Vector3d::UnitX()) *
                               AngleAxisd(std::get<4>(p), Vector3d::UnitY()) *
                               AngleAxisd(std::get<5>(p), Vector3d::UnitZ());

  pose_msg = tf2::toMsg(eigen_pose);
  return std::move(pose_msg);
}

bool crs_motion_planning::parsePathFromFile(const std::string& yaml_filepath,
                                            const std::string& waypoint_origin_frame,
                                            std::vector<geometry_msgs::msg::PoseArray>& raster_strips)
{
  std::vector<geometry_msgs::msg::PoseArray> temp_raster_strips;
  YAML::Node full_yaml_node = YAML::LoadFile(yaml_filepath);
  if (!full_yaml_node)
  {
    RCLCPP_ERROR(LOGGER, "Failed to load into YAML from file %s", yaml_filepath.c_str());
    return false;
  }

  if (full_yaml_node.Type() != YAML::NodeType::Sequence)
  {
    RCLCPP_WARN(
        LOGGER, "Top level YAML element is not a sequence but a %i type", static_cast<int>(full_yaml_node.Type()));
    return false;
  }

  YAML::Node paths = full_yaml_node[0]["paths"];
  if (!paths || paths.Type() != YAML::NodeType::Sequence)
  {
    RCLCPP_ERROR(LOGGER, "The 'path' YAML element was not found or is not a sequence");
    return false;
  }

  std::double_t offset_strip = 0.0;
  Eigen::Vector3f prev_end = Eigen::Vector3f::Zero();
  for (YAML::const_iterator path_it = paths.begin(); path_it != paths.end(); ++path_it)
  {
    std::vector<geometry_msgs::msg::PoseStamped> temp_poses;
    geometry_msgs::msg::PoseArray curr_pose_array;
    YAML::Node strip = (*path_it)["poses"];
    if (!strip)
    {
      RCLCPP_ERROR(LOGGER, "The 'poses' YAML element was not found");
      return false;
    }
    size_t count = 0;
    YAML::Node first_pose = strip[0], last_pose = strip[strip.size() - 1];
    Eigen::Vector3f first_xyz, last_xyz;
    try
    {
      first_xyz = Eigen::Vector3f(first_pose["position"]["x"].as<float>(),
                                  first_pose["position"]["y"].as<float>(),
                                  first_pose["position"]["z"].as<float>());
      last_xyz = Eigen::Vector3f(last_pose["position"]["x"].as<float>(),
                                 last_pose["position"]["y"].as<float>(),
                                 last_pose["position"]["z"].as<float>());
      if ((first_xyz - prev_end).norm() < (last_xyz - prev_end).norm())
      {
        prev_end = last_xyz;
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
      }
      else
      {
        prev_end = first_xyz;
        for (int i = strip.size() - 1; i >= 0; --i)
        {
          const YAML::Node& pose = strip[i];
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
      }
    }
    catch (YAML::InvalidNode& e)
    {
      continue;
    }

    temp_raster_strips.push_back(curr_pose_array);
  }
  raster_strips.reserve(temp_raster_strips.size());
  raster_strips = temp_raster_strips;
  return true;
}

visualization_msgs::msg::Marker crs_motion_planning::meshToMarker(const std::string& file_path,
                                                                  const std::string& ns,
                                                                  const std::string& frame_id,
                                                                  const std::array<float, 4> color)
{
  visualization_msgs::msg::Marker m;
  m.ns = ns;
  m.header.frame_id = frame_id;
  m.type = m.MESH_RESOURCE;
  m.action = m.ADD;
  m.pose = tf2::toMsg(Eigen::Isometry3d::Identity());
  m.lifetime = rclcpp::Duration(0);
  m.mesh_resource = "file://" + file_path;
  std::tie(m.scale.x, m.scale.y, m.scale.z) = std::make_tuple(1.0, 1.0, 1.0);
  std::tie(m.color.r, m.color.g, m.color.b, m.color.a) = std::make_tuple(color[0], color[1], color[2], color[3]);
  return m;
}

visualization_msgs::msg::MarkerArray
crs_motion_planning::convertToAxisMarkers(const std::vector<geometry_msgs::msg::PoseArray>& path,
                                          const std::string& frame_id,
                                          const std::string& ns,
                                          const std::size_t& start_id,
                                          const double& axis_scale,
                                          const double& axis_length,
                                          const std::array<float, 6>& offset)
{
  using namespace Eigen;

  visualization_msgs::msg::MarkerArray markers;

  auto create_line_marker = [&](const int id,
                                const std::tuple<float, float, float, float>& rgba) -> visualization_msgs::msg::Marker {
    visualization_msgs::msg::Marker line_marker;
    line_marker.action = line_marker.ADD;
    std::tie(line_marker.color.r, line_marker.color.g, line_marker.color.b, line_marker.color.a) = rgba;
    line_marker.header.frame_id = frame_id;
    line_marker.type = line_marker.LINE_LIST;
    line_marker.id = id;
    line_marker.lifetime = rclcpp::Duration(0);
    line_marker.ns = ns;
    std::tie(line_marker.scale.x, line_marker.scale.y, line_marker.scale.z) = std::make_tuple(axis_scale, 0.0, 0.0);
    line_marker.pose = pose3DtoPoseMsg(offset);
    return std::move(line_marker);
  };

  // markers for each axis line
  int marker_id = start_id;
  visualization_msgs::msg::Marker x_axis_marker = create_line_marker(++marker_id, std::make_tuple(1.0, 0.0, 0.0, 1.0));
  visualization_msgs::msg::Marker y_axis_marker = create_line_marker(++marker_id, std::make_tuple(0.0, 1.0, 0.0, 1.0));
  visualization_msgs::msg::Marker z_axis_marker = create_line_marker(++marker_id, std::make_tuple(0.0, 0.0, 1.0, 1.0));

  auto add_axis_line = [](const Isometry3d& eigen_pose,
                          const Vector3d& dir,
                          const geometry_msgs::msg::Point& p1,
                          visualization_msgs::msg::Marker& marker) {
    geometry_msgs::msg::Point p2;
    Eigen::Vector3d line_endpoint;

    // axis endpoint
    line_endpoint = eigen_pose * dir;
    std::tie(p2.x, p2.y, p2.z) = std::make_tuple(line_endpoint.x(), line_endpoint.y(), line_endpoint.z());

    // adding line
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  };

  for (auto& poses : path)
  {
    for (auto& pose : poses.poses)
    {
      Eigen::Isometry3d eigen_pose;
      tf2::fromMsg(pose, eigen_pose);

      geometry_msgs::msg::Point p1;
      std::tie(p1.x, p1.y, p1.z) = std::make_tuple(pose.position.x, pose.position.y, pose.position.z);

      add_axis_line(eigen_pose, Vector3d::UnitX() * axis_length, p1, x_axis_marker);
      add_axis_line(eigen_pose, Vector3d::UnitY() * axis_length, p1, y_axis_marker);
      add_axis_line(eigen_pose, Vector3d::UnitZ() * axis_length, p1, z_axis_marker);
    }
  }

  markers.markers.push_back(x_axis_marker);
  markers.markers.push_back(y_axis_marker);
  markers.markers.push_back(z_axis_marker);
  return std::move(markers);
}

visualization_msgs::msg::MarkerArray
crs_motion_planning::convertToDottedLineMarker(const std::vector<geometry_msgs::msg::PoseArray>& path,
                                               const std::string& frame_id,
                                               const std::string& ns,
                                               const std::size_t& start_id,
                                               const std::array<float, 6>& offset,
                                               const float& line_width,
                                               const float& point_size,
                                               const std::tuple<float, float, float, float>& line_rgba,
                                               const std::tuple<float, float, float, float>& point_rgba)
{
  visualization_msgs::msg::MarkerArray markers_msgs;
  visualization_msgs::msg::Marker line_marker, points_marker;

  // configure line marker
  line_marker.action = line_marker.ADD;
  std::tie(line_marker.color.r, line_marker.color.g, line_marker.color.b, line_marker.color.a) = line_rgba;
  line_marker.header.frame_id = frame_id;
  line_marker.type = line_marker.LINE_STRIP;
  line_marker.id = start_id;
  line_marker.lifetime = rclcpp::Duration(0);
  line_marker.ns = ns;
  std::tie(line_marker.scale.x, line_marker.scale.y, line_marker.scale.z) = std::make_tuple(line_width, 0.0, 0.0);
  line_marker.pose = pose3DtoPoseMsg(offset);

  // configure point marker
  points_marker = line_marker;
  points_marker.type = points_marker.POINTS;
  points_marker.ns = ns;
  std::tie(points_marker.color.r, points_marker.color.g, points_marker.color.b, points_marker.color.a) = point_rgba;
  std::tie(points_marker.scale.x, points_marker.scale.y, points_marker.scale.z) =
      std::make_tuple(point_size, point_size, point_size);

  int id_counter = start_id;
  for (auto& poses : path)
  {
    line_marker.points.clear();
    points_marker.points.clear();
    line_marker.points.reserve(poses.poses.size());
    points_marker.points.reserve(poses.poses.size());
    for (auto& pose : poses.poses)
    {
      geometry_msgs::msg::Point p;
      std::tie(p.x, p.y, p.z) = std::make_tuple(pose.position.x, pose.position.y, pose.position.z);
      line_marker.points.push_back(p);
      points_marker.points.push_back(p);
    }

    line_marker.id = (++id_counter);
    points_marker.id = (++id_counter);
    markers_msgs.markers.push_back(line_marker);
    markers_msgs.markers.push_back(points_marker);
  }

  return markers_msgs;
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
                                                  const double& max_dist,
                                                  std::vector<trajectory_msgs::msg::JointTrajectory>& split_traj,
                                                  std::vector<geometry_msgs::msg::PoseArray>& split_rasters,
                                                  std::vector<std::vector<double>>& time_steps,
                                                  const double& max_rotation_rate,
                                                  const double& joint_vel_mult)
{
  double max_vel_adj = max_joint_vel * joint_vel_mult;
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

    // Calculate rotation rate in rad/m between adjacent points
    double rotation_between = crs_motion_planning::calcRotation(given_raster.poses[i], given_raster.poses[i - 1]);
    double rotation_rate = rotation_between / dist_traveled;

    // Calculate max required joint velocity
    double req_joint_vel = (desired_ee_vel * max_joint_diff) / dist_traveled;
    double curr_time_step = dist_traveled / desired_ee_vel;

    // If required joint velocity is higher than max allowable then split raster
    if (req_joint_vel > max_vel_adj || dist_traveled > max_dist || rotation_rate > max_rotation_rate)
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
    const trajectory_msgs::msg::JointTrajectory& traj,
    rclcpp::Node::SharedPtr node)
{
  using namespace control_msgs::action;

  auto print_time = [&](const trajectory_msgs::msg::JointTrajectory& traj) {
    for (std::size_t i = 0; i < traj.points.size(); i++)
    {
      auto& p = traj.points[i];
      rclcpp::Duration dur(p.time_from_start);
      RCLCPP_DEBUG(logger, "Point %lu time: %f", i, dur.seconds());
    }
  };
  print_time(traj);

  rclcpp::Duration traj_dur(traj.points.back().time_from_start);
  bool res = false;

  FollowJointTrajectory::Goal goal;
  goal.trajectory = traj;
  auto goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  std::promise<bool> result_promise;
  std::future<bool> result_fut = result_promise.get_future();
  goal_options.goal_response_callback = [&](std::shared_future<GoalHandle::SharedPtr> future) {
    if (!future.get())
    {
      std::string err_msg = "Failed to accept goal";
      RCLCPP_ERROR(logger, err_msg.c_str());
      ac->async_cancel_all_goals();
      result_promise.set_value(false);
    }
  };

  goal_options.result_callback = [&](const GoalHandle::WrappedResult& result) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      std::string err_msg = result.result->error_string;
      RCLCPP_ERROR(logger, "Trajectory execution failed with error message: %s", err_msg.c_str());
      result_promise.set_value(false);
      return;
    }
    result_promise.set_value(true);
  };

  // send goal
  auto trajectory_exec_fut_ = ac->async_send_goal(goal, goal_options);

  // spinning
  std::atomic<bool> done;
  done = false;
  std::future<bool> spinner_fut;
  if (node)
  {
    spinner_fut = std::async([&]() -> bool {
      while (!done)
      {
        rclcpp::spin_some(node);
      }
      return true;
    });
  }

  traj_dur = traj_dur + rclcpp::Duration(std::chrono::duration<double>(TRAJECTORY_TIME_TOLERANCE));
  RCLCPP_INFO(logger, "Waiting %f seconds for goal", traj_dur.seconds());
  std::future_status status = result_fut.wait_for(traj_dur.to_chrono<std::chrono::seconds>());
  done = true;
  if (status != std::future_status::ready)
  {
    std::string err_msg =
        boost::str(boost::format("Trajectory execution timed out with flag %i") % static_cast<int>(status));
    RCLCPP_ERROR(logger, err_msg.c_str());
    ac->async_cancel_all_goals();
    return res;
  }
  RCLCPP_INFO(logger, "Finished trajectory");
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

void crs_motion_planning::findCartPoseArrayFromTraj(const trajectory_msgs::msg::JointTrajectory& joint_trajectory,
                                                    const cartesianTrajectoryConfig traj_config,
                                                    geometry_msgs::msg::PoseArray& cartesian_poses)
{
  const std::shared_ptr<const tesseract_environment::Environment> env =
      traj_config.tesseract_local->getEnvironmentConst();
  tesseract_common::TransformMap curr_transforms = env->getCurrentState()->link_transforms;

  tesseract_kinematics::ForwardKinematics::ConstPtr kin =
      traj_config.tesseract_local->getFwdKinematicsManagerConst()->getFwdKinematicSolver(traj_config.manipulator);

  Eigen::Isometry3d world_to_base_link, world_to_sander, world_to_tool0, tool0_to_sander;
  world_to_base_link = curr_transforms.find(traj_config.base_frame)->second;
  world_to_sander = curr_transforms.find(traj_config.tcp_frame)->second;
  world_to_tool0 = curr_transforms.find(traj_config.tool_frame)->second;
  tool0_to_sander = world_to_tool0.inverse() * world_to_sander;

  for (auto joint_pose : joint_trajectory.points)
  {
    Eigen::VectorXd joint_positions(joint_pose.positions.size());
    for (size_t i = 0; i < joint_pose.positions.size(); i++)
    {
      joint_positions[static_cast<int>(i)] = joint_pose.positions[i];
    }
    Eigen::Isometry3d eig_pose;
    kin->calcFwdKin(eig_pose, joint_positions);
    Eigen::Isometry3d world_to_base_link = Eigen::Isometry3d::Identity();
    eig_pose = world_to_base_link * eig_pose * Eigen::Quaterniond(-0.5, 0.5, -0.5, 0.5) *
               tool0_to_sander;  // Fwd kin switches to x forward instead of z
    geometry_msgs::msg::Pose curr_cart_pose = tf2::toMsg(eig_pose);
    cartesian_poses.poses.push_back(curr_cart_pose);
  }
  cartesian_poses.header.frame_id = traj_config.base_frame;
}

void crs_motion_planning::genCartesianTrajectory(
    const trajectory_msgs::msg::JointTrajectory& joint_trajectory,
    const crs_motion_planning::cartesianTrajectoryConfig traj_config,
    cartesian_trajectory_msgs::msg::CartesianTrajectory& cartesian_trajectory)
{
  geometry_msgs::msg::PoseArray cartesian_poses;
  crs_motion_planning::findCartPoseArrayFromTraj(joint_trajectory, traj_config, cartesian_poses);

  Eigen::Vector3d tcp_force_vec = Eigen::Vector3d::Zero();
  tcp_force_vec.z() = traj_config.target_force;
  geometry_msgs::msg::Vector3 target_wrench_force, target_wrench_torque;
  target_wrench_force = tf2::toMsg(tcp_force_vec, target_wrench_force);
  target_wrench_torque = tf2::toMsg(Eigen::Vector3d::Zero(), target_wrench_torque);
  geometry_msgs::msg::Wrench target_wrench;
  target_wrench.force = target_wrench_force;
  target_wrench.torque = target_wrench_torque;

  cartesian_trajectory.header.frame_id = traj_config.base_frame;
  cartesian_trajectory.tcp_frame = traj_config.tcp_frame;
  for (size_t i = 0; i < cartesian_poses.poses.size(); ++i)
  {
    geometry_msgs::msg::Pose pose = cartesian_poses.poses[i];
    cartesian_trajectory_msgs::msg::CartesianTrajectoryPoint curr_point;
    curr_point.pose = pose;
    curr_point.wrench = target_wrench;
    curr_point.time_from_start = joint_trajectory.points[i].time_from_start;
    cartesian_trajectory.points.push_back(curr_point);
  }
  cartesian_trajectory.points.back().wrench.force.z = 0;
}

void crs_motion_planning::genCartesianTrajectoryGoal(
    const cartesian_trajectory_msgs::msg::CartesianTrajectory& cartesian_trajectory,
    const cartesianTrajectoryConfig traj_config,
    cartesian_trajectory_msgs::action::CartesianComplianceTrajectory::Goal& cartesian_trajectory_goal)
{
  Eigen::Vector3d tcp_force_vec = Eigen::Vector3d::Zero();
  tcp_force_vec.z() = traj_config.target_force;
  geometry_msgs::msg::Vector3 target_wrench_force, target_wrench_torque;
  target_wrench_force = tf2::toMsg(tcp_force_vec, target_wrench_force);
  target_wrench_torque = tf2::toMsg(Eigen::Vector3d::Zero(), target_wrench_torque);
  geometry_msgs::msg::Wrench target_wrench;
  target_wrench.force = target_wrench_force;
  target_wrench.torque = target_wrench_torque;

  cartesian_trajectory_goal.speed = traj_config.target_speed;
  cartesian_trajectory_goal.force = target_wrench;
  cartesian_trajectory_goal.path_tolerance.position_error = traj_config.path_pose_tolerance;
  cartesian_trajectory_goal.path_tolerance.orientation_error = traj_config.path_ori_tolerance;
  cartesian_trajectory_goal.goal_tolerance.position_error = traj_config.goal_pose_tolerance;
  cartesian_trajectory_goal.goal_tolerance.orientation_error = traj_config.goal_ori_tolerance;
  cartesian_trajectory_goal.path_tolerance.wrench_error.force = traj_config.force_tolerance;
  cartesian_trajectory_goal.trajectory = cartesian_trajectory;
}

bool crs_motion_planning::execSurfaceTrajectory(
    rclcpp_action::Client<cartesian_trajectory_msgs::action::CartesianComplianceTrajectory>::SharedPtr ac,
    const rclcpp::Logger& logger,
    const trajectory_msgs::msg::JointTrajectory& traj,
    const crs_motion_planning::cartesianTrajectoryConfig& traj_config)
{
  using namespace cartesian_trajectory_msgs::action;
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
  cartesian_trajectory_msgs::msg::CartesianTrajectory cart_traj;
  crs_motion_planning::genCartesianTrajectory(traj, traj_config, cart_traj);
  crs_motion_planning::genCartesianTrajectoryGoal(cart_traj, traj_config, goal);

  auto goal_options = rclcpp_action::Client<CartesianComplianceTrajectory>::SendGoalOptions();

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

bool crs_motion_planning::execSurfaceTrajectory(
    rclcpp_action::Client<cartesian_trajectory_msgs::action::CartesianComplianceTrajectory>::SharedPtr ac,
    const rclcpp::Logger& logger,
    const cartesian_trajectory_msgs::msg::CartesianTrajectory& traj,
    const cartesianTrajectoryConfig& traj_config)
{
  using namespace cartesian_trajectory_msgs::action;
  using namespace rclcpp_action;
  using GoalHandleT = Client<CartesianComplianceTrajectory>::GoalHandle;

  rclcpp::Duration traj_dur(traj.points.back().time_from_start);
  bool res = false;
  std::string err_msg;

  auto print_traj_time = [&](const cartesian_trajectory_msgs::msg::CartesianTrajectory& traj) {
    RCLCPP_ERROR(logger, "Trajectory with %lu points time data", traj.points.size());
    for (std::size_t i = 0; i < traj.points.size(); i++)
    {
      const auto& p = traj.points[i];
      RCLCPP_ERROR(logger, "\tPoint %lu : %f secs", rclcpp::Duration(p.time_from_start).seconds());
    }
  };

  CartesianComplianceTrajectory::Goal goal;
  crs_motion_planning::genCartesianTrajectoryGoal(traj, traj_config, goal);

  auto goal_options = rclcpp_action::Client<CartesianComplianceTrajectory>::SendGoalOptions();

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

geometry_msgs::msg::PoseArray
crs_motion_planning::removeEdgeWaypoints(const geometry_msgs::msg::PoseArray& input_waypoints, const double& buffer)
{
  geometry_msgs::msg::PoseArray cropped_waypoints;
  cropped_waypoints.header = input_waypoints.header;
  geometry_msgs::msg::Pose first_point = input_waypoints.poses.front();
  geometry_msgs::msg::Pose last_point = input_waypoints.poses.back();
  for (auto pose : input_waypoints.poses)
  {
    if (abs(crs_motion_planning::calcPoseDist(pose, first_point)) > buffer &&
        abs(crs_motion_planning::calcPoseDist(pose, last_point)) > buffer)
      cropped_waypoints.poses.push_back(pose);
  }
  return cropped_waypoints;
}

std::vector<geometry_msgs::msg::PoseArray>
crs_motion_planning::removeEdgeWaypoints(const std::vector<geometry_msgs::msg::PoseArray>& input_waypoints,
                                         const double& buffer)
{
  std::vector<geometry_msgs::msg::PoseArray> cropped_waypoints;
  for (auto waypoints : input_waypoints)
  {
    geometry_msgs::msg::PoseArray cropped_raster = crs_motion_planning::removeEdgeWaypoints(waypoints, buffer);
    if (cropped_raster.poses.size() > 0)
      cropped_waypoints.push_back(cropped_raster);
  }
  return cropped_waypoints;
}

geometry_msgs::msg::PoseArray
crs_motion_planning::transformWaypoints(const geometry_msgs::msg::PoseArray& input_waypoints,
                                        const geometry_msgs::msg::TransformStamped& transform,
                                        const bool inverse)
{
  geometry_msgs::msg::PoseArray transformed_waypoints;
  Eigen::Isometry3d transform_eig = tf2::transformToEigen(transform);
  for (auto pose : input_waypoints.poses)
  {
    Eigen::Affine3d given_point;
    tf2::fromMsg(pose, given_point);
    Eigen::Affine3d transformed_point = transform_eig.inverse() * given_point;
    if (inverse)
      transformed_point = transform_eig * given_point;
    geometry_msgs::msg::Pose transformed_pose = tf2::toMsg(transformed_point);
    transformed_waypoints.poses.push_back(transformed_pose);
  }
  return transformed_waypoints;
}

std::vector<geometry_msgs::msg::PoseArray>
crs_motion_planning::transformWaypoints(const std::vector<geometry_msgs::msg::PoseArray>& input_waypoints,
                                        const geometry_msgs::msg::TransformStamped& transform,
                                        const bool inverse)
{
  std::vector<geometry_msgs::msg::PoseArray> transformed_waypoints;
  for (auto waypoints : input_waypoints)
  {
    transformed_waypoints.push_back(crs_motion_planning::transformWaypoints(waypoints, transform, inverse));
  }
  return transformed_waypoints;
}

bool crs_motion_planning::filterReachabilitySphere(const geometry_msgs::msg::PoseArray& waypoints,
                                                   const double& radius,
                                                   geometry_msgs::msg::PoseArray& reachable_waypoints)
{
  bool some_points_reachable = false;
  for (auto pose : waypoints.poses)
  {
    double distance =
        std::sqrt(std::pow(pose.position.x, 2) + std::pow(pose.position.y, 2) + std::pow(pose.position.z, 2));
    if (distance < radius)
    {
      reachable_waypoints.poses.push_back(pose);
      some_points_reachable = true;
    }
  }
  return some_points_reachable;
}

bool crs_motion_planning::filterReachabilitySphere(const std::vector<geometry_msgs::msg::PoseArray>& waypoints_vec,
                                                   const double& radius,
                                                   std::vector<geometry_msgs::msg::PoseArray>& reachable_waypoints_vec)
{
  bool some_points_reachable = false;
  for (auto waypoints : waypoints_vec)
  {
    geometry_msgs::msg::PoseArray reachable_waypoints;
    if (crs_motion_planning::filterReachabilitySphere(waypoints, radius, reachable_waypoints))
    {
      some_points_reachable = true;
      reachable_waypoints_vec.push_back(reachable_waypoints);
    }
  }
  return some_points_reachable;
}

geometry_msgs::msg::PoseArray
crs_motion_planning::filterSingularityCylinder(const geometry_msgs::msg::PoseArray& waypoints, const double& radius)
{
  geometry_msgs::msg::PoseArray filtered_waypoints;
  filtered_waypoints.header = waypoints.header;
  for (auto pose : waypoints.poses)
  {
    double rad_point = sqrt(pow(pose.position.x,2) + pow(pose.position.y,2));
    if (rad_point > radius)
    {
      filtered_waypoints.poses.push_back(pose);
    }
  }
  return filtered_waypoints;
}

std::vector<geometry_msgs::msg::PoseArray>
crs_motion_planning::filterSingularityCylinder(const std::vector<geometry_msgs::msg::PoseArray>& waypoints, const double& radius)
{
  std::vector<geometry_msgs::msg::PoseArray> filtered_waypoints;
  for (auto waypoints : waypoints)
  {
    geometry_msgs::msg::PoseArray filtered_raster = crs_motion_planning::filterSingularityCylinder(waypoints, radius);
    if (filtered_raster.poses.size() > 0)
      filtered_waypoints.push_back(filtered_raster);
  }
  return filtered_waypoints;
}

double crs_motion_planning::calcPoseDist(const geometry_msgs::msg::Pose& waypoint1,
                                         const geometry_msgs::msg::Pose& waypoint2)
{
  Eigen::Affine3d wp1, wp2;
  tf2::fromMsg(waypoint1, wp1);
  tf2::fromMsg(waypoint2, wp2);
  double dist = (wp1.translation() - wp2.translation()).norm();
  return dist;
}

double crs_motion_planning::calcRotation(const geometry_msgs::msg::Pose& waypoint1,
                                         const geometry_msgs::msg::Pose& waypoint2)
{
  Eigen::Affine3d wp1, wp2;
  tf2::fromMsg(waypoint1, wp1);
  tf2::fromMsg(waypoint2, wp2);
  Eigen::Affine3d wp1_2_wp2_transform = wp1.inverse() * wp2;
  Eigen::AngleAxisd rot_between_waypoints = Eigen::AngleAxisd(wp1_2_wp2_transform.rotation());
  return abs(rot_between_waypoints.angle());
}

std::vector<geometry_msgs::msg::PoseArray>
crs_motion_planning::organizeRasters(const std::vector<geometry_msgs::msg::PoseArray>& waypoints_vec)
{
  std::vector<geometry_msgs::msg::PoseArray> organized_rasters;
  std::vector<std::size_t> available_rasters;
  for (size_t i = 1; i < waypoints_vec.size(); ++i)
  {
    available_rasters.push_back(i);
  }
  organized_rasters.push_back(waypoints_vec.front());
  geometry_msgs::msg::Pose prev_end = organized_rasters.front().poses.back();
  for (size_t i = 1; i < waypoints_vec.size(); ++i)
  {
    double min_dist = 1000;
    size_t next_i = i;
    bool back = false;
    int element_num_remove = 0, count = 0;
    for (size_t raster_i : available_rasters)
    {
      geometry_msgs::msg::Pose front_pose, back_pose;
      front_pose = waypoints_vec[raster_i].poses.front();
      back_pose = waypoints_vec[raster_i].poses.back();
      double front_dist = crs_motion_planning::calcPoseDist(prev_end, front_pose);
      double back_dist = crs_motion_planning::calcPoseDist(prev_end, back_pose);
      if (front_dist < min_dist)
      {
        min_dist = front_dist;
        next_i = raster_i;
        back = false;
        element_num_remove = count;
      }
      if (back_dist < min_dist)
      {
        min_dist = back_dist;
        next_i = raster_i;
        back = true;
        element_num_remove = count;
      }
      count++;
    }
    geometry_msgs::msg::PoseArray added_vec;
    if (!back)
    {
      added_vec = waypoints_vec[next_i];
    }
    else
    {
      for (size_t pose_i = waypoints_vec[next_i].poses.size(); pose_i > 0; --pose_i)
      {
        added_vec.poses.push_back(waypoints_vec[next_i].poses[pose_i - 1]);
      }
    }
    organized_rasters.push_back(added_vec);
    prev_end = organized_rasters.back().poses.back();
    available_rasters.erase(available_rasters.begin() + element_num_remove);
  }
  return organized_rasters;
}

std::vector<double> crs_motion_planning::findRasterRotation(const geometry_msgs::msg::PoseArray& waypoints)
{
  Eigen::Affine3d prev_waypoint;
  double rotation_sum = 0;
  double distance_sum = 0;
  double max_rot_rate = 0;
  tf2::fromMsg(waypoints.poses.front(), prev_waypoint);
  for (size_t i = 1; i < waypoints.poses.size(); ++i)
  {
    Eigen::Affine3d curr_waypoint;
    tf2::fromMsg(waypoints.poses[i], curr_waypoint);
    Eigen::Affine3d prev_2_curr_transform = prev_waypoint.inverse() * curr_waypoint;
    Eigen::AngleAxisd rot_between_waypoints = Eigen::AngleAxisd(prev_2_curr_transform.rotation());
    rotation_sum += abs(rot_between_waypoints.angle());
    distance_sum += prev_2_curr_transform.translation().norm();
    double rot_rate = abs(rot_between_waypoints.angle()) / prev_2_curr_transform.translation().norm();
    if (rot_rate > max_rot_rate)
      max_rot_rate = rot_rate;
    prev_waypoint = curr_waypoint;
  }
  std::vector<double> return_vals;
  return_vals.push_back(max_rot_rate);
  return_vals.push_back(rotation_sum / distance_sum);
  return return_vals;
}
