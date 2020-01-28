#include <crs_motion_planning/path_processing_utils.h>

bool crs_motion_planning::parsePathFromFile(const std::string& yaml_filepath,
                                            const std::string& waypoint_origin_frame,
                                            std::vector<std::vector<geometry_msgs::msg::PoseStamped>> &raster_strips)
{
    YAML::Node full_yaml_node = YAML::LoadFile(yaml_filepath);
    YAML::Node paths = full_yaml_node[0]["paths"];
    std::double_t offset_strip = 0.0;
    for (YAML::const_iterator path_it = paths.begin(); path_it != paths.end(); ++path_it)
    {
        std::vector<geometry_msgs::msg::PoseStamped> temp_poses;
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

             Eigen::Isometry3d offset_pose = original_pose * Eigen::Translation3d(0.0, 0.0, offset_waypoint) * Eigen::Quaterniond(0, 1, 0, 0);


             current_pose.pose = tf2::toMsg(offset_pose);

             temp_poses.push_back(tf2::toMsg(current_pose));
           }
           catch (YAML::InvalidNode &e)
           {
             continue;
           }

        }
        raster_strips.push_back(temp_poses);
    }
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

void crs_motion_planning::rasterStripsToMarkerArray(const std::vector<geometry_msgs::msg::PoseStamped>& strips,
                                                    const std::string& frame,
                                                    visualization_msgs::msg::MarkerArray &arrows,
                                                    std::vector<float> color,
                                                    double size)
{
    arrows.markers.clear(); // reset marker list

    int arrow_count = 0;
    for (auto pose : strips)
    {
        Eigen::Affine3d arrow_pose;
        tf2::fromMsg(pose.pose, arrow_pose);

        geometry_msgs::msg::Point p, p_tip;
        p.x = pose.pose.position.x;
        p.y = pose.pose.position.y;
        p.z = pose.pose.position.z;

        Eigen::Vector3d tip = Eigen::Vector3d(p.x, p.y, p.z) + arrow_pose.linear() * Eigen::Vector3d(0, 0, size);
        p_tip = tf2::toMsg(tip);

        visualization_msgs::msg::Marker arrow;
        arrow.type =  visualization_msgs::msg::Marker::ARROW;
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

void crs_motion_planning::rasterStripsToMarkerArray(const std::vector<std::vector<geometry_msgs::msg::PoseStamped>>& strips,
                                                    const std::string& frame,
                                                    visualization_msgs::msg::MarkerArray &arrows,
                                                    std::vector<float> color,
                                                    double size)
{
    std::vector<geometry_msgs::msg::PoseStamped> combined_strips;
    for(auto strip : strips)
    {
        combined_strips.insert(combined_strips.end(), strip.begin(), strip.end());
    }
    crs_motion_planning::rasterStripsToMarkerArray(combined_strips, frame, arrows, color, size);
}

void crs_motion_planning::failedEdgesToMarkerArray(const std::vector<geometry_msgs::msg::PoseStamped>& vertices,
                                                   const std::string& frame,
                                                   visualization_msgs::msg::Marker &markers,
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

    for (auto vertex : vertices)
    {
        markers.points.push_back(vertex.pose.position);
    }
}

void crs_motion_planning::cleanRasterStrip(const std::vector<geometry_msgs::msg::PoseStamped>& original_strip,
                      const std::vector<std::size_t>& failed_vertices,
                      std::vector<std::vector<geometry_msgs::msg::PoseStamped>> &fixed_strips,
                      std::vector<geometry_msgs::msg::PoseStamped> &failed_vertex_poses)
{
    size_t j = 0;
    int min_len_raster = 0;
    std::vector<geometry_msgs::msg::PoseStamped> curr_raster;
    for (size_t i = 0; i < original_strip.size(); ++i)
    {
        if (j <= failed_vertices.size() && i == failed_vertices[j])
        {
            if (curr_raster.size() > min_len_raster)
            {
                fixed_strips.push_back(curr_raster);
            }
            j++;
            curr_raster.clear();
            failed_vertex_poses.push_back(original_strip[i]);
        }
        else
        {
            curr_raster.push_back(original_strip[i]);
        }
    }
    if (curr_raster.size() > min_len_raster)
    {
        fixed_strips.push_back(curr_raster);
    }
}

void crs_motion_planning::splitRastersByJointDist(const trajectory_msgs::msg::JointTrajectory& given_traj,
                                                  const std::vector<geometry_msgs::msg::PoseStamped>& given_raster,
                                                  const double& desired_ee_vel,
                                                  const double& max_joint_vel,
                                                  std::vector<trajectory_msgs::msg::JointTrajectory> &split_traj,
                                                  std::vector<std::vector<geometry_msgs::msg::PoseStamped>> &split_rasters,
                                                  std::vector<std::vector<double>> &time_steps)
{
    std::vector<geometry_msgs::msg::PoseStamped> curr_raster;
    trajectory_msgs::msg::JointTrajectory curr_traj;
    std::vector<double> curr_time_steps;
    curr_traj.joint_names = given_traj.joint_names;
    curr_traj.points.push_back(given_traj.points[0]);
    curr_raster.push_back(given_raster[0]);
    curr_time_steps.push_back(0);
    for (size_t i = 1; i < given_traj.points.size(); ++i)
    {
        // Find largest joint motion
        double max_joint_diff = 0;
        for (size_t j = 0; j<given_traj.points[i].positions.size(); ++j)
        {
            if (abs(given_traj.points[i].positions[j] - given_traj.points[i-1].positions[j]) > max_joint_diff)
                max_joint_diff = abs(given_traj.points[i].positions[j] - given_traj.points[i-1].positions[j]);
        }

        // Find cartesian distance traveled between points
        Eigen::Isometry3d curr_cart_pose, prev_cart_pose;
        Eigen::Vector3d cart_pose_diff;
        tf2::fromMsg(given_raster[i].pose, curr_cart_pose);
        tf2::fromMsg(given_raster[i-1].pose, prev_cart_pose);
        cart_pose_diff = curr_cart_pose.translation() - prev_cart_pose.translation();
        double dist_traveled = cart_pose_diff.norm();

        // Calculate max required joint velocity
        double req_joint_vel = (desired_ee_vel * max_joint_diff) / dist_traveled;
        double curr_time_step = dist_traveled / desired_ee_vel;

        // If required joint velocity is higher than max allowable then split raster
        if (req_joint_vel > max_joint_vel)
        {
            std::cout << "Size of curr traj: " << curr_raster.size() << std::endl;
            split_traj.push_back(curr_traj);
            curr_traj.points.clear();
            split_rasters.push_back(curr_raster);
            curr_raster.clear();
            time_steps.push_back(curr_time_steps);
            curr_time_steps.clear();
        }
        // Add current point to running trajectory and raster list
        curr_traj.points.push_back(given_traj.points[i]);
        curr_raster.push_back(given_raster[i]);
        curr_time_steps.push_back(curr_time_step);

    }
    split_traj.push_back(curr_traj);
    split_rasters.push_back(curr_raster);
    time_steps.push_back(curr_time_steps);
}
