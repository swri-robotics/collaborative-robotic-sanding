#include <crs_motion_planning/path_planning_utils.h>
#include <tesseract_rosutils/conversions.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("PATH_PLANNING_UTILS");

namespace crs_motion_planning
{
bool loadPathPlanningConfig(const std::string& yaml_fp, pathPlanningConfig::Ptr& motion_planner_config)
{
  descartesConfig descartes_config;
  trajoptSurfaceConfig trajopt_surface_config;
  crs_motion_planning::omplConfig ompl_config;
  crs_motion_planning::trajoptFreespaceConfig trajopt_freespace_config;
  motion_planner_config = std::make_unique<crs_motion_planning::pathPlanningConfig>();
  ;

  YAML::Node full_yaml_node = YAML::LoadFile(yaml_fp);
  if (!full_yaml_node)
  {
    RCLCPP_ERROR(LOGGER, "Failed to load into YAML from file %s", yaml_fp.c_str());
    return false;
  }

  try
  {
    // DESCARTES CONFIG
    YAML::Node descartes_yaml = full_yaml_node["descartes"];
    descartes_config.axial_step = descartes_yaml["axial_step"].as<double>();
    descartes_config.collision_safety_margin = descartes_yaml["collision_safety_margin"].as<double>();
    std::vector<double> xyzrpy = descartes_yaml["additional_tool_offset"].as<std::vector<double>>();
    descartes_config.tool_offset = Eigen::Translation3d(Eigen::Vector3d(xyzrpy[0], xyzrpy[1], xyzrpy[2])) *
                                   Eigen::AngleAxisd(xyzrpy[3], Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(xyzrpy[4], Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(xyzrpy[5], Eigen::Vector3d::UnitZ());
    descartes_config.allow_collisions = false;

    // TRAJOPT SURFACE CONFIG
    YAML::Node trajopt_surface_yaml = full_yaml_node["trajopt_surface"];
    trajopt_surface_config.smooth_velocities = trajopt_surface_yaml["smooth_velocities"].as<bool>();
    trajopt_surface_config.smooth_accelerations = trajopt_surface_yaml["smooth_accelerations"].as<bool>();
    trajopt_surface_config.smooth_jerks = trajopt_surface_yaml["smooth_jerks"].as<bool>();
    trajopt_surface_config.longest_valid_segment_fraction =
        trajopt_surface_yaml["longest_valid_segment_fraction"].as<double>();
    trajopt_surface_config.longest_valid_segment_length =
        trajopt_surface_yaml["longest_valid_segment_length"].as<double>();
    std::vector<double> xyzrpy_surface_co = trajopt_surface_yaml["surface_coefficients"].as<std::vector<double>>();
    Eigen::VectorXd surface_coeffs(6);
    surface_coeffs << xyzrpy_surface_co[0], xyzrpy_surface_co[1], xyzrpy_surface_co[2], xyzrpy_surface_co[3],
        xyzrpy_surface_co[4], xyzrpy_surface_co[5];
    trajopt_surface_config.surface_coeffs = surface_coeffs;
    trajopt_surface_config.waypoints_critical = trajopt_surface_yaml["waypoints_critical"].as<bool>();
    // trajopt surface collision configs
    tesseract_motion_planners::CollisionCostConfig coll_cost_config_srfc;
    coll_cost_config_srfc.enabled = trajopt_surface_yaml["collision_cost"]["enabled"].as<bool>();
    if (coll_cost_config_srfc.enabled)
    {
      coll_cost_config_srfc.buffer_margin = trajopt_surface_yaml["collision_cost"]["buffer_margin"].as<double>();
    }
    trajopt_surface_config.coll_cst_cfg = coll_cost_config_srfc;
    tesseract_motion_planners::CollisionConstraintConfig coll_cnt_config_srfc;
    coll_cnt_config_srfc.enabled = trajopt_surface_yaml["collision_constraint"]["enabled"].as<bool>();
    if (coll_cnt_config_srfc.enabled)
    {
      coll_cnt_config_srfc.safety_margin = trajopt_surface_yaml["collision_constraint"]["safety_margin"].as<double>();
    }
    trajopt_surface_config.coll_cnt_cfg = coll_cnt_config_srfc;
    // trajopt surface special collision pairs
    if (trajopt_surface_yaml["special_collision_costs"])
    {
      YAML::Node special_costs = trajopt_surface_yaml["special_collision_costs"];
      for (const auto& cost_node : special_costs)
      {
        std::string link1 = cost_node["link1"].as<std::string>();
        std::string link2 = cost_node["link2"].as<std::string>();
        double distance = cost_node["distance"].as<double>();
        double cost = cost_node["cost"].as<double>();
        trajopt_surface_config.special_collision_cost.push_back({ link1, link2, distance, cost });
      }
    }
    if (trajopt_surface_yaml["special_collision_constraints"])
    {
      YAML::Node special_constraints = trajopt_surface_yaml["special_collision_constraints"];
      for (const auto& constraint_node : special_constraints)
      {
        std::string link1 = constraint_node["link1"].as<std::string>();
        std::string link2 = constraint_node["link2"].as<std::string>();
        double distance = constraint_node["distance"].as<double>();
        double cost = constraint_node["cost"].as<double>();
        trajopt_surface_config.special_collision_constraint.push_back({ link1, link2, distance, cost });
      }
    }

    // OMPL CONFIG
    YAML::Node ompl_yaml = full_yaml_node["ompl"];
    ompl_config.collision_safety_margin = ompl_yaml["collision_safety_margin"].as<double>();
    ompl_config.planning_time = ompl_yaml["planning_time"].as<double>();
    ompl_config.simplify = ompl_yaml["simplify"].as<bool>();
    ompl_config.range = ompl_yaml["range"].as<double>();
    ompl_config.num_threads = ompl_yaml["num_threads"].as<int>();
    ompl_config.max_solutions = ompl_yaml["max_solutions"].as<int>();
    ompl_config.n_output_states = ompl_yaml["default_n_output_states"].as<double>();
    ompl_config.longest_valid_segment_fraction = ompl_yaml["longest_valid_segment_fraction"].as<double>();
    ompl_config.longest_valid_segment_length = ompl_yaml["longest_valid_segment_length"].as<double>();

    // TRAJOPT FREESPACE CONFIG
    YAML::Node trajopt_freespace_yaml = full_yaml_node["trajopt_freespace"];
    trajopt_freespace_config.smooth_velocities = trajopt_freespace_yaml["smooth_velocities"].as<bool>();
    trajopt_freespace_config.smooth_accelerations = trajopt_freespace_yaml["smooth_accelerations"].as<bool>();
    trajopt_freespace_config.smooth_jerks = trajopt_freespace_yaml["smooth_jerks"].as<bool>();
    trajopt_freespace_config.longest_valid_segment_fraction =
        trajopt_freespace_yaml["longest_valid_segment_fraction"].as<double>();
    trajopt_freespace_config.longest_valid_segment_length =
        trajopt_freespace_yaml["longest_valid_segment_length"].as<double>();
    // trajopt freespace collision configs
    tesseract_motion_planners::CollisionCostConfig coll_cost_config_fs;
    coll_cost_config_fs.enabled = trajopt_freespace_yaml["collision_cost"]["enabled"].as<bool>();
    if (coll_cost_config_fs.enabled)
    {
      coll_cost_config_fs.buffer_margin = trajopt_freespace_yaml["collision_cost"]["buffer_margin"].as<double>();
    }
    trajopt_freespace_config.coll_cst_cfg = coll_cost_config_fs;
    tesseract_motion_planners::CollisionConstraintConfig coll_cnt_config_fs;
    coll_cnt_config_fs.enabled = trajopt_freespace_yaml["collision_constraint"]["enabled"].as<bool>();
    if (coll_cnt_config_fs.enabled)
    {
      coll_cnt_config_fs.safety_margin = trajopt_freespace_yaml["collision_constraint"]["safety_margin"].as<double>();
    }
    trajopt_freespace_config.coll_cnt_cfg = coll_cnt_config_fs;
    // trajopt freespace special collision pairs
    if (trajopt_freespace_yaml["special_collision_costs"])
    {
      YAML::Node special_costs = trajopt_freespace_yaml["special_collision_costs"];
      for (const auto& cost_node : special_costs)
      {
        std::string link1 = cost_node["link1"].as<std::string>();
        std::string link2 = cost_node["link2"].as<std::string>();
        double distance = cost_node["distance"].as<double>();
        double cost = cost_node["cost"].as<double>();
        trajopt_freespace_config.special_collision_cost.push_back({ link1, link2, distance, cost });
      }
    }
    if (trajopt_freespace_yaml["special_collision_constraints"])
    {
      YAML::Node special_constraints = trajopt_freespace_yaml["special_collision_constraints"];
      for (const auto& constraint_node : special_constraints)
      {
        std::string link1 = constraint_node["link1"].as<std::string>();
        std::string link2 = constraint_node["link2"].as<std::string>();
        double distance = constraint_node["distance"].as<double>();
        double cost = constraint_node["cost"].as<double>();
        trajopt_freespace_config.special_collision_constraint.push_back({ link1, link2, distance, cost });
      }
    }

    // GENERAL CONFIG
    YAML::Node general_yaml = full_yaml_node["general"];
    motion_planner_config->use_trajopt_freespace = general_yaml["use_trajopt_freespace"].as<bool>();
    motion_planner_config->use_trajopt_surface = general_yaml["use_trajopt_surface"].as<bool>();
    motion_planner_config->default_to_descartes = general_yaml["default_to_descartes"].as<bool>();
    motion_planner_config->simplify_start_end_freespace = general_yaml["simplify_start_end_freespace"].as<bool>();
    motion_planner_config->manipulator = general_yaml["manipulator"].as<std::string>();
    motion_planner_config->world_frame = general_yaml["world_frame"].as<std::string>();
    motion_planner_config->robot_base_frame = general_yaml["robot_base_frame"].as<std::string>();
    motion_planner_config->tool0_frame = general_yaml["tool0_frame"].as<std::string>();
    motion_planner_config->required_tool_vel = general_yaml["required_tool_vel"].as<bool>();
    motion_planner_config->max_joint_vel = general_yaml["max_joint_vel"].as<double>();
    motion_planner_config->max_joint_vel_mult = general_yaml["max_joint_vel_mult"].as<double>();
    motion_planner_config->max_surface_dist = general_yaml["max_surface_dist"].as<double>();
    motion_planner_config->max_joint_acc = general_yaml["max_joint_acc"].as<double>();
    motion_planner_config->add_approach_and_retreat = general_yaml["add_approach_and_retreat"].as<bool>();
    motion_planner_config->minimum_raster_length = general_yaml["minimum_raster_length"].as<std::size_t>();
    motion_planner_config->trajopt_verbose_output = general_yaml["trajopt_verbose_output"].as<bool>();
    motion_planner_config->combine_strips = general_yaml["combine_strips"].as<bool>();
    motion_planner_config->global_descartes = general_yaml["global_descartes"].as<bool>();
    motion_planner_config->descartes_config = descartes_config;
    motion_planner_config->trajopt_surface_config = trajopt_surface_config;
    motion_planner_config->ompl_config = ompl_config;
    motion_planner_config->trajopt_freespace_config = trajopt_freespace_config;
    return true;
  }
  catch (YAML::InvalidNode& e)
  {
    RCLCPP_ERROR(LOGGER, "Invalid node while parsing yaml %s, %s", yaml_fp.c_str(), e.what());
    return false;
  }
  catch (YAML::BadConversion& e)
  {
    RCLCPP_ERROR(LOGGER, "failed to parse yaml %s, %s", yaml_fp.c_str(), e.what());
    return false;
  }
  catch (YAML::KeyNotFound& e)
  {
    RCLCPP_ERROR(LOGGER, "Key not found while parsing %s, %s", yaml_fp.c_str(), e.what());
    return false;
  }
}

crsMotionPlanner::crsMotionPlanner(pathPlanningConfig::Ptr config, rclcpp::Logger logger)
  : config_(std::move(config)), logger_(std::move(logger))
{
}

crsMotionPlanner::crsMotionPlanner(pathPlanningConfig config, rclcpp::Logger logger) : logger_(std::move(logger))
{
  config_ = std::make_unique<crs_motion_planning::pathPlanningConfig>();
  *config_ = config;
}

void crsMotionPlanner::updateConfiguration(pathPlanningConfig::Ptr config) { config_ = std::move(config); }
bool crsMotionPlanner::generateDescartesSeed(const geometry_msgs::msg::PoseArray& waypoints_pose_array,
                                             std::vector<std::size_t>& failed_edges,
                                             std::vector<std::size_t>& failed_vertices,
                                             trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  const double axial_step = config_->descartes_config.axial_step;
  const bool allow_collisions = config_->descartes_config.allow_collisions;
  const double collision_safety_margin = config_->descartes_config.collision_safety_margin;
  tesseract::Tesseract::Ptr tesseract_local = config_->tesseract_local;
  const std::shared_ptr<const tesseract_environment::Environment> env = tesseract_local->getEnvironmentConst();
  tesseract_common::TransformMap curr_transforms = env->getCurrentState()->link_transforms;

  tesseract_kinematics::ForwardKinematics::ConstPtr kin =
      tesseract_local->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_->manipulator);

  tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), kin->getActiveLinkNames(), curr_transforms);

  auto collision_checker = std::make_shared<tesseract_motion_planners::DescartesCollisionD>(
      env, adjacency_map->getActiveLinkNames(), kin->getJointNames(), collision_safety_margin);

  std::vector<descartes_light::PositionSamplerD::Ptr> sampler_result;

  Eigen::Isometry3d world_to_base_link, world_to_sander, world_to_tool0, tool0_to_sander;
  world_to_base_link = curr_transforms.find(config_->robot_base_frame)->second;
  world_to_sander = curr_transforms.find(config_->tcp_frame)->second;
  world_to_tool0 = curr_transforms.find(config_->tool0_frame)->second;
  tool0_to_sander = world_to_tool0.inverse() * world_to_sander;
  tool0_to_sander = tool0_to_sander * config_->tool_offset * config_->descartes_config.tool_offset;
  descartes_light::KinematicsInterfaceD::Ptr kin_interface =
      std::make_shared<ur_ikfast_kinematics::UR10eKinematicsD>(world_to_base_link, tool0_to_sander, nullptr, nullptr);

  for (size_t i = 0; i < waypoints_pose_array.poses.size(); ++i)
  {
    Eigen::Isometry3d current_waypoint_pose;
    tf2::fromMsg(waypoints_pose_array.poses[i], current_waypoint_pose);
    descartes_light::PositionSamplerD::Ptr curr_sampler_result =
        std::make_shared<descartes_light::AxialSymmetricSamplerD>(
            current_waypoint_pose,
            kin_interface,
            axial_step,
            std::shared_ptr<descartes_light::CollisionInterface<double>>(collision_checker->clone()),
            allow_collisions);
    sampler_result.push_back(curr_sampler_result);
  }

  auto edge_eval = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluatorD>(kin_interface->dof());
  auto timing_constraint =
      std::vector<descartes_core::TimingConstraintD>(sampler_result.size(), std::numeric_limits<double>::max());

  descartes_light::SolverD graph_builder(kin_interface->dof());

  if (!graph_builder.build(std::move(sampler_result), std::move(timing_constraint), std::move(edge_eval)))
  {
    failed_edges = graph_builder.getFailedEdges();
    failed_vertices = graph_builder.getFailedVertices();
    return false;
  }

  std::vector<double> solution;
  if (!graph_builder.search(solution))
  {
    return false;
  }

  Eigen::Map<Eigen::VectorXd> solution_vec(&solution[0], solution.size());
  Eigen::VectorXd seed_traj(solution_vec.size());
  seed_traj << solution_vec;

  int n_rows = seed_traj.size() / kin_interface->dof();
  Eigen::MatrixXd joint_traj_eigen_out =
      Eigen::Map<Eigen::MatrixXd>(seed_traj.data(), kin_interface->dof(), n_rows).transpose();

  tesseract_rosutils::toMsg(joint_trajectory, kin->getJointNames(), joint_traj_eigen_out);
  return true;
}

bool crsMotionPlanner::generateSurfacePlans(pathPlanningResults::Ptr& results)
{
  // Load rasters and get them in usable form
  std::vector<geometry_msgs::msg::PoseArray> raster_strips = config_->rasters;

  // Determine reachability of all rasters using descartes
  trajectory_msgs::msg::JointTrajectory joint_traj_msg_out_init, joint_traj_msg_out_final;
  RCLCPP_INFO(logger_, "RUNNING FIRST DESCARTES");

  bool gen_preplan;
  std::vector<geometry_msgs::msg::PoseArray> split_reachable_rasters;
  bool any_successes = false;
  size_t count_strips = 0;
  geometry_msgs::msg::PoseArray failed_vertex_poses;

  for (auto strip : raster_strips)
  {
    std::vector<size_t> failed_edges, failed_vertices;
    ++count_strips;
    RCLCPP_INFO(logger_, "Running Descartes on Strip %i of %i", count_strips, raster_strips.size());
    gen_preplan = generateDescartesSeed(strip, failed_edges, failed_vertices, joint_traj_msg_out_init);
    RCLCPP_INFO(logger_, "DONE");

    // Check if all rasters reachable
    if (!gen_preplan)
    {
      std::vector<geometry_msgs::msg::PoseArray> split_rasters;
      // Split up raster based on where planning failures occurred
      geometry_msgs::msg::PoseArray curr_failed_vertex_poses;
      RCLCPP_INFO(logger_, "SOME POINTS UNREACHABLE, CLEANING...");
      crs_motion_planning::cleanRasterStrip(strip, failed_vertices, split_rasters, curr_failed_vertex_poses);

      // Store failed vertices
      results->unreachable_waypoints.poses.insert(results->unreachable_waypoints.poses.end(),
                                                  curr_failed_vertex_poses.poses.begin(),
                                                  curr_failed_vertex_poses.poses.end());

      for (auto split_strip : split_rasters)
      {
        if (split_strip.poses.size() >= config_->minimum_raster_length)
        {
          results->reachable_waypoints.poses.insert(
              results->reachable_waypoints.poses.end(), split_strip.poses.begin(), split_strip.poses.end());
          split_reachable_rasters.push_back(std::move(split_strip));
          any_successes = true;
        }
        else
        {
          results->skipped_rasters.push_back(split_strip);
        }
      }
      RCLCPP_INFO(logger_, "DONE");
    }
    else
    {
      if (strip.poses.size() >= config_->minimum_raster_length)
      {
        RCLCPP_INFO(logger_, "STRIP FULLY REACHABLE");
        results->reachable_waypoints.poses.insert(
            results->reachable_waypoints.poses.end(), strip.poses.begin(), strip.poses.end());
        split_reachable_rasters.push_back(std::move(strip));
        any_successes = true;
      }
      else
      {
        results->skipped_rasters.push_back(strip);
      }
    }
  }

  if (!any_successes)
  {
    RCLCPP_ERROR(logger_, "NO REACHABLE POINTS FOUND OR ALL REACHABLE STRIPS ARE TOO SHORT");
    results->msg_out = "NO REACHABLE POINTS FOUND OR ALL REACHABLE STRIPS ARE TOO SHORT";
    return false;
  }

  std::vector<trajectory_msgs::msg::JointTrajectory> second_descartes_trajs;
  // Combine all rasters together to perform descartes globally
  if (config_->global_descartes)
  {
    std::vector<size_t> traj_lengths;
    geometry_msgs::msg::PoseArray combined_rasters;
    for (size_t i = 0; i < split_reachable_rasters.size(); ++i)
    {
      traj_lengths.push_back(split_reachable_rasters[i].poses.size());
      combined_rasters.poses.insert(combined_rasters.poses.end(),
                                    split_reachable_rasters[i].poses.begin(),
                                    split_reachable_rasters[i].poses.end());
    }

    // Perform descartes globally
    trajectory_msgs::msg::JointTrajectory global_descartes_traj_round1;
    std::vector<size_t> failed_edges2, failed_vertices2;
    if (!generateDescartesSeed(combined_rasters, failed_edges2, failed_vertices2, global_descartes_traj_round1))
    {
      RCLCPP_ERROR(logger_, "PREVIOUSLY REACHABLE POINTS ARE NOW UNREACHABLE");
      results->msg_out = "PREVIOUSLY REACHABLE POINTS ARE NOW UNREACHABLE";
      return false;
    }

    // Store global descartes back into individual trajectories
    if (config_->combine_strips)
    {
      split_reachable_rasters.clear();
      split_reachable_rasters.push_back(combined_rasters);
      second_descartes_trajs.push_back(global_descartes_traj_round1);
    }
    else
    {
      size_t start_traj_i = 0;
      for (size_t i = 0; i < split_reachable_rasters.size(); ++i)
      {
        trajectory_msgs::msg::JointTrajectory curr_global_traj;
        curr_global_traj.joint_names = global_descartes_traj_round1.joint_names;
        for (size_t j = 0; j < traj_lengths[i]; ++j)
        {
          curr_global_traj.points.push_back(global_descartes_traj_round1.points[start_traj_i + j]);
        }
        second_descartes_trajs.push_back(curr_global_traj);
        start_traj_i += traj_lengths[i];
      }
    }
  }
  else
  {
    for (auto strip : split_reachable_rasters)
    {
      trajectory_msgs::msg::JointTrajectory curr_traj;
      std::vector<size_t> failed_edges2, failed_vertices2;
      if (!generateDescartesSeed(strip, failed_edges2, failed_vertices2, curr_traj))
      {
        RCLCPP_ERROR(logger_, "PREVIOUSLY REACHABLE POINTS ARE NOW UNREACHABLE");
        results->msg_out = "PREVIOUSLY REACHABLE POINTS ARE NOW UNREACHABLE";
        return false;
      }
      second_descartes_trajs.push_back(curr_traj);
    }
  }

  // Check for additional splits required by speed constraint
  std::vector<geometry_msgs::msg::PoseArray> post_speed_split_rasters;
  std::vector<trajectory_msgs::msg::JointTrajectory> post_speed_split_trajs;
  std::vector<std::vector<double>> post_speed_split_time_steps;
  size_t raster_n = 0;

  // Split by speed
  if (config_->required_tool_vel)
  {
    RCLCPP_INFO(logger_, "CHECKING FOR SPLITS IN %i TRAJECTORIES BY TOOL SPEED", second_descartes_trajs.size());
    for (auto curr_joint_traj : second_descartes_trajs)
    {
      // Split rasters based on signigicant joint motions
      std::vector<trajectory_msgs::msg::JointTrajectory> double_split_traj;
      std::vector<geometry_msgs::msg::PoseArray> resplit_rasters;
      double desired_ee_val = config_->tool_speed;
      double max_joint_vel = config_->max_joint_vel;
      std::vector<std::vector<double>> time_steps;
      if (crs_motion_planning::splitRastersByJointDist(curr_joint_traj,
                                                       split_reachable_rasters[raster_n],
                                                       desired_ee_val,
                                                       max_joint_vel,
                                                       config_->max_surface_dist,
                                                       double_split_traj,
                                                       resplit_rasters,
                                                       time_steps,
                                                       config_->max_joint_vel_mult))
      {
        RCLCPP_INFO(logger_, "FOUND A SPLIT");
        for (size_t i = 0; i < resplit_rasters.size(); ++i)
        {
          if (resplit_rasters[i].poses.size() >= config_->minimum_raster_length)
          {
            post_speed_split_rasters.push_back(std::move(resplit_rasters[i]));
            post_speed_split_trajs.push_back(double_split_traj[i]);
            post_speed_split_time_steps.push_back(time_steps[i]);
          }
          else
          {
            results->skipped_rasters.push_back(resplit_rasters[i]);
          }
        }
        RCLCPP_INFO(logger_, "SPLIT IT");
      }
      else
      {
        RCLCPP_INFO(logger_, "NO SPLITS");
        post_speed_split_rasters.push_back(split_reachable_rasters[raster_n]);
        post_speed_split_trajs.push_back(std::move(curr_joint_traj));
        post_speed_split_time_steps.push_back(time_steps[0]);
      }
      raster_n++;
    }
  }
  else
  {
    post_speed_split_rasters = split_reachable_rasters;
    post_speed_split_trajs = second_descartes_trajs;
  }
  RCLCPP_INFO(logger_, "FINISHED SPLITTING TRAJECTORIES");

  // Approach and retreat
  std::vector<geometry_msgs::msg::PoseArray> final_split_rasters;
  std::vector<trajectory_msgs::msg::JointTrajectory> final_split_trajs;
  std::vector<std::vector<double>> final_time_steps;
  if (post_speed_split_rasters.empty())
  {
    RCLCPP_ERROR(logger_, "ALL TRAJECTORIES TOO SHORT AFTER SPLITTING BY SPEED");
    return false;
  }
  if (config_->add_approach_and_retreat)
  {
    // Add approach and retreat
    double approach = config_->approach_distance;
    double retreat = config_->retreat_distance;
    for (size_t i = 0; i < post_speed_split_rasters.size(); ++i)
    {
      // Initialize variables
      bool ret_app_addable = true;
      geometry_msgs::msg::PoseArray modified_raster;
      trajectory_msgs::msg::JointTrajectory new_raster_traj;
      tesseract_motion_planners::JointWaypoint::Ptr begin_orig, end_orig, begin_new, end_new;
      tesseract_motion_planners::CartesianWaypoint::Ptr new_raster_begin, new_raster_end;

      // Add approach and retreat points
      crs_motion_planning::addApproachAndRetreat(post_speed_split_rasters[i], approach, retreat, modified_raster);
      RCLCPP_INFO(logger_, "ADDED APPROACH AND RETREAT CARTESIANS");

      // Make waypoints used for determining the closest joint states for each new raster point
      begin_orig = std::make_shared<tesseract_motion_planners::JointWaypoint>(
          post_speed_split_trajs[i].points[0].positions, post_speed_split_trajs[i].joint_names);
      end_orig = std::make_shared<tesseract_motion_planners::JointWaypoint>(
          post_speed_split_trajs[i].points.back().positions, post_speed_split_trajs[i].joint_names);
      Eigen::Vector3d goal_pose_begin(modified_raster.poses[0].position.x,
                                      modified_raster.poses[0].position.y,
                                      modified_raster.poses[0].position.z);
      Eigen::Quaterniond goal_ori_begin(modified_raster.poses[0].orientation.w,
                                        modified_raster.poses[0].orientation.x,
                                        modified_raster.poses[0].orientation.y,
                                        modified_raster.poses[0].orientation.z);
      new_raster_begin =
          std::make_shared<tesseract_motion_planners::CartesianWaypoint>(goal_pose_begin, goal_ori_begin);

      Eigen::Vector3d goal_pose_end(modified_raster.poses.back().position.x,
                                    modified_raster.poses.back().position.y,
                                    modified_raster.poses.back().position.z);
      Eigen::Quaterniond goal_ori_end(modified_raster.poses.back().orientation.w,
                                      modified_raster.poses.back().orientation.x,
                                      modified_raster.poses.back().orientation.y,
                                      modified_raster.poses.back().orientation.z);
      new_raster_end = std::make_shared<tesseract_motion_planners::CartesianWaypoint>(goal_pose_end, goal_ori_end);

      // Find the closest joint state for both the new starting point and the new ending point and convert to vector of
      // doubles
      // Store new joint states points in joint trajectory points
      trajectory_msgs::msg::JointTrajectoryPoint new_traj_point_begin, new_traj_point_end;
      if (!findClosestJointOrientation(begin_orig, new_raster_begin, begin_new, config_->descartes_config.axial_step))
      {
        ret_app_addable = false;
        RCLCPP_WARN(logger_, "UNABLE TO ADD APPROACH RASTER POSE");
        Eigen::VectorXd new_begin_eig = begin_orig->getPositions(post_speed_split_trajs[i].joint_names);
        std::vector<double> new_begin_vec(new_begin_eig.data(), new_begin_eig.data() + new_begin_eig.size());
        new_traj_point_begin.positions = new_begin_vec;
        new_raster_traj.points.push_back(new_traj_point_begin);
      }
      else
      {
        RCLCPP_INFO(logger_, "ADDED NEW RASTER BEGIN");
        Eigen::VectorXd new_begin_eig = begin_new->getPositions(post_speed_split_trajs[i].joint_names);
        std::vector<double> new_begin_vec(new_begin_eig.data(), new_begin_eig.data() + new_begin_eig.size());
        new_traj_point_begin.positions = new_begin_vec;
        new_raster_traj.points.push_back(new_traj_point_begin);
      }
      new_raster_traj.points.insert(new_raster_traj.points.end(),
                                    post_speed_split_trajs[i].points.begin(),
                                    post_speed_split_trajs[i].points.end());
      if (!findClosestJointOrientation(end_orig, new_raster_end, end_new, config_->descartes_config.axial_step))
      {
        ret_app_addable = false;
        RCLCPP_WARN(logger_, "UNABLE TO ADD RETREAT RASTER POSE");
        Eigen::VectorXd new_end_eig = end_orig->getPositions(post_speed_split_trajs[i].joint_names);
        std::vector<double> new_end_vec(new_end_eig.data(), new_end_eig.data() + new_end_eig.size());
        new_traj_point_end.positions = new_end_vec;
        new_raster_traj.points.push_back(new_traj_point_end);
      }
      else
      {
        RCLCPP_INFO(logger_, "ADDED NEW RASTER END");
        Eigen::VectorXd new_end_eig = end_new->getPositions(post_speed_split_trajs[i].joint_names);
        std::vector<double> new_end_vec(new_end_eig.data(), new_end_eig.data() + new_end_eig.size());
        new_traj_point_end.positions = new_end_vec;
        new_raster_traj.points.push_back(new_traj_point_end);
      }
      new_raster_traj.joint_names = post_speed_split_trajs[i].joint_names;
      new_raster_traj.header = post_speed_split_trajs[i].header;

      // Store new raster and trajectory in final vector to pass to trajopt
      if (ret_app_addable)
      {
        final_split_rasters.push_back(modified_raster);
        final_split_trajs.push_back(new_raster_traj);
      }

      // Update time parameterization if required
      if (config_->required_tool_vel && ret_app_addable)
      {
        std::vector<double> modified_time_steps = post_speed_split_time_steps[i];
        modified_time_steps.push_back(approach / config_->tool_speed);
        modified_time_steps.insert(
            modified_time_steps.end(), post_speed_split_time_steps[i].begin(), post_speed_split_time_steps[i].end());
        modified_time_steps.push_back(retreat / config_->tool_speed);
        final_time_steps.push_back(modified_time_steps);
      }
    }

    // Check approach/retreat for reachability
  }
  else
  {
    final_split_rasters = post_speed_split_rasters;
    final_split_trajs = post_speed_split_trajs;
    final_time_steps = post_speed_split_time_steps;
  }
  results->descartes_trajectory_results = final_split_trajs;

  // trajopt
  RCLCPP_INFO(logger_, "TIME TO OPTIMIZE");

  // Run trajectories through trajopt
  std::string target_frame = config_->tcp_frame;

  Eigen::VectorXd surface_coeffs(6);
  if (config_->trajopt_surface_config.surface_coeffs.size() == 0)
    surface_coeffs << 10, 10, 10, 10, 10, 10;
  else
    surface_coeffs = config_->trajopt_surface_config.surface_coeffs;
  RCLCPP_INFO(logger_, "BUILT CONFIG SETTINGS");

  std::vector<trajectory_msgs::msg::JointTrajectory> trajopt_trajectories;
  std::vector<bool> trajopt_solved;
  if (config_->use_trajopt_surface)
  {
    bool waypoints_critical = config_->trajopt_surface_config.waypoints_critical;
    for (size_t i = 0; i < final_split_rasters.size(); ++i)
    {
      std::vector<tesseract_motion_planners::Waypoint::Ptr> curr_raster;
      RCLCPP_INFO(logger_, "BUILDING WAYPOINT SET %i OF %i", i + 1, final_split_rasters.size());
      for (auto waypoint : final_split_rasters[i].poses)
      {
        Eigen::Vector3d surface_pose(waypoint.position.x, waypoint.position.y, waypoint.position.z);
        Eigen::Quaterniond surface_ori(
            waypoint.orientation.w, waypoint.orientation.x, waypoint.orientation.y, waypoint.orientation.z);
        tesseract_motion_planners::CartesianWaypoint::Ptr surface_waypoint =
            std::make_shared<tesseract_motion_planners::CartesianWaypoint>(surface_pose, surface_ori);
        surface_waypoint->setCoefficients(surface_coeffs);
        surface_waypoint->setIsCritical(waypoints_critical);
        curr_raster.push_back(std::move(surface_waypoint));
      }

      auto traj_pc = std::make_shared<tesseract_motion_planners::TrajOptPlannerDefaultConfig>(
          config_->tesseract_local, config_->manipulator, config_->tcp_frame, config_->tool_offset);

      traj_pc->optimizer = sco::ModelType::BPMPD;

      traj_pc->collision_cost_config = config_->trajopt_surface_config.coll_cst_cfg;
      traj_pc->collision_constraint_config = config_->trajopt_surface_config.coll_cnt_cfg;

      traj_pc->init_type = config_->trajopt_surface_config.init_type;
      traj_pc->longest_valid_segment_fraction = config_->trajopt_surface_config.longest_valid_segment_fraction;
      traj_pc->longest_valid_segment_length = config_->trajopt_surface_config.longest_valid_segment_length;

      traj_pc->smooth_velocities = config_->trajopt_surface_config.smooth_velocities;
      traj_pc->smooth_accelerations = config_->trajopt_surface_config.smooth_accelerations;
      traj_pc->smooth_jerks = config_->trajopt_surface_config.smooth_accelerations;
      traj_pc->target_waypoints = curr_raster;
      trajopt::SafetyMarginData::Ptr traj_smd_cost =
          std::make_shared<trajopt::SafetyMarginData>(config_->trajopt_surface_config.coll_cst_cfg.buffer_margin, 20);
      for (auto& spec_cost : config_->trajopt_surface_config.special_collision_cost)
      {
        traj_smd_cost->setPairSafetyMarginData(
            std::get<0>(spec_cost), std::get<1>(spec_cost), std::get<2>(spec_cost), std::get<3>(spec_cost));
      }
      trajopt::SafetyMarginData::Ptr traj_smd_cnt =
          std::make_shared<trajopt::SafetyMarginData>(config_->trajopt_surface_config.coll_cnt_cfg.safety_margin, 20);
      for (auto& spec_cnt : config_->trajopt_surface_config.special_collision_constraint)
      {
        traj_smd_cnt->setPairSafetyMarginData(
            std::get<0>(spec_cnt), std::get<1>(spec_cnt), std::get<2>(spec_cnt), std::get<3>(spec_cnt));
      }

      if (config_->trajopt_surface_config.special_collision_cost.size() > 0)
        traj_pc->special_collision_cost = traj_smd_cost;
      if (config_->trajopt_surface_config.special_collision_constraint.size() > 0)
        traj_pc->special_collision_constraint = traj_smd_cnt;

      Eigen::MatrixXd joint_eigen_from_jt;
      joint_eigen_from_jt = tesseract_rosutils::toEigen(results->descartes_trajectory_results[i],
                                                        results->descartes_trajectory_results[i].joint_names);

      traj_pc->seed_trajectory = joint_eigen_from_jt;

      trajectory_msgs::msg::JointTrajectory trajopt_result_traj;
      tesseract_motion_planners::PlannerResponse planner_resp;
      tesseract_motion_planners::TrajOptMotionPlanner traj_surface_planner;
      traj_surface_planner.setConfiguration(traj_pc);
      RCLCPP_INFO(logger_, "Solving raster: %i of %i", i + 1, final_split_rasters.size());
      traj_surface_planner.solve(planner_resp,
                                 tesseract_motion_planners::PostPlanCheckType::SINGLE_TIMESTEP_COLLISION,
                                 config_->trajopt_verbose_output);

      if (planner_resp.status.value() < 0)
      {
        RCLCPP_WARN(logger_, "FAILED: %s", planner_resp.status.message().c_str());
        if (config_->default_to_descartes)
        {
          trajopt_trajectories.push_back(final_split_trajs[i]);
          trajopt_solved.push_back(true);
        }
        else
        {
          trajopt_solved.push_back(false);
        }
        results->failed_rasters.push_back(final_split_rasters[i]);
      }
      else
      {
        RCLCPP_INFO(logger_, "SUCCEEDED");
        Eigen::MatrixXd result_traj(planner_resp.joint_trajectory.trajectory.rows(),
                                    planner_resp.joint_trajectory.trajectory.cols());
        result_traj << planner_resp.joint_trajectory.trajectory;
        tesseract_rosutils::toMsg(
            trajopt_result_traj, results->descartes_trajectory_results[i].joint_names, result_traj);
        results->solved_rasters.push_back(final_split_rasters[i]);
        trajopt_trajectories.push_back(std::move(trajopt_result_traj));
        trajopt_solved.push_back(true);
      }
    }
    if (trajopt_trajectories.empty())
    {
      RCLCPP_ERROR(logger_, "NO TRAJECTORIES WERE ABLE TO BE SOLVED");
      return false;
    }
  }
  else
  {
    results->failed_rasters = final_split_rasters;
    trajopt_trajectories = final_split_trajs;
    for (size_t i = 0; i < trajopt_trajectories.size(); ++i)
      trajopt_solved.push_back(true);
  }
  RCLCPP_INFO(logger_, "SETTING TIMESTAMPS");
  // Assign trajectory timestamps for motion execution
  std::vector<double> traj_times;
  size_t trajopt_traj_n = 0;
  std::vector<trajectory_msgs::msg::JointTrajectory> time_mod_traj;
  for (size_t i = 0; i < results->descartes_trajectory_results.size(); ++i)
  {
    if (trajopt_solved[i])
    {
      RCLCPP_INFO(logger_,
                  "Raster: %i of %i with %i waypoints",
                  trajopt_traj_n + 1,
                  trajopt_trajectories.size(),
                  trajopt_trajectories[trajopt_traj_n].points.size());
      trajopt_trajectories[trajopt_traj_n].header.frame_id = config_->world_frame;
      if (config_->required_tool_vel)
      {
        double curr_traj_time = 0;
        for (size_t j = 0; j < trajopt_trajectories[trajopt_traj_n].points.size(); ++j)
        {
          double added_time = 0;
          if (!config_->use_gazebo_sim_timing)
          {
            added_time = curr_traj_time;
          }

          trajopt_trajectories[trajopt_traj_n].points[j].time_from_start =
              rclcpp::Duration::from_seconds(final_time_steps[i][j] + added_time);
          curr_traj_time += final_time_steps[i][j];
        }

        if (config_->use_gazebo_sim_timing)
        {
          trajopt_trajectories[trajopt_traj_n].points.back().time_from_start.sec = 0;
        }
        traj_times.push_back(std::move(curr_traj_time));
        trajectory_msgs::msg::JointTrajectory curr_time_mod_traj;
        crs_motion_planning::timeParameterizeFreespace(
            trajopt_trajectories[trajopt_traj_n], config_->max_joint_vel, config_->max_joint_acc, curr_time_mod_traj);
        time_mod_traj.push_back(curr_time_mod_traj);
      }
      else
      {
        time_mod_traj.push_back(trajopt_trajectories[trajopt_traj_n]);
      }
      trajopt_traj_n++;
    }
  }

  RCLCPP_INFO(logger_, "ALL DONE");
  results->final_raster_trajectories = std::move(time_mod_traj);
  return true;
}

bool crsMotionPlanner::generateOMPLSeed(const tesseract_motion_planners::JointWaypoint::Ptr& start_pose,
                                        const tesseract_motion_planners::JointWaypoint::Ptr& end_pose,
                                        tesseract_common::JointTrajectory& seed_trajectory)
{
  tesseract_motion_planners::OMPLMotionPlanner ompl_planner;
  auto rrt_connect_configuration = std::make_shared<tesseract_motion_planners::RRTConnectConfigurator>();
  rrt_connect_configuration->range = config_->ompl_config.range;

  std::vector<tesseract_motion_planners::OMPLPlannerConfigurator::ConstPtr> ompl_configurators;
  ompl_configurators.insert(ompl_configurators.end(), config_->ompl_config.num_threads, rrt_connect_configuration);
  // Convert ompl_config to an actual ompl config file
  auto ompl_planner_config = std::make_shared<tesseract_motion_planners::OMPLPlannerFreespaceConfig>(
      config_->tesseract_local, config_->manipulator, ompl_configurators);
  tesseract_kinematics::ForwardKinematics::ConstPtr kin =
      config_->tesseract_local->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_->manipulator);
  ompl_planner_config->start_waypoint = start_pose;
  ompl_planner_config->end_waypoint = end_pose;
  ompl_planner_config->collision_safety_margin = config_->ompl_config.collision_safety_margin;
  ompl_planner_config->planning_time = config_->ompl_config.planning_time;
  ompl_planner_config->simplify = config_->ompl_config.simplify;
  ompl_planner_config->collision_continuous = config_->ompl_config.collision_continuous;
  ompl_planner_config->collision_check = config_->ompl_config.collision_check;
  ompl_planner_config->max_solutions = config_->ompl_config.max_solutions;
  ompl_planner_config->n_output_states = config_->ompl_config.n_output_states;
  ompl_planner_config->longest_valid_segment_fraction = config_->ompl_config.longest_valid_segment_fraction;
  ompl_planner_config->longest_valid_segment_length = config_->ompl_config.longest_valid_segment_length;
  RCLCPP_INFO(logger_, "OUTPUT STATES: %i", config_->ompl_config.n_output_states);
  std::cout << "GENERATING OMPL SEED FROM \n"
            << start_pose->getPositions().matrix() << "\nTO\n"
            << end_pose->getPositions().matrix() << std::endl;

  if (!ompl_planner.setConfiguration(ompl_planner_config))
  {
    return false;
  }
  RCLCPP_INFO(logger_, "OMPL CONFIGURATION SET");

  tesseract_motion_planners::PlannerResponse ompl_planner_response;
  tesseract_common::StatusCode status = ompl_planner.solve(ompl_planner_response);
  if (status.value() != tesseract_motion_planners::OMPLMotionPlannerStatusCategory::SolutionFound &&
      status.value() != tesseract_motion_planners::OMPLMotionPlannerStatusCategory::ErrorFoundValidSolutionInCollision)
  {
    RCLCPP_ERROR(logger_, "FAILED TO GENERATE OMPL SEED");
    return false;
  }

  RCLCPP_INFO(logger_, "OMPL SEED GENERATED");

  if (ompl_planner_response.joint_trajectory.trajectory.rows() < 5)
  {
    Eigen::MatrixXd new_traj(5, ompl_planner_response.joint_trajectory.joint_names.size());
    new_traj.row(0) = ompl_planner_response.joint_trajectory.trajectory.row(0);
    int rows_added = 1;
    for (int i = 0; i < 5 - ompl_planner_response.joint_trajectory.trajectory.rows(); ++i)
    {
      Eigen::VectorXd new_row;
      new_row = new_traj.row(i) * 0.5 + ompl_planner_response.joint_trajectory.trajectory.row(1) * 0.5;
      new_traj.row(i + 1) = new_row;
      rows_added++;
    }
    for (int i = rows_added; i < 5; ++i)
    {
      new_traj.row(i) = ompl_planner_response.joint_trajectory.trajectory.row(i - rows_added + 1);
    }
    ompl_planner_response.joint_trajectory.trajectory = new_traj;
  }
  seed_trajectory = ompl_planner_response.joint_trajectory;

  return true;
}

bool crsMotionPlanner::trajoptFreespaceFromOMPL(const tesseract_motion_planners::JointWaypoint::Ptr& start_pose,
                                                const tesseract_motion_planners::JointWaypoint::Ptr& end_pose,
                                                const tesseract_common::JointTrajectory& seed_trajectory,
                                                trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  RCLCPP_INFO(logger_, "SETTING UP TRAJOPT CONFIG");
  auto traj_pc = std::make_shared<tesseract_motion_planners::TrajOptPlannerFreespaceConfig>(
      config_->tesseract_local, config_->manipulator, config_->tcp_frame, config_->tool_offset);
  traj_pc->optimizer = sco::ModelType::BPMPD;
  traj_pc->smooth_velocities = config_->trajopt_freespace_config.smooth_velocities;
  traj_pc->smooth_accelerations = config_->trajopt_freespace_config.smooth_accelerations;
  traj_pc->smooth_jerks = config_->trajopt_freespace_config.smooth_jerks;
  traj_pc->collision_cost_config = config_->trajopt_freespace_config.coll_cst_cfg;
  traj_pc->collision_constraint_config = config_->trajopt_freespace_config.coll_cnt_cfg;
  traj_pc->init_type = config_->trajopt_freespace_config.init_type;
  traj_pc->contact_test_type = config_->trajopt_freespace_config.contact_test_type;
  traj_pc->longest_valid_segment_fraction = config_->trajopt_freespace_config.longest_valid_segment_fraction;
  traj_pc->longest_valid_segment_length = config_->trajopt_freespace_config.longest_valid_segment_length;
  trajopt::SafetyMarginData::Ptr traj_smd_cost =
      std::make_shared<trajopt::SafetyMarginData>(config_->trajopt_freespace_config.coll_cst_cfg.buffer_margin, 20);
  for (auto& spec_cost : config_->trajopt_freespace_config.special_collision_cost)
  {
    traj_smd_cost->setPairSafetyMarginData(
        std::get<0>(spec_cost), std::get<1>(spec_cost), std::get<2>(spec_cost), std::get<3>(spec_cost));
  }
  trajopt::SafetyMarginData::Ptr traj_smd_cnt =
      std::make_shared<trajopt::SafetyMarginData>(config_->trajopt_freespace_config.coll_cnt_cfg.safety_margin, 20);
  for (auto& spec_cnt : config_->trajopt_freespace_config.special_collision_constraint)
  {
    traj_smd_cnt->setPairSafetyMarginData(
        std::get<0>(spec_cnt), std::get<1>(spec_cnt), std::get<2>(spec_cnt), std::get<3>(spec_cnt));
  }

  if (config_->trajopt_freespace_config.special_collision_cost.size() > 0)
    traj_pc->special_collision_cost = traj_smd_cost;
  if (config_->trajopt_freespace_config.special_collision_constraint.size() > 0)
    traj_pc->special_collision_constraint = traj_smd_cnt;
  RCLCPP_INFO(logger_, "OPTIMIZING WITH TRAJOPT");
  std::vector<tesseract_motion_planners::Waypoint::Ptr> trgt_wypts;
  trgt_wypts.push_back(start_pose);
  trgt_wypts.push_back(end_pose);
  traj_pc->target_waypoints = trgt_wypts;
  traj_pc->seed_trajectory = seed_trajectory.trajectory;
  traj_pc->num_steps = seed_trajectory.trajectory.rows();
  tesseract_motion_planners::TrajOptMotionPlanner traj_motion_planner;
  tesseract_motion_planners::PlannerResponse plan_resp;
  traj_motion_planner.setConfiguration(traj_pc);
  traj_motion_planner.solve(plan_resp,
                            tesseract_motion_planners::PostPlanCheckType::DISCRETE_CONTINUOUS_COLLISION,
                            config_->trajopt_verbose_output);
  if (plan_resp.status.value() != 0)
  {
    RCLCPP_ERROR(logger_, "FAILED TO OPTIMIZE WITH TRAJOPT");
    RCLCPP_ERROR(logger_, "%s", plan_resp.status.message().c_str());
    return false;
  }
  RCLCPP_INFO(logger_, "OPTIMIZED WITH TRAJOPT");
  // Store generated trajectory
  tesseract_common::TrajArray traj_array = plan_resp.joint_trajectory.trajectory;
  Eigen::MatrixXd traj_cumulative_trajectory(traj_array.rows(), traj_array.cols());
  traj_cumulative_trajectory << traj_array;
  // Convert trajectory to ROSmsg
  tesseract_rosutils::toMsg(joint_trajectory, seed_trajectory.joint_names, traj_cumulative_trajectory);
  return true;
}

bool crsMotionPlanner::generateFreespacePlans(pathPlanningResults::Ptr& results)
{
  // Check if start exists
  // Generate ompl seed from start to raster.begin().begin() (convert from trajectory_msg to eigen::vectorXd)
  if (config_->use_start)
  {
    RCLCPP_INFO(logger_, "SETTING UP FIRST OMPL PROBLEM");
    auto begin_raster = std::make_shared<tesseract_motion_planners::JointWaypoint>(
        results->final_raster_trajectories[0].points[0].positions, results->final_raster_trajectories[0].joint_names);
    tesseract_common::JointTrajectory seed_trajectory;
    bool ompl_simplify = config_->ompl_config.simplify;
    config_->ompl_config.simplify = config_->simplify_start_end_freespace;
    Eigen::VectorXd start_pose = config_->start_pose->getPositions();
    if (!generateOMPLSeed(config_->start_pose, begin_raster, seed_trajectory))
    {
      results->msg_out = "Failed to generate ompl seed";
      RCLCPP_ERROR(logger_, "Failed to generate ompl seed");
      return false;
    }
    config_->ompl_config.simplify = ompl_simplify;
    trajectory_msgs::msg::JointTrajectory curr_joint_traj, ompl_joint_traj;
    Eigen::MatrixXd cumulative_trajectory(seed_trajectory.trajectory.rows(), seed_trajectory.trajectory.cols());
    cumulative_trajectory << seed_trajectory.trajectory;
    tesseract_rosutils::toMsg(ompl_joint_traj, seed_trajectory.joint_names, cumulative_trajectory);
    ompl_joint_traj.header.frame_id = config_->world_frame;
    if (config_->use_trajopt_freespace)
    {
      if (!trajoptFreespaceFromOMPL(config_->start_pose, begin_raster, seed_trajectory, curr_joint_traj))
      {
        results->msg_out = "Failed to perform trajopt on first freespace motion";
        RCLCPP_ERROR(logger_, "Failed to perform trajopt on first freespace motion");
        return false;
      }
    }
    else
    {
      curr_joint_traj = ompl_joint_traj;
    }

    curr_joint_traj.header.frame_id = config_->world_frame;
    // Modify time
    RCLCPP_INFO(logger_, "MODIFYING TRAJECTORY TIME OUTPUT");
    if (config_->use_gazebo_sim_timing)
    {
      for (int i = 0; i < curr_joint_traj.points.size(); ++i)
      {
        curr_joint_traj.points[static_cast<size_t>(i)].time_from_start.sec = 0;
        curr_joint_traj.points[static_cast<size_t>(i)].time_from_start.nanosec = 1e8;
      }
    }
    else
    {
      trajectory_msgs::msg::JointTrajectory new_traj;
      crs_motion_planning::timeParameterizeFreespace(
          curr_joint_traj, config_->max_joint_vel, config_->max_joint_acc, new_traj);
      curr_joint_traj = new_traj;
    }
    RCLCPP_INFO(logger_, "STORING TRAJECTORIES");
    results->ompl_start_end_trajectories.push_back(ompl_joint_traj);
    results->final_start_end_trajectories.push_back(curr_joint_traj);
    results->final_trajectories.push_back(curr_joint_traj);
  }
  RCLCPP_INFO(logger_, "STORING FIRST RASTER");
  results->final_trajectories.push_back(results->final_raster_trajectories[0]);
  // run seed through trajopt if using trajopt for freespace
  for (size_t i = 0; i < results->final_raster_trajectories.size() - 1; ++i)
  {
    RCLCPP_INFO(logger_, "SOLVING FREESPACE IN BETWEEN RASTERS %i AND %i", i, i + 1);
    // Generate ompl seed from raster[i].back() to raster[i+1].begin() (convert from trajectory_msg to eigen::vectorXd)
    // run seed through trajopt if using trajopt for freespace
    auto end_raster_i = std::make_shared<tesseract_motion_planners::JointWaypoint>(
        results->final_raster_trajectories[i].points.back().positions,
        results->final_raster_trajectories[i].joint_names);
    auto start_raster_ip1 = std::make_shared<tesseract_motion_planners::JointWaypoint>(
        results->final_raster_trajectories[i + 1].points[0].positions,
        results->final_raster_trajectories[i + 1].joint_names);
    tesseract_common::JointTrajectory seed_trajectory;
    if (!generateOMPLSeed(end_raster_i, start_raster_ip1, seed_trajectory))
    {
      results->msg_out = "Failed to generate ompl seed";
      RCLCPP_ERROR(logger_, "Failed to generate ompl seed between rasters %i and %i", i, i + 1);
      return false;
    }
    trajectory_msgs::msg::JointTrajectory curr_joint_traj, ompl_joint_traj;
    Eigen::MatrixXd cumulative_trajectory(seed_trajectory.trajectory.rows(), seed_trajectory.trajectory.cols());
    cumulative_trajectory << seed_trajectory.trajectory;
    tesseract_rosutils::toMsg(ompl_joint_traj, seed_trajectory.joint_names, cumulative_trajectory);
    ompl_joint_traj.header.frame_id = config_->world_frame;
    if (config_->use_trajopt_freespace)
    {
      if (!trajoptFreespaceFromOMPL(end_raster_i, start_raster_ip1, seed_trajectory, curr_joint_traj))
      {
        results->msg_out = "Failed to perform trajopt on a freespace motion";
        RCLCPP_ERROR(logger_, "Failed to perform trajopt on ompl seed between rasters %i and %i", i, i + 1);
        return false;
      }
    }
    else
    {
      curr_joint_traj = ompl_joint_traj;
    }

    curr_joint_traj.header.frame_id = config_->world_frame;
    // Modify time
    RCLCPP_INFO(logger_, "MODIFYING TRAJECTORY TIME OUTPUT");
    if (config_->use_gazebo_sim_timing)
    {
      for (int i = 0; i < curr_joint_traj.points.size(); ++i)
      {
        curr_joint_traj.points[static_cast<size_t>(i)].time_from_start.sec = 0;
        curr_joint_traj.points[static_cast<size_t>(i)].time_from_start.nanosec = 1e8;
      }
    }
    else
    {
      trajectory_msgs::msg::JointTrajectory new_traj;
      crs_motion_planning::timeParameterizeFreespace(
          curr_joint_traj, config_->max_joint_vel, config_->max_joint_acc, new_traj);
      curr_joint_traj = new_traj;
    }
    RCLCPP_INFO(logger_, "STORING FREESPACE JOINT TRAJECTORIES");
    results->ompl_trajectories.push_back(ompl_joint_traj);
    results->final_freespace_trajectories.push_back(curr_joint_traj);
    results->final_trajectories.push_back(curr_joint_traj);

    RCLCPP_INFO(logger_, "STORING RASTER, %i", i + 1);
    results->final_trajectories.push_back(results->final_raster_trajectories[i + 1]);
  }
  RCLCPP_INFO(logger_, "FINAL FREESPACE MOTION");

  // Check if end exists
  if (config_->use_end)
  {
    RCLCPP_INFO(logger_, "SETTING UP LAST OMPL PROBLEM");
    auto end_raster = std::make_shared<tesseract_motion_planners::JointWaypoint>(
        results->final_raster_trajectories.back().points.back().positions,
        results->final_raster_trajectories.back().joint_names);
    tesseract_common::JointTrajectory seed_trajectory;
    bool ompl_simplify = config_->ompl_config.simplify;
    config_->ompl_config.simplify = config_->simplify_start_end_freespace;
    if (!generateOMPLSeed(end_raster, config_->end_pose, seed_trajectory))
    {
      results->msg_out = "Failed to generate ompl seed";
      RCLCPP_ERROR(logger_, "Failed to generate ompl seed");
      return false;
    }
    config_->ompl_config.simplify = ompl_simplify;
    trajectory_msgs::msg::JointTrajectory curr_joint_traj, ompl_joint_traj;
    Eigen::MatrixXd cumulative_trajectory(seed_trajectory.trajectory.rows(), seed_trajectory.trajectory.cols());
    cumulative_trajectory << seed_trajectory.trajectory;
    tesseract_rosutils::toMsg(ompl_joint_traj, seed_trajectory.joint_names, cumulative_trajectory);
    ompl_joint_traj.header.frame_id = config_->world_frame;
    if (config_->use_trajopt_freespace)
    {
      if (!trajoptFreespaceFromOMPL(end_raster, config_->end_pose, seed_trajectory, curr_joint_traj))
      {
        results->msg_out = "Failed to perform trajopt on last freespace motion";
        RCLCPP_ERROR(logger_, "Failed to perform trajopt on last freespace motionT");
        return false;
      }
    }
    else
    {
      curr_joint_traj = ompl_joint_traj;
    }
    curr_joint_traj.header.frame_id = config_->world_frame;
    // Modify time
    RCLCPP_INFO(logger_, "MODIFYING TRAJECTORY TIME OUTPUT");
    if (config_->use_gazebo_sim_timing)
    {
      for (int i = 0; i < curr_joint_traj.points.size(); ++i)
      {
        curr_joint_traj.points[static_cast<size_t>(i)].time_from_start.sec = 0;
        curr_joint_traj.points[static_cast<size_t>(i)].time_from_start.nanosec = 1e8;
      }
    }
    else
    {
      trajectory_msgs::msg::JointTrajectory new_traj;
      crs_motion_planning::timeParameterizeFreespace(
          curr_joint_traj, config_->max_joint_vel, config_->max_joint_acc, new_traj);
      curr_joint_traj = new_traj;
    }
    RCLCPP_INFO(logger_, "STORING TRAJECTORIES");
    results->ompl_start_end_trajectories.push_back(ompl_joint_traj);
    results->final_start_end_trajectories.push_back(curr_joint_traj);
    results->final_trajectories.push_back(curr_joint_traj);
  }

  return true;
}

bool crsMotionPlanner::generateProcessPlan(pathPlanningResults::Ptr& results)
{
  RCLCPP_INFO(logger_, "generating surface plans");
  bool success_path = generateSurfacePlans(results);
  if (success_path)
  {
    return generateFreespacePlans(results);
  }
  else
  {
    return false;
  }
  return true;
}

bool crsMotionPlanner::generateFreespacePlan(const tesseract_motion_planners::JointWaypoint::Ptr& start_pose,
                                             const tesseract_motion_planners::JointWaypoint::Ptr& end_pose,
                                             trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  RCLCPP_INFO(logger_, "SETTING UP OMPL PROBLEM");
  tesseract_common::JointTrajectory seed_trajectory;
  if (!generateOMPLSeed(start_pose, end_pose, seed_trajectory))
  {
    return false;
  }
  trajectory_msgs::msg::JointTrajectory ompl_joint_traj;
  Eigen::MatrixXd cumulative_trajectory(seed_trajectory.trajectory.rows(), seed_trajectory.trajectory.cols());
  cumulative_trajectory << seed_trajectory.trajectory;
  tesseract_rosutils::toMsg(ompl_joint_traj, seed_trajectory.joint_names, cumulative_trajectory);
  ompl_joint_traj.header.frame_id = config_->world_frame;
  if (config_->use_trajopt_freespace)
  {
    if (!trajoptFreespaceFromOMPL(start_pose, end_pose, seed_trajectory, joint_trajectory))
    {
      return false;
    }
  }
  else
  {
    joint_trajectory = ompl_joint_traj;
  }
  joint_trajectory.header.frame_id = config_->world_frame;
  // Modify time
  if (config_->use_gazebo_sim_timing)
  {
    RCLCPP_INFO(logger_, "MODIFYING TRAJECTORY TIME OUTPUT");
    for (int i = 0; i < joint_trajectory.points.size(); ++i)
    {
      joint_trajectory.points[static_cast<size_t>(i)].time_from_start.sec = 0;
      joint_trajectory.points[static_cast<size_t>(i)].time_from_start.nanosec = 1e8;
    }
  }
  else
  {
    trajectory_msgs::msg::JointTrajectory new_traj;
    crs_motion_planning::timeParameterizeFreespace(
        joint_trajectory, config_->max_joint_vel, config_->max_joint_acc, new_traj);
    joint_trajectory = new_traj;
  }

  return true;
}

bool crsMotionPlanner::generateFreespacePlan(const tesseract_motion_planners::JointWaypoint::Ptr& start_pose,
                                             const tesseract_motion_planners::CartesianWaypoint::Ptr& end_pose,
                                             trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  tesseract_motion_planners::JointWaypoint::Ptr goal_waypoint;
  if (!findClosestJointOrientation(start_pose, end_pose, goal_waypoint))
  {
    RCLCPP_ERROR(logger_, "FAILED TO FIND A FEASIBLE IK SOLUTION");
    return false;
  }
  RCLCPP_INFO(logger_, "PLANNING OMPL VIA JOINT WAYPOINTS");
  if (!generateFreespacePlan(start_pose, goal_waypoint, joint_trajectory))
  {
    return false;
  }
  return true;
}

bool crsMotionPlanner::findClosestJointOrientation(const tesseract_motion_planners::JointWaypoint::Ptr& start_pose,
                                                   const tesseract_motion_planners::CartesianWaypoint::Ptr& end_pose,
                                                   tesseract_motion_planners::JointWaypoint::Ptr& returned_pose,
                                                   const double& axial_step)
{
  const bool allow_collisions = false;
  const double collision_safety_margin = config_->ompl_config.collision_safety_margin;
  tesseract::Tesseract::Ptr tesseract_local = config_->tesseract_local;
  const std::shared_ptr<const tesseract_environment::Environment> env = tesseract_local->getEnvironmentConst();
  tesseract_common::TransformMap curr_transforms = env->getCurrentState()->link_transforms;

  tesseract_kinematics::ForwardKinematics::ConstPtr kin =
      tesseract_local->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_->manipulator);

  tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), kin->getActiveLinkNames(), curr_transforms);

  auto collision_checker = std::make_shared<tesseract_motion_planners::DescartesCollisionD>(
      env, adjacency_map->getActiveLinkNames(), kin->getJointNames(), collision_safety_margin);

  Eigen::Isometry3d world_to_base_link, world_to_sander, world_to_tool0, tool0_to_sander;
  world_to_base_link = curr_transforms.find(config_->robot_base_frame)->second;
  world_to_sander = curr_transforms.find(config_->tcp_frame)->second;
  world_to_tool0 = curr_transforms.find(config_->tool0_frame)->second;
  tool0_to_sander = world_to_tool0.inverse() * world_to_sander;
  tool0_to_sander = tool0_to_sander * config_->descartes_config.tool_offset;
  descartes_light::KinematicsInterfaceD::Ptr kin_interface =
      std::make_shared<ur_ikfast_kinematics::UR10eKinematicsD>(world_to_base_link, tool0_to_sander, nullptr, nullptr);

  Eigen::Isometry3d goal_pose = end_pose->getTransform();
  std::vector<descartes_light::PositionSamplerD::Ptr> sampler_result;
  Eigen::VectorXd start_pose_eig = start_pose->getPositions(kin->getJointNames());
  std::vector<double> start_pose_vec(start_pose_eig.data(), start_pose_eig.data() + start_pose_eig.size());
  sampler_result.emplace_back(std::make_shared<descartes_light::FixedJointPoseSampler<double>>(start_pose_vec));

  if (axial_step < 0)
  {
    sampler_result.emplace_back(std::make_shared<descartes_light::CartesianPointSamplerD>(
        goal_pose,
        kin_interface,
        std::shared_ptr<descartes_light::CollisionInterface<double>>(collision_checker->clone()),
        allow_collisions));
  }
  else
  {
    sampler_result.emplace_back(std::make_shared<descartes_light::AxialSymmetricSamplerD>(
        goal_pose,
        kin_interface,
        axial_step,
        std::shared_ptr<descartes_light::CollisionInterface<double>>(collision_checker->clone()),
        allow_collisions));
  }
  auto edge_eval = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluatorD>(kin_interface->dof());
  auto timing_constraint =
      std::vector<descartes_core::TimingConstraintD>(sampler_result.size(), std::numeric_limits<double>::max());

  descartes_light::SolverD graph_builder(kin_interface->dof());

  if (!graph_builder.build(std::move(sampler_result), std::move(timing_constraint), std::move(edge_eval)))
  {
    return false;
  }

  std::vector<double> solution;
  if (!graph_builder.search(solution))
  {
    return false;
  }

  Eigen::Map<Eigen::VectorXd> solution_vec(&solution[0], solution.size());
  Eigen::VectorXd seed_traj(solution_vec.size());
  seed_traj << solution_vec;

  int n_rows = seed_traj.size() / kin_interface->dof();
  Eigen::MatrixXd joint_traj_eigen_out =
      Eigen::Map<Eigen::MatrixXd>(seed_traj.data(), kin_interface->dof(), n_rows).transpose();

  Eigen::VectorXd end_eig = joint_traj_eigen_out.row(1);
  returned_pose = std::make_shared<tesseract_motion_planners::JointWaypoint>(end_eig, kin->getJointNames());
  return true;
}

}  // namespace crs_motion_planning
