#include <crs_motion_planning/path_planning_utils.h>
#include <tesseract_rosutils/conversions.h>

namespace crs_motion_planning
{
crsMotionPlanner::crsMotionPlanner(pathPlanningConfig::Ptr config) : config_(std::move(config)) {}

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
  tesseract_common::TransformMap curr_transforms = env->getCurrentState()->transforms;

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
  descartes_light::KinematicsInterfaceD::Ptr kin_interface =
      std::make_shared<ur_ikfast_kinematics::UR10eKinematicsD>(world_to_base_link, tool0_to_sander, nullptr, nullptr);

  for (size_t i = 0; i < waypoints_pose_array.poses.size(); ++i)
  {
    Eigen::Isometry3d current_waypoint_pose;
    tf2::fromMsg(waypoints_pose_array.poses[i], current_waypoint_pose);
    sampler_result.emplace_back(std::make_shared<descartes_light::AxialSymmetricSamplerD>(
        current_waypoint_pose,
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

  crs_motion_planning::tesseractRosutilsToMsg(joint_trajectory, kin->getJointNames(), joint_traj_eigen_out);
  return true;
}

bool crsMotionPlanner::generateSurfacePlans(pathPlanningResults::Ptr& results)
{
  // Load rasters and get them in usable form
  std::vector<geometry_msgs::msg::PoseArray> raster_strips = config_->rasters;

  // Determine reachability of all rasters using descartes
  trajectory_msgs::msg::JointTrajectory joint_traj_msg_out_init, joint_traj_msg_out_final;
  std::cout << "RUNNING FIRST DESCARTES" << std::endl;

  bool gen_preplan;
  std::vector<geometry_msgs::msg::PoseArray> split_reachable_rasters;
  std::vector<trajectory_msgs::msg::JointTrajectory> split_traj;
  bool any_successes = false;
  size_t count_strips = 0;
  geometry_msgs::msg::PoseArray failed_vertex_poses;

  for (auto strip : raster_strips)
  {
    std::vector<size_t> failed_edges, failed_vertices;
    gen_preplan = generateDescartesSeed(strip, failed_edges, failed_vertices, joint_traj_msg_out_init);
    std::cout << "DONE" << std::endl;

    // Check if all rasters reachable
    if (!gen_preplan)
    {
      std::vector<geometry_msgs::msg::PoseArray> split_rasters;
      // Split up raster based on where planning failures occurred
      geometry_msgs::msg::PoseArray curr_failed_vertex_poses;
      std::cout << "CLEANING" << std::endl;
      crs_motion_planning::cleanRasterStrip(strip, failed_vertices, split_rasters, curr_failed_vertex_poses);

      // Display failed vertices
      results->unreachable_waypoints.poses.insert(results->unreachable_waypoints.poses.end(),
                                                  curr_failed_vertex_poses.poses.begin(),
                                                  curr_failed_vertex_poses.poses.end());

      for (auto split_strip : split_rasters)
      {
        // Generate Descartes preplan
        if (split_strip.poses.size() >= config_->minimum_raster_length)
        {
          geometry_msgs::msg::PoseArray split_strip_ar;

          if (config_->add_approach_and_retreat)
          {
            crs_motion_planning::addApproachAndRetreat(
                split_strip, config_->approach_distance, config_->retreat_distance, split_strip_ar);
          }
          else
          {
            split_strip_ar = split_strip;
          }
          std::cout << "DESCARTES ROUND 2" << std::endl;
          results->reachable_waypoints.poses.insert(
              results->reachable_waypoints.poses.end(), split_strip.poses.begin(), split_strip.poses.end());
          if (generateDescartesSeed(split_strip_ar, failed_edges, failed_vertices, joint_traj_msg_out_final))
          {
            split_traj.push_back(joint_traj_msg_out_final);
            split_reachable_rasters.push_back(std::move(split_strip_ar));
            any_successes = true;
            std::cout << "SUCCESS" << std::endl;
          }
          else
          {
            results->unreachable_waypoints.poses.insert(
                results->unreachable_waypoints.poses.end(), split_strip_ar.poses.begin(), split_strip_ar.poses.end());
            std::cout << "ADDED APPROACH AND/OR RETREAT MADE RASTER UNREACHABLE" << std::endl;
          }
        }
        else
        {
          results->skipped_rasters.push_back(split_strip);
        }
        std::cout << "DONE" << std::endl;
      }
    }
    else
    {
      std::cout << "SUCCESS" << std::endl;
      geometry_msgs::msg::PoseArray split_strip_ar;
      if (config_->add_approach_and_retreat)
      {
        crs_motion_planning::addApproachAndRetreat(
            strip, config_->approach_distance, config_->retreat_distance, split_strip_ar);
      }
      else
      {
        split_strip_ar = strip;
      }
      results->reachable_waypoints.poses.insert(
          results->reachable_waypoints.poses.end(), strip.poses.begin(), strip.poses.end());
      std::cout << "DESCARTES ROUND 2" << std::endl;
      if (generateDescartesSeed(split_strip_ar, failed_edges, failed_vertices, joint_traj_msg_out_final))
      {
        split_traj.push_back(joint_traj_msg_out_final);
        split_reachable_rasters.push_back(std::move(split_strip_ar));
        any_successes = true;
        std::cout << "SUCCESS" << std::endl;
      }
      else
      {
        results->unreachable_waypoints.poses.insert(
            results->unreachable_waypoints.poses.end(), split_strip_ar.poses.begin(), split_strip_ar.poses.end());
        std::cout << "ADDED APPROACH AND/OR RETREAT MADE RASTER UNREACHABLE" << std::endl;
      }
    }
    std::cout << "Strip " << ++count_strips << " of " << raster_strips.size() << std::endl;
  }

  // Check if successfully generated preplan with descartes
  if (any_successes)
  {
    std::vector<geometry_msgs::msg::PoseArray> final_split_rasters;
    std::vector<std::vector<double>> final_time_steps;
    size_t raster_n = 0;
    std::cout << "CHECKING FOR SPLITS IN " << split_traj.size() << " TRAJECTORIES" << std::endl;
    for (auto curr_joint_traj : split_traj)
    {
      // Split rasters based on signigicant joint motions
      std::vector<trajectory_msgs::msg::JointTrajectory> double_split_traj;
      std::vector<geometry_msgs::msg::PoseArray> resplit_rasters;
      double desired_ee_val = config_->tool_speed;
      double max_joint_vel = config_->max_joint_vel;
      std::vector<std::vector<double>> time_steps;
      if (config_->required_tool_vel && crs_motion_planning::splitRastersByJointDist(curr_joint_traj,
                                                                                     split_reachable_rasters[raster_n],
                                                                                     desired_ee_val,
                                                                                     max_joint_vel,
                                                                                     double_split_traj,
                                                                                     resplit_rasters,
                                                                                     time_steps))
      {
        std::cout << "FOUND A SPLIT" << std::endl;
        if (config_->add_approach_and_retreat)
        {
          for (size_t i = 0; i < resplit_rasters.size(); ++i)
          {
            if (resplit_rasters[i].poses.size() >= config_->minimum_raster_length)
            {
              geometry_msgs::msg::PoseArray modified_raster;
              double approach = config_->approach_distance;
              double retreat = config_->retreat_distance;
              std::vector<double> modified_time_steps = time_steps[i];
              if (i == 0)
              {
                // Add departure to end of resplit_rasters[i]
                crs_motion_planning::addApproachAndRetreat(resplit_rasters[i], 0, retreat, modified_raster);
                modified_time_steps = time_steps[i];
                modified_time_steps.push_back(retreat * desired_ee_val);
              }
              else if (i == double_split_traj.size() - 1)
              {
                // Add entrance to end of resplit_rasters[i]
                crs_motion_planning::addApproachAndRetreat(resplit_rasters[i], approach, 0, modified_raster);
                modified_time_steps.push_back(approach * desired_ee_val);
                modified_time_steps.insert(modified_time_steps.end(), time_steps[i].begin(), time_steps[i].end());
              }
              else
              {
                // Add entrance and departure to resplit_rasters[i]
                crs_motion_planning::addApproachAndRetreat(resplit_rasters[i], approach, retreat, modified_raster);
                modified_time_steps.push_back(approach * desired_ee_val);
                modified_time_steps.insert(modified_time_steps.end(), time_steps[i].begin(), time_steps[i].end());
                modified_time_steps.push_back(retreat * desired_ee_val);
              }

              trajectory_msgs::msg::JointTrajectory modified_joint_traj_msg_out;
              // Generate Descartes preplan
              std::cout << "DESCARTES ROUND 3" << std::endl;
              std::vector<size_t> failed_edges, failed_vertices;

              if (generateDescartesSeed(modified_raster, failed_edges, failed_vertices, modified_joint_traj_msg_out))
              {
                std::cout << "SUCCESS" << std::endl;
                final_split_rasters.push_back(modified_raster);
                results->descartes_trajectory_results.push_back(modified_joint_traj_msg_out);
                final_time_steps.push_back(modified_time_steps);
              }
              std::cout << "DONE" << std::endl;
            }
            else
            {
              results->skipped_rasters.push_back(resplit_rasters[i]);
            }
          }
        }
      }
      else
      {
        std::cout << "NO SPLITS" << std::endl;
        final_split_rasters.push_back(split_reachable_rasters[raster_n]);
        results->descartes_trajectory_results.push_back(std::move(curr_joint_traj));
        if (config_->required_tool_vel)
        {
          final_time_steps.push_back(time_steps[0]);
        }
      }
      raster_n++;
    }

    std::cout << "TIME TO OPTIMIZE" << std::endl;

    // Run trajectories through trajopt
    std::string target_frame = config_->tcp_frame;

    Eigen::VectorXd surface_coeffs(6);
    if (config_->trajopt_surface_config.surface_coeffs.size() == 0)
      surface_coeffs << 10, 10, 10, 10, 10, 10;
    else
      surface_coeffs = config_->trajopt_surface_config.surface_coeffs;
    std::cout << "BUILT CONFIG SETTINGS" << std::endl;

    std::vector<trajectory_msgs::msg::JointTrajectory> trajopt_trajectories;
    std::vector<bool> trajopt_solved;
    bool waypoints_critical = config_->trajopt_surface_config.waypoints_critical;
    for (size_t i = 0; i < final_split_rasters.size(); ++i)
    {
      std::vector<tesseract_motion_planners::Waypoint::Ptr> curr_raster;
      std::cout << "BUILDING WAYPOINT SET " << i + 1 << " OF " << final_split_rasters.size() << std::endl;
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

      traj_pc->smooth_velocities = config_->trajopt_surface_config.smooth_velocities;
      traj_pc->smooth_accelerations = config_->trajopt_surface_config.smooth_accelerations;
      traj_pc->smooth_jerks = config_->trajopt_surface_config.smooth_accelerations;
      traj_pc->target_waypoints = curr_raster;

      Eigen::MatrixXd joint_eigen_from_jt;
      joint_eigen_from_jt = tesseract_rosutils::toEigen(results->descartes_trajectory_results[i],
                                                        results->descartes_trajectory_results[i].joint_names);

      traj_pc->seed_trajectory = joint_eigen_from_jt;

      trajectory_msgs::msg::JointTrajectory trajopt_result_traj;
      tesseract_motion_planners::PlannerResponse planner_resp;
      tesseract_motion_planners::TrajOptMotionPlanner traj_surface_planner;
      traj_surface_planner.setConfiguration(traj_pc);
      std::cout << "Solving raster: " << i + 1 << " of " << final_split_rasters.size() << std::endl;
      traj_surface_planner.solve(planner_resp, config_->trajopt_verbose_output);

      if (planner_resp.status.value() < 0)
      {
        std::cout << "FAILED: " << planner_resp.status.message() << std::endl;
        results->failed_rasters.push_back(final_split_rasters[i]);
        trajopt_solved.push_back(false);
      }
      else
      {
        std::cout << "SUCCEEDED" << std::endl;
        Eigen::MatrixXd result_traj(planner_resp.joint_trajectory.trajectory.rows(),
                                    planner_resp.joint_trajectory.trajectory.cols());
        result_traj << planner_resp.joint_trajectory.trajectory;
        crs_motion_planning::tesseractRosutilsToMsg(
            trajopt_result_traj, results->descartes_trajectory_results[i].joint_names, result_traj);
        results->solved_rasters.push_back(final_split_rasters[i]);
        trajopt_trajectories.push_back(std::move(trajopt_result_traj));
        trajopt_solved.push_back(true);
      }
    }
    std::cout << "SETTING TIMESTAMPS" << std::endl;
    // Assign trajectory timestamps for motion execution
    std::vector<double> traj_times;
    size_t trajopt_traj_n = 0;
    for (size_t i = 0; i < results->descartes_trajectory_results.size(); ++i)
    {
      if (trajopt_solved[i])
      {
        std::cout << "Raster: " << trajopt_traj_n + 1 << " of " << trajopt_trajectories.size() << " with "
                  << trajopt_trajectories[trajopt_traj_n].points.size() << " waypoints " << std::endl;
        trajopt_trajectories[trajopt_traj_n].header.frame_id = config_->world_frame;
        if (config_->required_tool_vel)
        {
          double curr_traj_time = 0;
          for (size_t j = 1; j < trajopt_trajectories[trajopt_traj_n].points.size(); ++j)
          {
            double added_time = 0;
            if (!config_->use_gazebo_sim_timing)
            {
              added_time = curr_traj_time;
            }
            trajopt_trajectories[trajopt_traj_n].points[j - 1].time_from_start.sec =
                static_cast<int>(floor(final_time_steps[i][j] + added_time));
            trajopt_trajectories[trajopt_traj_n].points[j - 1].time_from_start.nanosec = static_cast<uint>(
                1e9 * (final_time_steps[i][j] + added_time - floor(final_time_steps[i][j] + added_time)));
            curr_traj_time += final_time_steps[i][j];
          }
          trajopt_trajectories[trajopt_traj_n].points.back().time_from_start.sec = 0;
          traj_times.push_back(std::move(curr_traj_time));
          trajopt_traj_n++;
        }
      }
    }

    std::cout << "ALL DONE" << std::endl;
    results->final_raster_trajectories = std::move(trajopt_trajectories);
    return true;
  }
  else
  {
    results->msg_out = "Failed to create surface plans";
    return false;
  }
}

bool crsMotionPlanner::generateOMPLSeed(const tesseract_motion_planners::JointWaypoint::Ptr& start_pose,
                                        const tesseract_motion_planners::JointWaypoint::Ptr& end_pose,
                                        tesseract_common::JointTrajectory& seed_trajectory)
{
  tesseract_motion_planners::OMPLMotionPlanner<ompl::geometric::RRTConnect> ompl_planner;

  // Convert ompl_config to an actual ompl config file
  auto ompl_planner_config =
      std::make_shared<tesseract_motion_planners::OMPLPlannerFreespaceConfig<ompl::geometric::RRTConnect>>(
          config_->tesseract_local, config_->manipulator);
  tesseract_kinematics::ForwardKinematics::ConstPtr kin =
      config_->tesseract_local->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_->manipulator);
  ompl_planner_config->start_waypoint = start_pose;
  ompl_planner_config->end_waypoint = end_pose;
  ompl_planner_config->collision_safety_margin = config_->ompl_config.collision_safety_margin;
  ompl_planner_config->planning_time = config_->ompl_config.planning_time;
  ompl_planner_config->simplify = config_->ompl_config.simplify;
  ompl_planner_config->collision_continuous = config_->ompl_config.collision_continuous;
  ompl_planner_config->collision_check = config_->ompl_config.collision_check;
  ompl_planner_config->settings.range = config_->ompl_config.range;
  ompl_planner_config->num_threads = config_->ompl_config.num_threads;
  ompl_planner_config->max_solutions = config_->ompl_config.max_solutions;
  ompl_planner_config->n_output_states = config_->ompl_config.n_output_states;
  std::cout << "OUTPUT STATES: " << config_->ompl_config.n_output_states << std::endl;
  std::cout << "GENERATING OMPL SEED FROM \n"
            << start_pose->getPositions().matrix() << "\nTO\n"
            << end_pose->getPositions().matrix() << std::endl;

  // Solve
  if (!ompl_planner.setConfiguration(ompl_planner_config))
  {
    return false;
  }

  tesseract_motion_planners::PlannerResponse ompl_planner_response;
  tesseract_common::StatusCode status = ompl_planner.solve(ompl_planner_response);
  if (status.value() != tesseract_motion_planners::OMPLFreespacePlannerStatusCategory::SolutionFound &&
      status.value() !=
          tesseract_motion_planners::OMPLFreespacePlannerStatusCategory::ErrorFoundValidSolutionInCollision)
  {
    std::cout << "FAILED TO GENERATE OMPL SEED" << std::endl;
    return false;
  }

  std::cout << "OMPL SEED GENERATED" << std::endl;
  seed_trajectory = ompl_planner_response.joint_trajectory;

  return true;
}

bool crsMotionPlanner::trajoptFreespaceFromOMPL(const tesseract_motion_planners::JointWaypoint::Ptr& start_pose,
                                                const tesseract_motion_planners::JointWaypoint::Ptr& end_pose,
                                                const tesseract_common::JointTrajectory& seed_trajectory,
                                                trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  std::cout << "SETTING UP TRAJOPT CONFIG" << std::endl;
  auto traj_pc = std::make_shared<tesseract_motion_planners::TrajOptPlannerFreespaceConfig>(
      config_->tesseract_local, config_->manipulator, config_->tcp_frame, config_->tool_offset);
  if (config_->use_trajopt_freespace)
  {
    traj_pc->optimizer = sco::ModelType::BPMPD;
    traj_pc->smooth_velocities = config_->trajopt_freespace_config.smooth_velocities;
    traj_pc->smooth_accelerations = config_->trajopt_freespace_config.smooth_accelerations;
    traj_pc->smooth_jerks = config_->trajopt_freespace_config.smooth_jerks;
    traj_pc->collision_cost_config = config_->trajopt_freespace_config.coll_cst_cfg;
    traj_pc->collision_constraint_config = config_->trajopt_freespace_config.coll_cnt_cfg;
    traj_pc->init_type = config_->trajopt_freespace_config.init_type;
    traj_pc->contact_test_type = config_->trajopt_freespace_config.contact_test_type;
  }
  std::cout << "OPTIMIZING WITH TRAJOPT" << std::endl;
  std::vector<tesseract_motion_planners::Waypoint::Ptr> trgt_wypts;
  trgt_wypts.push_back(start_pose);
  trgt_wypts.push_back(end_pose);
  traj_pc->target_waypoints = trgt_wypts;
  traj_pc->seed_trajectory = seed_trajectory.trajectory;
  traj_pc->num_steps = seed_trajectory.trajectory.rows();
  tesseract_motion_planners::TrajOptMotionPlanner traj_motion_planner;
  tesseract_motion_planners::PlannerResponse plan_resp;
  traj_motion_planner.setConfiguration(traj_pc);
  traj_motion_planner.solve(plan_resp, false);
  if (plan_resp.status.value() != 0)
  {
    std::cout << "FAILED TO OPTIMIZE WITH TRAJOPT" << std::endl;
    std::cout << plan_resp.status.message() << std::endl;
    return false;
  }
  std::cout << "OPTIMIZED WITH TRAJOPT" << std::endl;
  // Store generated trajectory
  tesseract_common::TrajArray traj_array = plan_resp.joint_trajectory.trajectory;
  Eigen::MatrixXd traj_cumulative_trajectory(traj_array.rows(), traj_array.cols());
  traj_cumulative_trajectory << traj_array;
  // Convert trajectory to ROSmsg
  crs_motion_planning::tesseractRosutilsToMsg(
      joint_trajectory, seed_trajectory.joint_names, traj_cumulative_trajectory);
  return true;
}

bool crsMotionPlanner::generateFreespacePlans(pathPlanningResults::Ptr& results)
{
  // Check if start exists
  // Generate ompl seed from start to raster.begin().begin() (convert from trajectory_msg to eigen::vectorXd)
  if (config_->use_start)
  {
    std::cout << "SETTING UP FIRST OMPL PROBLEM" << std::endl;
    auto begin_raster = std::make_shared<tesseract_motion_planners::JointWaypoint>(
        results->final_raster_trajectories[0].points[0].positions, results->final_raster_trajectories[0].joint_names);
    tesseract_common::JointTrajectory seed_trajectory;
    bool ompl_simplify = config_->ompl_config.simplify;
    config_->ompl_config.simplify = config_->simplify_start_end_freespace;
    Eigen::VectorXd start_pose = config_->start_pose->getPositions();
    if (!generateOMPLSeed(config_->start_pose, begin_raster, seed_trajectory))
    {
      results->msg_out = "Failed to generate ompl seed";
      return false;
    }
    config_->ompl_config.simplify = ompl_simplify;
    trajectory_msgs::msg::JointTrajectory curr_joint_traj, ompl_joint_traj;
    Eigen::MatrixXd cumulative_trajectory(seed_trajectory.trajectory.rows(), seed_trajectory.trajectory.cols());
    cumulative_trajectory << seed_trajectory.trajectory;
    crs_motion_planning::tesseractRosutilsToMsg(ompl_joint_traj, seed_trajectory.joint_names, cumulative_trajectory);
    ompl_joint_traj.header.frame_id = config_->world_frame;
    if (config_->use_trajopt_freespace)
    {
      if (!trajoptFreespaceFromOMPL(config_->start_pose, begin_raster, seed_trajectory, curr_joint_traj))
      {
        results->msg_out = "Failed to perform trajopt on first freespace motion";
        return false;
      }

      curr_joint_traj.header.frame_id = config_->world_frame;
      // Modify time
      if (config_->use_gazebo_sim_timing)
      {
        std::cout << "MODIFYING TRAJECTORY TIME OUTPUT" << std::endl;
        for (int i = 0; i < curr_joint_traj.points.size(); ++i)
        {
          curr_joint_traj.points[static_cast<size_t>(i)].time_from_start.sec = 0;
          curr_joint_traj.points[static_cast<size_t>(i)].time_from_start.nanosec = 1e8;
        }
      }
    }
    else
    {
      curr_joint_traj = ompl_joint_traj;
    }
    std::cout << "STORING TRAJECTORIES" << std::endl;
    results->ompl_start_end_trajectories.push_back(ompl_joint_traj);
    results->final_start_end_trajectories.push_back(curr_joint_traj);
    results->final_trajectories.push_back(curr_joint_traj);
  }
  std::cout << "STORING FIRST RASTER" << std::endl;
  results->final_trajectories.push_back(results->final_raster_trajectories[0]);
  // run seed through trajopt if using trajopt for freespace
  for (size_t i = 0; i < results->final_raster_trajectories.size() - 1; ++i)
  {
    std::cout << "SOLVING FREESPACE IN BETWEEN RASTERS " << i << " AND " << i + 1 << std::endl;
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
      return false;
    }
    trajectory_msgs::msg::JointTrajectory curr_joint_traj, ompl_joint_traj;
    Eigen::MatrixXd cumulative_trajectory(seed_trajectory.trajectory.rows(), seed_trajectory.trajectory.cols());
    cumulative_trajectory << seed_trajectory.trajectory;
    crs_motion_planning::tesseractRosutilsToMsg(ompl_joint_traj, seed_trajectory.joint_names, cumulative_trajectory);
    ompl_joint_traj.header.frame_id = config_->world_frame;
    if (config_->use_trajopt_freespace)
    {
      if (!trajoptFreespaceFromOMPL(end_raster_i, start_raster_ip1, seed_trajectory, curr_joint_traj))
      {
        results->msg_out = "Failed to perform trajopt on first freespace motion";
        return false;
      }

      curr_joint_traj.header.frame_id = config_->world_frame;
      // Modify time
      if (config_->use_gazebo_sim_timing)
      {
        std::cout << "MODIFYING TRAJECTORY TIME OUTPUT" << std::endl;
        for (int i = 0; i < curr_joint_traj.points.size(); ++i)
        {
          curr_joint_traj.points[static_cast<size_t>(i)].time_from_start.sec = 0;
          curr_joint_traj.points[static_cast<size_t>(i)].time_from_start.nanosec = 1e8;
        }
      }
    }
    else
    {
      curr_joint_traj = ompl_joint_traj;
    }
    std::cout << "STORING FREESPACE JOINT TRAJECTORIES" << std::endl;
    results->ompl_trajectories.push_back(ompl_joint_traj);
    results->final_freespace_trajectories.push_back(curr_joint_traj);
    results->final_trajectories.push_back(curr_joint_traj);

    std::cout << "STORING RASTER " << i + 1 << std::endl;
    results->final_trajectories.push_back(results->final_raster_trajectories[i + 1]);
  }
  std::cout << "FINAL FREESPACE MOTION" << std::endl;

  // Check if end exists
  if (config_->use_end)
  {
    std::cout << "SETTING UP LAST OMPL PROBLEM" << std::endl;
    auto end_raster = std::make_shared<tesseract_motion_planners::JointWaypoint>(
        results->final_raster_trajectories.back().points.back().positions,
        results->final_raster_trajectories.back().joint_names);
    tesseract_common::JointTrajectory seed_trajectory;
    bool ompl_simplify = config_->ompl_config.simplify;
    config_->ompl_config.simplify = config_->simplify_start_end_freespace;
    if (!generateOMPLSeed(end_raster, config_->end_pose, seed_trajectory))
    {
      results->msg_out = "Failed to generate ompl seed";
      return false;
    }
    config_->ompl_config.simplify = ompl_simplify;
    trajectory_msgs::msg::JointTrajectory curr_joint_traj, ompl_joint_traj;
    Eigen::MatrixXd cumulative_trajectory(seed_trajectory.trajectory.rows(), seed_trajectory.trajectory.cols());
    cumulative_trajectory << seed_trajectory.trajectory;
    crs_motion_planning::tesseractRosutilsToMsg(ompl_joint_traj, seed_trajectory.joint_names, cumulative_trajectory);
    ompl_joint_traj.header.frame_id = config_->world_frame;
    if (config_->use_trajopt_freespace)
    {
      if (!trajoptFreespaceFromOMPL(end_raster, config_->end_pose, seed_trajectory, curr_joint_traj))
      {
        results->msg_out = "Failed to perform trajopt on first freespace motion";
        return false;
      }

      curr_joint_traj.header.frame_id = config_->world_frame;
      // Modify time
      if (config_->use_gazebo_sim_timing)
      {
        std::cout << "MODIFYING TRAJECTORY TIME OUTPUT" << std::endl;
        for (int i = 0; i < curr_joint_traj.points.size(); ++i)
        {
          curr_joint_traj.points[static_cast<size_t>(i)].time_from_start.sec = 0;
          curr_joint_traj.points[static_cast<size_t>(i)].time_from_start.nanosec = 1e8;
        }
      }
    }
    else
    {
      curr_joint_traj = ompl_joint_traj;
    }
    std::cout << "STORING TRAJECTORIES" << std::endl;
    results->ompl_start_end_trajectories.push_back(ompl_joint_traj);
    results->final_start_end_trajectories.push_back(curr_joint_traj);
    results->final_trajectories.push_back(curr_joint_traj);
  }

  return true;
}

bool crsMotionPlanner::generateProcessPlan(pathPlanningResults::Ptr& results)
{
  std::cout << "generating surface plans" << std::endl;
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
  std::cout << "SETTING UP OMPL PROBLEM" << std::endl;
  tesseract_common::JointTrajectory seed_trajectory;
  if (!generateOMPLSeed(start_pose, end_pose, seed_trajectory))
  {
    return false;
  }
  trajectory_msgs::msg::JointTrajectory ompl_joint_traj;
  Eigen::MatrixXd cumulative_trajectory(seed_trajectory.trajectory.rows(), seed_trajectory.trajectory.cols());
  cumulative_trajectory << seed_trajectory.trajectory;
  crs_motion_planning::tesseractRosutilsToMsg(ompl_joint_traj, seed_trajectory.joint_names, cumulative_trajectory);
  ompl_joint_traj.header.frame_id = config_->world_frame;
  if (config_->use_trajopt_freespace)
  {
    if (!trajoptFreespaceFromOMPL(start_pose, end_pose, seed_trajectory, joint_trajectory))
    {
      return false;
    }

    joint_trajectory.header.frame_id = config_->world_frame;
    // Modify time
    if (config_->use_gazebo_sim_timing)
    {
      std::cout << "MODIFYING TRAJECTORY TIME OUTPUT" << std::endl;
      for (int i = 0; i < joint_trajectory.points.size(); ++i)
      {
        joint_trajectory.points[static_cast<size_t>(i)].time_from_start.sec = 0;
        joint_trajectory.points[static_cast<size_t>(i)].time_from_start.nanosec = 1e8;
      }
    }
  }
  else
  {
    joint_trajectory = ompl_joint_traj;
  }

  return true;
}

bool crsMotionPlanner::generateFreespacePlan(const tesseract_motion_planners::JointWaypoint::Ptr& start_pose,
                                             const tesseract_motion_planners::CartesianWaypoint::Ptr& end_pose,
                                             trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  const bool allow_collisions = false;
  const double collision_safety_margin = config_->ompl_config.collision_safety_margin;
  tesseract::Tesseract::Ptr tesseract_local = config_->tesseract_local;
  const std::shared_ptr<const tesseract_environment::Environment> env = tesseract_local->getEnvironmentConst();
  tesseract_common::TransformMap curr_transforms = env->getCurrentState()->transforms;

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
  descartes_light::KinematicsInterfaceD::Ptr kin_interface =
      std::make_shared<ur_ikfast_kinematics::UR10eKinematicsD>(world_to_base_link, tool0_to_sander, nullptr, nullptr);

  Eigen::Isometry3d goal_pose = end_pose->getTransform();
  descartes_light::PositionSamplerD::Ptr sampler_result;

  sampler_result = std::make_shared<descartes_light::CartesianPointSamplerD>(
      goal_pose,
      kin_interface,
      std::shared_ptr<descartes_light::CollisionInterface<double>>(collision_checker->clone()),
      allow_collisions);

  std::vector<double> soln_set;
  sampler_result->sample(soln_set);
  size_t j_num = start_pose->getNames().size();
  size_t num_sols = soln_set.size() / j_num;
  std::cout << "FOUND " << num_sols << " viable positions" << std::endl;
  if (num_sols < 1)
  {
    return false;
  }
  Eigen::VectorXd start_eig = start_pose->getPositions(kin->getJointNames());
  double min_j_dist = 999999;
  Eigen::VectorXd end_eig = start_eig;
  for (size_t i = 0; i < num_sols; ++i)
  {
    Eigen::VectorXd curr_sol(6), start_to_end(6);
    curr_sol << soln_set[0 + i * j_num], soln_set[1 + i * j_num], soln_set[2 + i * j_num], soln_set[3 + i * j_num],
        soln_set[4 + i * j_num], soln_set[5 + i * j_num];
    start_to_end = curr_sol - start_eig;
    double j_dist = start_to_end.norm();
    if (j_dist <= min_j_dist)
    {
      std::cout << "FOUND A BETTER JOINT CONFIG" << std::endl;
      min_j_dist = j_dist;
      end_eig = curr_sol;
    }
  }
  tesseract_motion_planners::JointWaypoint::Ptr goal_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(end_eig, kin->getJointNames());

  std::cout << "PLANNING OMPL VIA JOINT WAYPOINTS" << std::endl;
  bool success = generateFreespacePlan(start_pose, goal_waypoint, joint_trajectory);

  //    std::cout << "THIS IS NOT YET IMPLEMENTED" << std::endl;
  return success;
}

}  // namespace crs_motion_planning
