#include <crs_motion_planning/path_planning_utils.h>
#include <tesseract_rosutils/conversions.h>

namespace crs_motion_planning
{

crsMotionPlanner::crsMotionPlanner(pathPlanningConfig::Ptr config) : config_(std::move(config)) {}

void crsMotionPlanner::updateConfiguration(pathPlanningConfig::Ptr config)
{
    config_ = std::move(config);
}

bool crsMotionPlanner::generateDescartesSeed(const geometry_msgs::msg::PoseArray &waypoints_pose_array,
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

    tesseract_kinematics::ForwardKinematics::ConstPtr kin = tesseract_local->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_->manipulator);

    tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(env->getSceneGraph(), kin->getActiveLinkNames(), curr_transforms);

    auto collision_checker = std::make_shared<tesseract_motion_planners::DescartesCollisionD>(env, adjacency_map->getActiveLinkNames(), kin->getJointNames(), collision_safety_margin);

    std::vector<descartes_light::PositionSamplerD::Ptr> sampler_result;

    Eigen::Isometry3d world_to_base_link, world_to_sander, world_to_tool0, tool0_to_sander;
    world_to_base_link = curr_transforms.find(config_->robot_base_frame)->second;
    world_to_sander = curr_transforms.find(config_->tcp_frame)->second;
    world_to_tool0 = curr_transforms.find(config_->tool0_frame)->second;
    tool0_to_sander = world_to_tool0.inverse() * world_to_sander;
    descartes_light::KinematicsInterfaceD::Ptr kin_interface = std::make_shared<ur_ikfast_kinematics::UR10eKinematicsD>(world_to_base_link, tool0_to_sander, nullptr, nullptr);

    for (size_t i = 0; i < waypoints_pose_array.poses.size(); ++i)
    {
        Eigen::Isometry3d current_waypoint_pose;
        tf2::fromMsg(waypoints_pose_array.poses[i], current_waypoint_pose);
        sampler_result.emplace_back(std::make_shared<descartes_light::AxialSymmetricSamplerD>(current_waypoint_pose,
                                                                                                     kin_interface,
                                                                                                     axial_step,
                                                                                                     std::shared_ptr<descartes_light::CollisionInterface<double>>(collision_checker->clone()),
                                                                                                     allow_collisions));
    }

    auto edge_eval = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluatorD>(kin_interface->dof());
    auto timing_constraint = std::vector<descartes_core::TimingConstraintD>(sampler_result.size(), std::numeric_limits<double>::max());

    descartes_light::SolverD graph_builder(kin_interface->dof());

    if (!graph_builder.build(std::move(sampler_result),
                             std::move(timing_constraint),
                             std::move(edge_eval)))
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

    Eigen::Map<Eigen::VectorXd> solution_vec (&solution[0], solution.size());
    Eigen::VectorXd seed_traj(solution_vec.size());
    seed_traj << solution_vec;

    int n_rows = seed_traj.size() / kin_interface->dof();
    Eigen::MatrixXd joint_traj_eigen_out = Eigen::Map<Eigen::MatrixXd>(seed_traj.data(), kin_interface->dof(), n_rows).transpose();

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
        gen_preplan = generateDescartesSeed(strip,
                                            failed_edges,
                                            failed_vertices,
                                            joint_traj_msg_out_init);
        config_->descartes_config.axial_step = 0.05;
        std::cout << "DONE" << std::endl;


        // Check if all rasters reachable
        if (!gen_preplan)
        {
            std::vector<geometry_msgs::msg::PoseArray> split_rasters;
            // Split up raster based on where planning failures occurred
            geometry_msgs::msg::PoseArray curr_failed_vertex_poses;
            std::cout << "CLEANING" << std::endl;
            crs_motion_planning::cleanRasterStrip(strip,
                                                  failed_vertices,
                                                  split_rasters,
                                                  curr_failed_vertex_poses);

            // Display failed vertices
            failed_vertex_poses.poses.insert(failed_vertex_poses.poses.end(), curr_failed_vertex_poses.poses.begin(), curr_failed_vertex_poses.poses.end());

            for (auto split_strip : split_rasters)
            {
                // Generate Descartes preplan
                std::cout << "DESCARTES ROUND 2 ELECTRIC BOOGALO" << std::endl;
                if (generateDescartesSeed(split_strip,
                                          failed_edges,
                                          failed_vertices,
                                          joint_traj_msg_out_final) &&
                        split_strip.poses.size() >= config_->minimum_raster_length)
                {
                  split_traj.push_back(joint_traj_msg_out_final);
                  split_reachable_rasters.push_back(std::move(split_strip));
                  any_successes = true;
                  std::cout << "SUCCESS" << std::endl;
                }
                std::cout << "DONE" << std::endl;
            }
        }
        else
        {
            std::cout << "SUCCESS" << std::endl;
            split_traj.push_back(joint_traj_msg_out_init);
            split_reachable_rasters.push_back(std::move(strip));
        }
        std::cout << "Strip " << ++count_strips << " of " << raster_strips.size() << std::endl;
    }

    // Check if successfully generated preplan with descartes
    if (any_successes)
    {
        std::vector<trajectory_msgs::msg::JointTrajectory> final_split_traj;
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
            if(crs_motion_planning::splitRastersByJointDist(curr_joint_traj,
                                                         split_reachable_rasters[raster_n],
                                                         desired_ee_val,
                                                         max_joint_vel,
                                                         double_split_traj,
                                                         resplit_rasters,
                                                         time_steps))
            {
                std::cout << "FOUND A SPLIT" << std::endl;
                if (config_->add_approach_and_retreate)
                {
                    for (size_t i = 0; i < resplit_rasters.size(); ++i)
                    {
                        geometry_msgs::msg::PoseArray modified_raster;
                        double approach = config_->approach_distance;
                        double retreat = config_->retreat_distance;
                        std::vector<double> modified_time_steps = time_steps[i];
                        if (i == 0)
                        {
                            // Add departure to end of resplit_rasters[i]
                            crs_motion_planning::addApproachAndRetreat(resplit_rasters[i], 0, retreat, modified_raster);
                            // ADD TIME STEPS
                        }
                        else if (i == double_split_traj.size() - 1)
                        {
                            // Add entrance to end of resplit_rasters[i]
                            crs_motion_planning::addApproachAndRetreat(resplit_rasters[i], approach, retreat, modified_raster);
                            // ADD TIME STEPS
                        }
                        else
                        {
                            // Add entrance and departure to resplit_rasters[i]
                            crs_motion_planning::addApproachAndRetreat(resplit_rasters[i], approach, 0, modified_raster);
                            // ADD TIME STEPS
                        }

                        trajectory_msgs::msg::JointTrajectory modified_joint_traj_msg_out;
                        // Generate Descartes preplan
                        std::cout << "DESCARTES ROUND 3, DESCARTES WITH A VEGENCE" << std::endl;
                        std::vector<size_t> failed_edges, failed_vertices;

                        if (generateDescartesSeed(modified_raster,
                                                  failed_edges,
                                                  failed_vertices,
                                                  modified_joint_traj_msg_out))
                        {
                            std::cout << "SUCCESS" << std::endl;
                            for (int j = 0; j < 1; ++j)
                            {
                                final_split_rasters.push_back(modified_raster);
                                final_split_traj.push_back(modified_joint_traj_msg_out);
                                final_time_steps.push_back(modified_time_steps);
                            }
                        }
                        std::cout << "DONE" << std::endl;
                    }
                }
            }
            else
            {
                std::cout << "NO SPLITS FOR YOU" << std::endl;
                for (int j = 0; j < 1; ++j)
                {
                    final_split_rasters.push_back(split_reachable_rasters[raster_n]);
                    final_split_traj.push_back(std::move(curr_joint_traj));
                    final_time_steps.push_back(time_steps[0]);
                }
            }
            raster_n++;
        }
        std::cout << "TIME TO OPTIMIZE" << std::endl;

        //**************************************************TRAJOPT***************************************************
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
            std::cout << "BUILDING WAYPOINT SET " << i+1 << " OF " << final_split_rasters.size() << std::endl;
            for (auto waypoint : final_split_rasters[i].poses)
            {
                Eigen::Vector3d surface_pose(waypoint.position.x, waypoint.position.y, waypoint.position.z);
                Eigen::Quaterniond surface_ori(waypoint.orientation.w, waypoint.orientation.x, waypoint.orientation.y, waypoint.orientation.z);
                tesseract_motion_planners::CartesianWaypoint::Ptr surface_waypoint = std::make_shared<tesseract_motion_planners::CartesianWaypoint>(surface_pose, surface_ori);
                surface_waypoint->setCoefficients(surface_coeffs);
                surface_waypoint->setIsCritical(waypoints_critical);
                curr_raster.push_back(std::move(surface_waypoint));
            }

            auto traj_pc = std::make_shared<tesseract_motion_planners::TrajOptPlannerDefaultConfig>(config_->tesseract_local, config_->manipulator, config_->tcp_frame, config_->tool_offset);

            traj_pc->collision_cost_config = config_->trajopt_surface_config.coll_cst_cfg;
            traj_pc->collision_constraint_config = config_->trajopt_surface_config.coll_cnt_cfg;

            traj_pc->init_type = config_->trajopt_surface_config.init_type;
            traj_pc->longest_valid_segment_fraction = config_->trajopt_surface_config.longest_valid_segment_fraction;

            traj_pc->smooth_velocities = config_->trajopt_surface_config.smooth_velocities;
            traj_pc->smooth_accelerations = config_->trajopt_surface_config.smooth_accelerations;
            traj_pc->smooth_jerks = config_->trajopt_surface_config.smooth_accelerations;
            traj_pc->target_waypoints = curr_raster;

            Eigen::MatrixXd joint_eigen_from_jt;
            joint_eigen_from_jt = tesseract_rosutils::toEigen(final_split_traj[i],final_split_traj[i].joint_names);

            traj_pc->seed_trajectory = joint_eigen_from_jt;

            trajectory_msgs::msg::JointTrajectory trajopt_result_traj;
            tesseract_motion_planners::PlannerResponse planner_resp;
            tesseract_motion_planners::TrajOptMotionPlanner traj_surface_planner;
            traj_surface_planner.setConfiguration(traj_pc);
            std::cout << "Solving raster: " << i+1 << " of " << final_split_rasters.size() << std::endl;
            traj_surface_planner.solve(planner_resp);

            if (planner_resp.status.value() < 0)
            {
                std::cout << "FAILED: " << planner_resp.status.message() << std::endl;
                trajopt_solved.push_back(false);
            }
            else
            {
                std::cout << "SUCCEEDED" << std::endl;
                Eigen::MatrixXd result_traj(planner_resp.joint_trajectory.trajectory.rows(), planner_resp.joint_trajectory.trajectory.cols());
                result_traj << planner_resp.joint_trajectory.trajectory;
                crs_motion_planning::tesseractRosutilsToMsg(trajopt_result_traj, final_split_traj[i].joint_names, result_traj);

                trajopt_trajectories.push_back(std::move(trajopt_result_traj));
                trajopt_solved.push_back(true);
            }
        }
        //**************************************************TRAJOPT***************************************************
        std::cout << "TIME TO GET THIS WHOLE TIME THING DOWN" << std::endl;
        // Assign trajectory timestamps for motion execution
        std::vector<double> traj_times;
        size_t trajopt_traj_n = 0;
        for (size_t i = 0; i < final_split_traj.size(); ++i)
        {
            if (trajopt_solved[i])
            {
                std::cout << "Raster: " << trajopt_traj_n + 1 << " of " << trajopt_trajectories.size() << " with " << trajopt_trajectories[trajopt_traj_n].points.size() << " waypoints " << std::endl;
                trajopt_trajectories[trajopt_traj_n].header.frame_id  = "world";
                double curr_traj_time = 0;
                for (size_t j = 1; j < trajopt_trajectories[trajopt_traj_n].points.size(); ++j)
                {
                    trajopt_trajectories[trajopt_traj_n].points[j-1].time_from_start.sec = static_cast<int>(floor(final_time_steps[i][j]));
                    trajopt_trajectories[trajopt_traj_n].points[j-1].time_from_start.nanosec = static_cast<uint>(1e9 * (final_time_steps[i][j] - floor(final_time_steps[i][j])));
                    curr_traj_time += final_time_steps[i][j];
                }
                trajopt_trajectories[trajopt_traj_n].points.back().time_from_start.sec = 0;
                traj_times.push_back(std::move(curr_traj_time));
                trajopt_traj_n++;
            }
        }

        std::cout << "ALL DONE" << std::endl;
        results->final_raster_trajectories = std::move(trajopt_trajectories);
        return true;
    }
    else
    {

        return false;
    }
}

bool crsMotionPlanner::generateProcessPlan(pathPlanningResults::Ptr& results)
{
    std::cout << "generating surface plans" << std::endl;
    bool success_path = generateSurfacePlans(results);
    return success_path;
}

} // namespace crs_motion_planning
