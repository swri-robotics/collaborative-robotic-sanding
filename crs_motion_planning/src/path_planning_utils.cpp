#include <crs_motion_planning/path_planning_utils.h>

namespace crs_motion_planning
{

bool generateDescartesSeed(const tesseract_kinematics::ForwardKinematics::ConstPtr kin,
                                                const std::shared_ptr<const tesseract_environment::Environment> env,
                                                const geometry_msgs::msg::PoseArray &waypoints,
                                                const descartes_light::KinematicsInterfaceD::Ptr &kin_interface,
                                                const double &axial_step,
                                                const bool &allow_collisions,
                                                const double &collision_safety_margin,
                                                std::vector<std::size_t>& failed_edges,
                                                std::vector<std::size_t>& failed_vertices,
                                                trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
    tesseract_common::TransformMap curr_transforms = env->getCurrentState()->transforms;

    tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(env->getSceneGraph(), kin->getActiveLinkNames(), curr_transforms);

    auto collision_checker = std::make_shared<tesseract_motion_planners::DescartesCollisionD>(env, adjacency_map->getActiveLinkNames(), kin->getJointNames(), collision_safety_margin);

    std::vector<descartes_light::PositionSamplerD::Ptr> sampler_result;

    for (size_t i = 0; i < waypoints.poses.size(); ++i)
    {
        Eigen::Isometry3d current_waypoint_pose;
        tf2::fromMsg(waypoints.poses[i], current_waypoint_pose);
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


bool generateDescartesSeed(const crs_motion_planning::pathPlanningConfig::Ptr config,
                           const geometry_msgs::msg::PoseArray &waypoints,
                           const double &axial_step,
                           const bool &allow_collisions,
                           const double &collision_safety_margin,
                           std::vector<std::size_t>& failed_edges,
                           std::vector<std::size_t>& failed_vertices,
                           trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
    tesseract::Tesseract::Ptr tesseract_local = config->tesseract_local;
    const std::shared_ptr<const tesseract_environment::Environment> env = tesseract_local->getEnvironmentConst();
    tesseract_common::TransformMap curr_transforms = env->getCurrentState()->transforms;

    tesseract_kinematics::ForwardKinematics::ConstPtr kin = tesseract_local->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config->manipulator);

    tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(env->getSceneGraph(), kin->getActiveLinkNames(), curr_transforms);

    auto collision_checker = std::make_shared<tesseract_motion_planners::DescartesCollisionD>(env, adjacency_map->getActiveLinkNames(), kin->getJointNames(), collision_safety_margin);

    std::vector<descartes_light::PositionSamplerD::Ptr> sampler_result;

    Eigen::Isometry3d world_to_base_link, world_to_sander, world_to_tool0, tool0_to_sander;
    world_to_base_link = curr_transforms.find(config->robot_base_frame)->second;
    world_to_sander = curr_transforms.find(config->tcp_frame)->second;
    world_to_tool0 = curr_transforms.find(config->tool0_frame)->second;
    tool0_to_sander = world_to_tool0.inverse() * world_to_sander;
    descartes_light::KinematicsInterfaceD::Ptr kin_interface = std::make_shared<ur_ikfast_kinematics::UR10eKinematicsD>(world_to_base_link, tool0_to_sander, nullptr, nullptr);

    for (size_t i = 0; i < waypoints.poses.size(); ++i)
    {
        Eigen::Isometry3d current_waypoint_pose;
        tf2::fromMsg(waypoints.poses[i], current_waypoint_pose);
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

bool generateDescartesSeed(const crs_motion_planning::pathPlanningConfig::Ptr config,
                           const geometry_msgs::msg::PoseArray &waypoints,
                           std::vector<std::size_t>& failed_edges,
                           std::vector<std::size_t>& failed_vertices,
                           trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
    return generateDescartesSeed(config,
                                 waypoints,
                                 config->descartes_config.axial_step,
                                 config->descartes_config.allow_collisions,
                                 config->descartes_config.collision_safety_margin,
                                 failed_edges,
                                 failed_vertices,
                                 joint_trajectory);
}

crsMotionPlanner::crsMotionPlanner(pathPlanningConfig::Ptr config) : config_(std::move(config)) {}

void crsMotionPlanner::updateConfiguration(pathPlanningConfig::Ptr config)
{
    config_ = std::move(config);
}

bool crsMotionPlanner::generateDescartesSeed(const geometry_msgs::msg::PoseArray &waypoints_pose_array,
                                             const double &axial_step,
                                             const bool &allow_collisions,
                                             const double &collision_safety_margin,
                                             std::vector<std::size_t>& failed_edges,
                                             std::vector<std::size_t>& failed_vertices,
                                             trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
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

bool crsMotionPlanner::generateDescartesSeed(const geometry_msgs::msg::PoseArray &waypoints_pose_array,
                                             std::vector<std::size_t>& failed_edges,
                                             std::vector<std::size_t>& failed_vertices,
                                             trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
    return crsMotionPlanner::generateDescartesSeed(waypoints_pose_array,
                                                   config_->descartes_config.axial_step,
                                                   config_->descartes_config.allow_collisions,
                                                   config_->descartes_config.collision_safety_margin,
                                                   failed_edges,
                                                   failed_vertices,
                                                   joint_trajectory);
}

} // namespace crs_motion_planning
