#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <crs_motion_planning/path_planning_utils.h>

#include <crs_msgs/srv/call_freespace_motion.hpp>
#include <crs_msgs/srv/plan_process_motions.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <crs_msgs/srv/load_part.hpp>

#include <tf2/transform_storage.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <boost/format.hpp>

static const double WAIT_SERVER_TIMEOUT = 10.0;  // seconds
static const std::string FOLLOW_JOINT_TRAJECTORY_ACTION = "follow_joint_trajectory";

class ProcessPlannerTestServer : public rclcpp::Node
{
public:
  ProcessPlannerTestServer()
    : Node("process_planner_test_node")
    , clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME))
    , tf_buffer_(clock_)
    , tf_listener_(tf_buffer_)
  {
    // ROS communications
    test_process_planner_service_ = this->create_service<std_srvs::srv::Trigger>(
        "test_process_planner",
        std::bind(&ProcessPlannerTestServer::planService, this, std::placeholders::_1, std::placeholders::_2));

    trajectory_exec_client_cbgroup_ =
        this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    trajectory_exec_client_ =
        rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(this->get_node_base_interface(),
                                                                                  this->get_node_graph_interface(),
                                                                                  this->get_node_logging_interface(),
                                                                                  this->get_node_waitables_interface(),
                                                                                  FOLLOW_JOINT_TRAJECTORY_ACTION,
                                                                                  trajectory_exec_client_cbgroup_);

    joint_state_listener_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 1, std::bind(&ProcessPlannerTestServer::jointCallback, this, std::placeholders::_1));

    call_process_plan_client_ = this->create_client<crs_msgs::srv::PlanProcessMotions>("plan_process_motion");

    part_loader_client_ = this->create_client<crs_msgs::srv::LoadPart>("load_part_tesseract_env");

    toolpath_filepath_ = ament_index_cpp::get_package_share_directory("crs_support") + "/toolpaths/scanned_part1/"
                                                                                       "job_90degrees.yaml";

    test_part_loader_client_ = this->create_service<std_srvs::srv::Trigger>(
        "test_part_loader",
        std::bind(&ProcessPlannerTestServer::testPartLoader, this, std::placeholders::_1, std::placeholders::_2));

    part_filepath_ = ament_index_cpp::get_package_share_directory("crs_support") + "/meshes/Parts/visual/"
                                                                                   "part1_ch.stl";
    // waiting for server
    if (!trajectory_exec_client_->wait_for_action_server(std::chrono::duration<double>(WAIT_SERVER_TIMEOUT)))
    {
      std::string err_msg =
          boost::str(boost::format("Failed to find action server %s") % FOLLOW_JOINT_TRAJECTORY_ACTION);
      RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
      throw std::runtime_error(err_msg);
    }
    RCLCPP_INFO(this->get_logger(), "%s action client found", FOLLOW_JOINT_TRAJECTORY_ACTION.c_str());
  }

private:
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr joint_msg) { curr_joint_state_ = *joint_msg; }

  void planService(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Planning now");
    // Load rasters and get them in usable form
    std::string waypoint_origin_frame = "part";
    std::vector<geometry_msgs::msg::PoseArray> raster_strips;
    crs_motion_planning::parsePathFromFile(toolpath_filepath_, waypoint_origin_frame, raster_strips);
    geometry_msgs::msg::PoseArray strip_of_interset;
    for (auto strip : raster_strips)
    {
      strip_of_interset.poses.insert(strip_of_interset.poses.end(), strip.poses.begin(), strip.poses.end());
    }

    // Get transform between world and part
    tf2::TimePoint time_point = tf2::TimePointZero;
    geometry_msgs::msg::TransformStamped world_to_goal_frame;
    try
    {
      world_to_goal_frame = tf_buffer_.lookupTransform("world", waypoint_origin_frame, time_point);
    }
    catch (tf2::LookupException& e)
    {
      response->success = false;
      response->message = "TF lookup failed: " + std::string(e.what());
      return;
    }

    std::vector<geometry_msgs::msg::PoseArray> raster_strips_world_frame;
    for (auto strip : raster_strips)
    {
      geometry_msgs::msg::PoseArray curr_strip, ar_strip;
      for (size_t i = 0; i < strip.poses.size(); ++i)
      {
        geometry_msgs::msg::PoseStamped surface_pose_world_frame, surface_pose_og_frame;
        surface_pose_og_frame.pose = strip.poses[i];
        surface_pose_og_frame.header = strip.header;
        tf2::doTransform(surface_pose_og_frame, surface_pose_world_frame, world_to_goal_frame);
        geometry_msgs::msg::Pose sf_pose_wf = surface_pose_world_frame.pose;
        curr_strip.poses.push_back(std::move(sf_pose_wf));
      }
      raster_strips_world_frame.push_back(curr_strip);
    }

    auto proc_req = std::make_shared<crs_msgs::srv::PlanProcessMotions::Request>();
    proc_req->tool_link = "sander_center_link";
    proc_req->tool_speed = 0.3;
    proc_req->approach_dist = 0.045;
    proc_req->retreat_dist = 0.045;
    proc_req->start_position = curr_joint_state_;
    proc_req->end_position = curr_joint_state_;
    Eigen::Isometry3d tool_offset_req = Eigen::Isometry3d::Identity();
    geometry_msgs::msg::Pose geom_tool_offset;
    tesseract_rosutils::toMsg(geom_tool_offset, tool_offset_req);
    geom_tool_offset.position.z = 0.0;
    proc_req->tool_offset = geom_tool_offset;
    std::vector<crs_msgs::msg::ToolProcessPath> path_requests;
    crs_msgs::msg::ToolProcessPath path_wf;
    path_wf.rasters = raster_strips_world_frame;
    path_requests.push_back(path_wf);
    proc_req->process_paths = path_requests;

    auto process_plan_cb = std::bind(&ProcessPlannerTestServer::processPlanCallback, this, std::placeholders::_1);
    call_process_plan_client_->async_send_request(proc_req, process_plan_cb);

    response->success = true;
    response->message = "Plan Sent";
  }

  void processPlanCallback(const rclcpp::Client<crs_msgs::srv::PlanProcessMotions>::SharedFuture future)
  {
    using namespace crs_motion_planning;
    bool success = future.get()->succeeded;

    if (success)
    {
      std::vector<crs_msgs::msg::ProcessMotionPlan> process_plans = future.get()->plans;
      for (size_t j = 0; j < process_plans.size(); ++j)
      {
        if (!rclcpp::ok())
        {
          return;
        }
        RCLCPP_INFO(this->get_logger(), "EXECUTING PROCESS\t%i OF %i", j + 1, process_plans.size());
        trajectory_msgs::msg::JointTrajectory start_traj = process_plans[j].start;
        trajectory_msgs::msg::JointTrajectory end_traj = process_plans[j].end;
        std::vector<trajectory_msgs::msg::JointTrajectory> process_motions = process_plans[j].process_motions;
        std::vector<trajectory_msgs::msg::JointTrajectory> freespace_motions = process_plans[j].free_motions;
        if (start_traj.points.size() > 0)
        {
          RCLCPP_INFO(this->get_logger(), "EXECUTING FIRST FREESPACE");
          if (!execTrajectory(trajectory_exec_client_, this->get_logger(), start_traj))
          {
            return;
          }
        }

        for (size_t i = 0; i < freespace_motions.size(); ++i)
        {
          RCLCPP_INFO(this->get_logger(), "EXECUTING SURFACE TRAJECTORY\t%i OF %i", i + 1, process_motions.size());
          if (!execTrajectory(trajectory_exec_client_, this->get_logger(), process_motions[i]))
          {
            return;
          }

          RCLCPP_INFO(this->get_logger(), "EXECUTING FREESPACE TRAJECTORY\t%i OF %i", i + 1, freespace_motions.size());
          if (!execTrajectory(trajectory_exec_client_, this->get_logger(), freespace_motions[i]))
          {
            return;
          }
        }

        RCLCPP_INFO(this->get_logger(),
                    "EXECUTING SURFACE TRAJECTORY\t%i OF %i",
                    process_motions.size(),
                    process_motions.size());
        execTrajectory(trajectory_exec_client_, this->get_logger(), process_motions.back());
        if (end_traj.points.size() > 0)
        {
          RCLCPP_INFO(this->get_logger(), "EXECUTING FINAL FREESPACE");
          if (!execTrajectory(trajectory_exec_client_, this->get_logger(), end_traj))
          {
            return;
          }
        }
      }

      std::cout << "ALL DONE" << std::endl;
    }
    else
    {
      std::cout << future.get()->err_msg << std::endl;
    }
  }

  void testPartLoader(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    geometry_msgs::msg::Pose part_pose;
    part_pose.position.x = -0.53;
    part_pose.position.y = -0.14;
    part_pose.position.z = 1.1;

    part_pose.orientation.x = 0.5;
    part_pose.orientation.y = -0.5;
    part_pose.orientation.z = 0.5;
    part_pose.orientation.w = 0.5;

    auto load_part_request = std::make_shared<crs_msgs::srv::LoadPart::Request>();
    load_part_request->path_to_part = part_filepath_;
    load_part_request->part_origin = part_pose;

    part_loader_client_->async_send_request(load_part_request);
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_process_planner_service_;
  rclcpp::Client<crs_msgs::srv::PlanProcessMotions>::SharedPtr call_process_plan_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_exec_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_listener_;
  rclcpp::callback_group::CallbackGroup::SharedPtr trajectory_exec_client_cbgroup_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_part_loader_client_;
  rclcpp::Client<crs_msgs::srv::LoadPart>::SharedPtr part_loader_client_;

  std::shared_ptr<rclcpp::Clock> clock_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  sensor_msgs::msg::JointState curr_joint_state_;

  std::string toolpath_filepath_;
  std::string part_filepath_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::Node::SharedPtr node = std::make_shared<ProcessPlannerTestServer>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
