#include <rclcpp/rclcpp.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

//#include <crs_motion_planning/path_processing_utils.h>
#include <crs_motion_planning/path_planning_utils.h>

#include <crs_msgs/srv/call_freespace_motion.hpp>
#include <crs_msgs/srv/plan_process_motions.hpp>

#include <tf2/transform_storage.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/pose.hpp>

static const std::vector<double> COEFFICIENTS {10, 10, 10, 10, 10, 10};



class SurfaceServer: public rclcpp::Node
{
public:
  SurfaceServer()
    : Node("surface_server_node"),
      clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)),
      tf_buffer_(clock_),
      tf_listener_(tf_buffer_)
  {
    // ROS parameters
    this->declare_parameter("urdf_path", ament_index_cpp::get_package_share_directory("crs_support") + "/urdf/crs_v3.urdf");
    this->declare_parameter("srdf_path", ament_index_cpp::get_package_share_directory("crs_support") + "/urdf/ur10e_robot.srdf");
    this->declare_parameter("base_link_frame", "world");
    this->declare_parameter("manipulator_group", "manipulator");

    // ROS communications
    load_paths_service_ = this->create_service<std_srvs::srv::Trigger>("load_paths", std::bind(&SurfaceServer::planService, this, std::placeholders::_1, std::placeholders::_2));
    traj_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("set_trajectory_test",1);
    original_path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("original_raster_paths",1);
    corrected_path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("fixed_raster_paths",1);
    failed_vertex_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("failed_vertices",1);
    joint_state_listener_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1, std::bind(&SurfaceServer::jointCallback, this, std::placeholders::_1));
    call_process_plan_client_ = this->create_client<crs_msgs::srv::PlanProcessMotions>("plan_process_motion");

    std::string urdf_path, srdf_path;
    urdf_path = this->get_parameter("urdf_path").as_string();
    srdf_path = this->get_parameter("srdf_path").as_string();
    std::stringstream urdf_xml_string, srdf_xml_string;
    std::ifstream urdf_in(urdf_path);
    urdf_xml_string << urdf_in.rdbuf();
    std::ifstream srdf_in(srdf_path);
    srdf_xml_string << srdf_in.rdbuf();

    tesseract_local_ = std::make_shared<tesseract::Tesseract>();
    tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    tesseract_local_->init(urdf_xml_string.str(), srdf_xml_string.str(), locator);

    base_link_frame_ = this->get_parameter("base_link_frame").as_string();
    manipulator_ = this->get_parameter("manipulator_group").as_string();
    toolpath_filepath_ = ament_index_cpp::get_package_share_directory("crs_support") + "/toolpaths/scanned_part1/job_90degrees.yaml";
  }
private:
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr joint_msg)
  {
    curr_joint_state_ = *joint_msg;
  }
  void planService(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {

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
        world_to_goal_frame = tf_buffer_.lookupTransform("world", "part", time_point);
      }
      catch (tf2::LookupException &e)
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
      proc_req->tool_speed = 0.4;
      proc_req->approach_dist = 0.05;
      proc_req->retreat_dist = 0.05;
      proc_req->start_position = curr_joint_state_;
      proc_req->end_position = curr_joint_state_;
      Eigen::Isometry3d tool_offset_req = Eigen::Isometry3d::Identity();
      geometry_msgs::msg::Pose geom_tool_offset;
      tesseract_rosutils::toMsg(geom_tool_offset, tool_offset_req);
      proc_req->tool_offset = geom_tool_offset;
      std::vector<crs_msgs::msg::ToolProcessPath> path_requests;
      crs_msgs::msg::ToolProcessPath path_wf;
      path_wf.rasters = raster_strips_world_frame;
      path_requests.push_back(path_wf);
      proc_req->process_paths = path_requests;

      auto process_plan_cb = std::bind(&SurfaceServer::processPlanCallback, this, std::placeholders::_1);
      call_process_plan_client_->async_send_request(proc_req, process_plan_cb);

      response->success = true;
      response->message = "TRAJECTORIES PUBLISHED";
  }

  void processPlanCallback(const rclcpp::Client<crs_msgs::srv::PlanProcessMotions>::SharedFuture future)
  {
      bool success = future.get()->succeeded;

      if (success)
      {
          std::vector<crs_msgs::msg::ProcessMotionPlan> process_plans = future.get()->plans;
          for (size_t j = 0; j < process_plans.size(); ++j)
          {
              std::cout << "PUBLISHING PROCESS\t" << j+1 << " OF " << process_plans.size() << std::endl;
              trajectory_msgs::msg::JointTrajectory start_traj = process_plans[j].start;
              trajectory_msgs::msg::JointTrajectory end_traj = process_plans[j].end;
              std::vector<trajectory_msgs::msg::JointTrajectory> process_motions = process_plans[j].process_motions;
              std::vector<trajectory_msgs::msg::JointTrajectory> freespace_motions = process_plans[j].free_motions;
              traj_publisher_->publish(start_traj);
              std::this_thread::sleep_for(std::chrono::seconds(start_traj.points.size()/10 + 1));

              for (size_t i = 0; i < freespace_motions.size(); ++i)
              {
                  std::cout << "PUBLISHING SURFACE\t" << i+1 << " OF " << process_motions.size() << std::endl;
                  traj_publisher_->publish(process_motions[i]);
                  std::this_thread::sleep_for(std::chrono::seconds(2));
                  std::cout << "PUBLISHING FREESPACE\t" << i+1 << " OF " << freespace_motions.size() << std::endl;
                  traj_publisher_->publish(freespace_motions[i]);
                  std::this_thread::sleep_for(std::chrono::seconds(freespace_motions[i].points.size()/10 + 1));
              }
              std::cout << "PUBLISHING SURFACE\t" << process_motions.size() << " OF " << process_motions.size() << std::endl;
              traj_publisher_->publish(process_motions.back());
              std::this_thread::sleep_for(std::chrono::seconds(2));
              traj_publisher_->publish(end_traj);
          }

          std::cout << "ALL DONE" << std::endl;
      }
      else
      {
          std::cout << future.get()->err_msg << std::endl;
      }


  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_paths_service_;
  rclcpp::Client<crs_msgs::srv::PlanProcessMotions>::SharedPtr call_process_plan_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr original_path_publisher_, corrected_path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr failed_vertex_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_listener_;
  tesseract::Tesseract::Ptr tesseract_local_;

  std::shared_ptr<rclcpp::Clock> clock_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  sensor_msgs::msg::JointState curr_joint_state_;

  std::string base_link_frame_;
  std::string manipulator_;
  std::string toolpath_filepath_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SurfaceServer>());
  rclcpp::shutdown();
  return 0;
}
