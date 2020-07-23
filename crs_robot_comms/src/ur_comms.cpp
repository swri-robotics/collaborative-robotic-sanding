#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <ur_dashboard_msgs/srv/load.hpp>
#include <ur_dashboard_msgs/srv/is_program_running.hpp>

#include <crs_msgs/srv/run_robot_script.hpp>

static const std::string UR_DASHBOARD_LOAD_SERVICE = "/ur_hardware_interface/dashboard/load_program";
static const std::string UR_DASHBOARD_PLAY_SERVICE = "/ur_hardware_interface/dashboard/play";
static const std::string UR_RESTART_CONTROL_SERVICE = "/ur_hardware_interface/resend_robot_program";
static const std::string UR_CHECK_RUNNING_SERVICE = "/ur_hardware_interface/dashboard/program_running";
static const std::string CRS_RUN_ROBOT_SCRIPT_SERVICE = "run_robot_script";
static const std::string RECLAIM_REMOTE_CONTROL_SERVICE = "reconnect_to_robot";

class URRobotComms
{
public:
  URRobotComms(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
    , pnode_(std::make_shared<rclcpp::Node>(std::string(node_->get_name()) + "_private"))
  {
    // ROS communications
    load_robot_script_service_ = node_->create_service<crs_msgs::srv::RunRobotScript>(
        CRS_RUN_ROBOT_SCRIPT_SERVICE,
        std::bind(&URRobotComms::runRobotScriptCB, this, std::placeholders::_1, std::placeholders::_2));

    reclaim_remote_control_service_= node_->create_service<std_srvs::srv::Trigger>(
                RECLAIM_REMOTE_CONTROL_SERVICE,
                std::bind(&URRobotComms::reclaimRemoteControlCB, this, std::placeholders::_1, std::placeholders::_2));

    load_ur_program_client_ = pnode_->create_client<ur_dashboard_msgs::srv::Load>(UR_DASHBOARD_LOAD_SERVICE);
    check_if_running_client_ = pnode_->create_client<ur_dashboard_msgs::srv::IsProgramRunning>(UR_CHECK_RUNNING_SERVICE);
    play_robot_script_client_ = pnode_->create_client<std_srvs::srv::Trigger>(UR_DASHBOARD_PLAY_SERVICE);
    restart_robot_control_client_ = pnode_->create_client<std_srvs::srv::Trigger>(UR_RESTART_CONTROL_SERVICE);
  }

  void get_priv_node(rclcpp::Node::SharedPtr& priv_node)
  {
    priv_node = pnode_;
  }

private:

  bool establishComputerControl()
  {
      // Return control of UR back to computer
      std_srvs::srv::Trigger::Request::SharedPtr remote_control_req = std::make_shared<std_srvs::srv::Trigger::Request>();
      std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr> remote_control_future = restart_robot_control_client_->async_send_request(remote_control_req);

      std::future_status remote_control_status = remote_control_future.wait_for(std::chrono::seconds(5));
      if (remote_control_status != std::future_status::ready)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "resend_robot_program UR service error or timeout");
        return false;
      }
      if (!remote_control_future.get()->success)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "resend_robot_program UR service failed");
        return false;
      }
      RCLCPP_INFO(node_->get_logger(), "resend_robot_program UR service succeeded");
      return true;
  }

  void runRobotScriptCB(std::shared_ptr<crs_msgs::srv::RunRobotScript::Request> request,
                        std::shared_ptr<crs_msgs::srv::RunRobotScript::Response> response)
  {
    rclcpp::Rate r(1);

    // Call load service on UR dashboard
    ur_dashboard_msgs::srv::Load::Request::SharedPtr load_script_req = std::make_shared<ur_dashboard_msgs::srv::Load::Request>();
    load_script_req->filename = request->filename;
    std::shared_future<ur_dashboard_msgs::srv::Load::Response::SharedPtr> load_script_future = load_ur_program_client_->async_send_request(load_script_req);

    std::future_status load_script_status = load_script_future.wait_for(std::chrono::seconds(20));
    if (load_script_status != std::future_status::ready)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Load UR dashboard service error or timeout");
      response->success = false;
      response->answer = "Unable to complete process";
      return;
    }
    if (!load_script_future.get()->success)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Load UR dashboard service failed");
      response->success = false;
      response->answer = "Unable to complete process";
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "Load UR dashboard service succeeded");
    r.sleep();

    // Call play service on UR dashboard
    std_srvs::srv::Trigger::Request::SharedPtr play_script_req = std::make_shared<std_srvs::srv::Trigger::Request>();
    std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr> play_script_future = play_robot_script_client_->async_send_request(play_script_req);

    std::future_status play_script_status = play_script_future.wait_for(std::chrono::seconds(20));
    if (play_script_status != std::future_status::ready)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Play UR dashboard service error or timeout");
      response->success = false;
      response->answer = "Unable to complete process";
      return;
    }
    if (!play_script_future.get()->success)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Play UR dashboard service failed");
      response->success = false;
      response->answer = "Unable to complete process";
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "Play UR dashboard service succeeded");
    r.sleep();

    // Check to see if script has finished playing
    ur_dashboard_msgs::srv::IsProgramRunning::Request::SharedPtr check_script_req = std::make_shared<ur_dashboard_msgs::srv::IsProgramRunning::Request>();
    bool script_finished = false;
    while (!script_finished)
    {
        r.sleep();
        std::shared_future<ur_dashboard_msgs::srv::IsProgramRunning::Response::SharedPtr> check_script_future = check_if_running_client_->async_send_request(check_script_req);
        std::future_status check_script_status = check_script_future.wait_for(std::chrono::seconds(20));
        if (check_script_status != std::future_status::ready)
        {
          RCLCPP_ERROR(node_->get_logger(),
                       "Program running UR dashboard service error or timeout");
          response->success = false;
          response->answer = "Unable to complete process";
          return;
        }
        if (!check_script_future.get()->success)
        {
          RCLCPP_ERROR(node_->get_logger(),
                       "Program running UR dashboard service failed");
          response->success = false;
          response->answer = "Unable to complete process";
          return;
        }
        script_finished = !check_script_future.get()->program_running;
    }
//    r.sleep();

    // Return control of UR back to computer
    if (!establishComputerControl())
    {
        response->success = false;
        response->answer = "Unable to complete process";
        return;
    }

    response->success = true;
    response->answer = "Robot script completed";
  }

  void reclaimRemoteControlCB(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
      // Return control of UR back to computer
      if (!establishComputerControl())
      {
          response->success = false;
          response->message = "Unable to complete process";
          return;
      }
      response->success = true;
      response->message = "Remote control over robot established";
  }

  rclcpp::Service<crs_msgs::srv::RunRobotScript>::SharedPtr load_robot_script_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reclaim_remote_control_service_;

  rclcpp::Client<ur_dashboard_msgs::srv::Load>::SharedPtr load_ur_program_client_;
  rclcpp::Client<ur_dashboard_msgs::srv::IsProgramRunning>::SharedPtr check_if_running_client_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr play_robot_script_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr restart_robot_control_client_;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> pnode_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("ur_comms_node");
  executor.add_node(node);
  URRobotComms exec(node);
  std::shared_ptr<rclcpp::Node> pnode;
  exec.get_priv_node(pnode);
  executor.add_node(pnode);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
