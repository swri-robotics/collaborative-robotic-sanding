#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>

static const std::string CONTROLLER_CHANGER_SERVICE = "compliance_controller_on";
static const std::string TOGGLE_SANDER_SERVICE = "toggle_sander";

class URRobotCommsSim : public rclcpp::Node
{
public:
  URRobotCommsSim() : Node("ur_robot_comms_sim")
  {
    // ROS communications
    controller_changer_service_ = this->create_service<std_srvs::srv::SetBool>(
        CONTROLLER_CHANGER_SERVICE,
        std::bind(&URRobotCommsSim::simControllerChangeCB, this, std::placeholders::_1, std::placeholders::_2));
    ur_io_service_ = this->create_service<std_srvs::srv::SetBool>(
        TOGGLE_SANDER_SERVICE,
        std::bind(&URRobotCommsSim::simUrIoCB, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void simControllerChangeCB(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    response->success = true;
  }
  void simUrIoCB(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                 std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    response->success = true;
  }
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr controller_changer_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr ur_io_service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<rclcpp::Node> node = std::make_shared<URRobotCommsSim>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
