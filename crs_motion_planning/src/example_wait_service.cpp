#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>


class ExampleServerClientCall
{
public:
  ExampleServerClientCall(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
    , pnode_(std::make_shared<rclcpp::Node>(std::string(node_->get_name()) + "_private"))
  {
    test_process_planner_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "internal_server",
        std::bind(&ProcessPlannerTestServer::planService, this, std::placeholders::_1, std::placeholders::_2));

    call_process_plan_client_ = pnode_->create_client<std_srvs::srv::Trigger>("external_service_to_be_called");
  }

  void get_priv_node(rclcpp::Node::SharedPtr& private_node)
  {
    private_node = pnode_;
  }

private:

  void planService(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    auto proc_req = std::make_shared<std_srvs::srv::Trigger::Request>();
    std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr> result_future = call_client_->async_send_request(proc_req);

    std::future_status status = result_future.wait_for(std::chrono::seconds(20));
    if (status != std::future_status::ready)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "service error or timeout");
    }

    if (!result_future.get()->succeeded)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "service failed");
    }

    RCLCPP_INFO(node_->get_logger(), "service succeeded");

    response->success = true;
    response->message = "Successful";
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> pnode_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<rclcpp::Node> main_node = std::make_shared<rclcpp::Node>("ros_node");
  executor.add_node(main_node);
  ExampleServerClientCall exec(main_node);
  std::shared_ptr<rclcpp::Node> private_node;
  exec.get_priv_node(private_node);
  executor.add_node(private_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
