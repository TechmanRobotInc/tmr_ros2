#include <rclcpp/rclcpp.hpp>
#include <tm_msgs/srv/send_script.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

bool send_cmd(std::string cmd, std::shared_ptr<rclcpp::Node> node, rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client){
  auto request = std::make_shared<tm_msgs::srv::SendScript::Request>();
  request->id = "demo";
  request->script = cmd;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if(result.get()->ok){
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"OK");
    } else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"not OK");
    }

  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }
  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("demo_leave_listen_node");
  rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client =
    node->create_client<tm_msgs::srv::SendScript>("send_script");
  
  std::string cmd = "ScriptExit()";
  
  send_cmd(cmd, node, client);

  rclcpp::shutdown();
  return 0;
}
