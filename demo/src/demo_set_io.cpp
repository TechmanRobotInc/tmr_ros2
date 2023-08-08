#include <rclcpp/rclcpp.hpp>
#include <tm_msgs/srv/set_io.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("demo_set_io");
  rclcpp::Client<tm_msgs::srv::SetIO>::SharedPtr client =
    node->create_client<tm_msgs::srv::SetIO>("set_io");
  
  auto request = std::make_shared<tm_msgs::srv::SetIO::Request>();
  request->module = tm_msgs::srv::SetIO::Request::MODULE_CONTROLBOX;
  request->type = tm_msgs::srv::SetIO::Request::TYPE_DIGITAL_OUT;
  request->pin = 0;
  request->state = tm_msgs::srv::SetIO::Request::STATE_ON;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 1;
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

  rclcpp::shutdown();
  return 0;
}
