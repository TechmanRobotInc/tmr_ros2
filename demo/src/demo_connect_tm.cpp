#include <rclcpp/rclcpp.hpp>
#include <tm_msgs/srv/connect_tm.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);


  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("demo_connect_tm");
  rclcpp::Client<tm_msgs::srv::ConnectTM>::SharedPtr client =
    node->create_client<tm_msgs::srv::ConnectTM>("connect_tmsvr");
  
  auto request = std::make_shared<tm_msgs::srv::ConnectTM::Request>();
  request->server = tm_msgs::srv::ConnectTM::Request::TMSVR;
  request->reconnect = true;
  request->timeout = 0;
  request->timeval = 0;

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
    rclcpp::executor::FutureReturnCode::SUCCESS)
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
