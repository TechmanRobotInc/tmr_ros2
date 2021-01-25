#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/ask_sta.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include "iostream"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);


  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("demo_ask_sta");
  rclcpp::Client<tm_msgs::srv::AskSta>::SharedPtr client =
    node->create_client<tm_msgs::srv::AskSta>("ask_sta");
  
  auto request = std::make_shared<tm_msgs::srv::AskSta::Request>();
  request->subcmd = "00";
  request->subdata = "";
  request->wait_time = 1;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      std::cout<<"Interrupted while waiting for the service. Exiting."<<std::endl;
      return false;
    }
    std::cout<<"service not available, waiting again...";
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    if(result.get()->ok){
      std::cout<<"OK"<<std::endl;
      std::cout<<"subcmd is "<<result.get()->subcmd<<std::endl;
      std::cout<<"subdata is "<<result.get()->subdata<<std::endl;
    } else{
      std::cout<<"not OK"<<std::endl;
    }

  } else {
    std::cout<<"Failed to call service"<<std::endl;
  }
  return true;

  rclcpp::shutdown();
  return 0;
}
