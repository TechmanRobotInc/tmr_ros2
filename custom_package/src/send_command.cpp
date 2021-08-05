#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "techman_robot_msgs/srv/techman_robot_command.hpp"

int main(int argc, char * argv[]){
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("send_command_client");
  auto client = node->create_client<techman_robot_msgs::srv::TechmanRobotCommand>("tm_send_command");

  while(!client->wait_for_service(1s)){
    if(!rclcpp::ok()){
      RCLCPP_ERROR_STREAM(node->get_logger(), "Client interrupted while waiting for service to appear.");
      return 1;
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "waiting for service...");
  }

  auto request = std::make_shared<techman_robot_msgs::srv::TechmanRobotCommand::Request>();
  request->command = "MOVE_JOG";
  request->command_parameter_string = "0,0,90,0,90,0";

  auto res_future = client->async_send_request(request);
  if(rclcpp::spin_until_future_complete(node, res_future) != rclcpp::executor::FutureReturnCode::SUCCESS){
    RCLCPP_ERROR_STREAM(node->get_logger(), "Service call failed.");
    return 1;
  }

  auto res = res_future.get();
  RCLCPP_INFO_STREAM(node->get_logger(), "is_success " << res->is_success);	

  rclcpp::shutdown();
  return 0;
}