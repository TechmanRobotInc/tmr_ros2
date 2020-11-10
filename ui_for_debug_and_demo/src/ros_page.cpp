#include "tm_ros_driver_windows.hpp"
using namespace std::chrono_literals;
void RosPage::feedback_states_callback(tm_msgs::msg::FeedbackState::SharedPtr msg){
  send_ui_feed_back_status(msg);
}

void RosPage::initial_subscriber(){
  feedBackStatusSubscription = node->create_subscription<tm_msgs::msg::FeedbackState>(
      "feedback_states", 10, std::bind(&RosPage::feedback_states_callback, this, std::placeholders::_1));
}
void RosPage::initial_client(){
  connectSvrClient = node->create_client<tm_msgs::srv::ConnectTM>("connect_tmsvr");
  connectSctClient = node->create_client<tm_msgs::srv::ConnectTM>("connect_tmsct");
}
void RosPage::send_service(rclcpp::Client<tm_msgs::srv::ConnectTM>::SharedPtr client, std::shared_ptr<tm_msgs::srv::ConnectTM::Request> request){
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return ;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  
  if(result.get()->ok){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OK");
  } else{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"not OK");
  }

}
void RosPage::send_sct_as_re_connect(){
  auto request = std::make_shared<tm_msgs::srv::ConnectTM::Request>();
  request->server = tm_msgs::srv::ConnectTM::Request::TMSVR;
  request->reconnect = true;
  request->timeout = 0;
  request->timeval = 0;
  send_service(connectSctClient, request);
}
void RosPage::send_svr_as_re_connect(){
  auto request = std::make_shared<tm_msgs::srv::ConnectTM::Request>();
  request->server = tm_msgs::srv::ConnectTM::Request::TMSCT;
  request->reconnect = true;
  request->timeout = 0;
  request->timeval = 0;
  
  send_service(connectSvrClient, request);
}
void RosPage::change_control_box_io_button(){
  std::cout<<"ready to send io"<<std::endl;
  rclcpp::Client<tm_msgs::srv::SetIO>::SharedPtr client =
    node->create_client<tm_msgs::srv::SetIO>("set_io");
  
  auto request = std::make_shared<tm_msgs::srv::SetIO::Request>();
  request->module = tm_msgs::srv::SetIO::Request::MODULE_CONTROLBOX;
  request->type = tm_msgs::srv::SetIO::Request::TYPE_DIGITAL_OUT;
  request->pin = 0;
  if(!lastStatus){
    request->state = tm_msgs::srv::SetIO::Request::STATE_ON;
    std::cout<<"set on"<<std::endl;
  } else{
    request->state = tm_msgs::srv::SetIO::Request::STATE_OFF;
    std::cout<<"set off"<<std::endl;
  }
  lastStatus = !lastStatus;
  
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  
  if(result.get()->ok){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OK");
  } else{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"not OK");
  }
  std::cout<<"success send io"<<std::endl;
}
RosPage::RosPage(std::string nodeName)
 :lastStatus(false)
 {
  node = rclcpp::Node::make_shared(nodeName);
  initial_subscriber();
  initial_client();
}
RosPage::~RosPage(){
  rclcpp::shutdown();
  std::cout<<"call ~RosPage"<<std::endl;
}
void RosPage::run(){
  rclcpp::spin(node);
}