#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/msg/svr_response.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("tm_feedback_state")
    {
      subscription_ = this->create_subscription<tm_msgs::msg::SvrResponse>(
      "svr_response", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    
    void topic_callback(const tm_msgs::msg::SvrResponse::SharedPtr msg) const
    {
      
      RCLCPP_INFO(this->get_logger(), "the id is %s, mode is %d, content is %s, error_code is %d", msg->id, msg->mode ,msg->content, msg->error_code);
      
    }
    rclcpp::Subscription<tm_msgs::msg::SvrResponse>::SharedPtr subscription_;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}