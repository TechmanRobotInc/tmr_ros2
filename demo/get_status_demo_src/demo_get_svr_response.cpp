#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tm_msgs/msg/svr_response.hpp>
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("demo_get_svr_response")
    {
      subscription_ = this->create_subscription<tm_msgs::msg::SvrResponse>(
      "svr_response", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    
    void topic_callback(const tm_msgs::msg::SvrResponse::SharedPtr msg) const
    {
      
      RCLCPP_INFO_STREAM(this->get_logger(), "SvrResponse: id is = " << msg->id << ", mode is " << (int)msg->mode << ", content is " << msg->content << ", error code is " << (int)msg->error_code); 
      
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
