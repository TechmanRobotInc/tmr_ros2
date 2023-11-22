#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tm_msgs/msg/sct_response.hpp>
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("demo_get_sct_response")
    {
      subscription_ = this->create_subscription<tm_msgs::msg::SctResponse>(
      "sct_response", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    
    void topic_callback(const tm_msgs::msg::SctResponse::SharedPtr msg) const
    {
      
      RCLCPP_INFO_STREAM(this->get_logger(),"SctResponse: id is = " << msg->id << ", script is " << msg->script);
      
    }
    rclcpp::Subscription<tm_msgs::msg::SctResponse>::SharedPtr subscription_;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
