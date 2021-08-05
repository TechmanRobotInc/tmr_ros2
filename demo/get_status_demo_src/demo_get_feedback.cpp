#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/msg/feedback_state.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("demo_get_feedback")
    {
      subscription_ = this->create_subscription<tm_msgs::msg::FeedbackState>(
      "feedback_states", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    
    void topic_callback(const tm_msgs::msg::FeedbackState::SharedPtr msg) const
    {
      if(msg->joint_pos.size() == 6){
        RCLCPP_INFO_STREAM(this->get_logger(),"FeedbackState: joint pos = (" << 
                msg->joint_pos[0] << ", " << 
                msg->joint_pos[1] << ", " << 
                msg->joint_pos[2] << ", " <<
                msg->joint_pos[3] << ", " << 
                msg->joint_pos[4] << ", " << 
                msg->joint_pos[5] << ")"); 
      }
      
    }
    rclcpp::Subscription<tm_msgs::msg::FeedbackState>::SharedPtr subscription_;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
