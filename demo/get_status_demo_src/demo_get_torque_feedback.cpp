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
      if(msg->joint_tor_max.size() == 6){
        RCLCPP_INFO_STREAM(this->get_logger(),"the max torque is " << 
                msg->joint_tor_max[0] << ", " << 
                msg->joint_tor_max[1] << ", " << 
                msg->joint_tor_max[2] << ", " <<
                msg->joint_tor_max[3] << ", " << 
                msg->joint_tor_max[4] << ", " << 
                msg->joint_tor_max[5] << ")"); 
      }        
      if(msg->joint_tor_average.size() == 6){
        RCLCPP_INFO_STREAM(this->get_logger(),"the average torque is " << 
                msg->joint_tor_average[0] << ", " << 
                msg->joint_tor_average[1] << ", " << 
                msg->joint_tor_average[2] << ", " <<
                msg->joint_tor_average[3] << ", " << 
                msg->joint_tor_average[4] << ", " << 
                msg->joint_tor_average[5] << ")"); 
      }
      if(msg->joint_tor_min.size() == 6){
        RCLCPP_INFO_STREAM(this->get_logger(),"the min torque is " << 
                msg->joint_tor_min[0] << ", " << 
                msg->joint_tor_min[1] << ", " << 
                msg->joint_tor_min[2] << ", " <<
                msg->joint_tor_min[3] << ", " << 
                msg->joint_tor_min[4] << ", " << 
                msg->joint_tor_min[5] << ")"); 
        RCLCPP_INFO_STREAM(this->get_logger(),"---end this cycle---");
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
