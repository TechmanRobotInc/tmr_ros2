#include "tm_driver.h"
#include "tm_print.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


class TmSvrRos2 : public rclcpp::Node
{
public:
    TmSvrCommunication &svr_;
    TmRobotState &state_;
    
    std::thread pub_thread_;

    struct PubMsg {
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
        sensor_msgs::msg::JointState joint_msg;
    } pm_;

public:
    explicit TmSvrRos2(const rclcpp::NodeOptions &options, TmDriver &iface);

protected:
    void publish_msg();
    void publisher();
};