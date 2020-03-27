#include "tm_driver.h"
#include "tm_print.h"

#include "rclcpp/rclcpp.hpp"

#include "tm_msgs/srv/send_script.hpp"
#include "tm_msgs/srv/set_event.hpp"
#include "tm_msgs/srv/set_io.hpp"
#include "tm_msgs/srv/set_positions.hpp"


class TmSctRos2 : public rclcpp::Node
{
public:
    TmSctCommunication &sct_;

    rclcpp::Service<tm_msgs::srv::SendScript>::SharedPtr send_script_srv_;

public:
    explicit TmSctRos2(const rclcpp::NodeOptions &options, TmDriver &iface);

public:
    bool send_script(
        const std::shared_ptr<tm_msgs::srv::SendScript::Request> req,
        std::shared_ptr<tm_msgs::srv::SendScript::Response> res);


};