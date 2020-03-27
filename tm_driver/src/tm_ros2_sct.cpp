
#include "tm_driver/tm_ros2_sct.h"

TmSctRos2::TmSctRos2(const rclcpp::NodeOptions &options, TmDriver &iface)
    : Node("tm_sct", options)
    , sct_(iface.sct)
{
    bool rb = sct_.start(5000);

    send_script_srv_ = create_service<tm_msgs::srv::SendScript>("send_script",
        std::bind(&TmSctRos2::send_script, this,
        std::placeholders::_1,
        std::placeholders::_2));
}

bool TmSctRos2::send_script(
    const std::shared_ptr<tm_msgs::srv::SendScript::Request> req,
    std::shared_ptr<tm_msgs::srv::SendScript::Response> res)
{
    bool rb = sct_.send_script_str(req->id, req->script) == TmCommRC::OK;
    res->success = rb;
    return rb;
}