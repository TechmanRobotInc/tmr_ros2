
#include "tm_driver/tm_ros2_svr.h"

TmSvrRos2::TmSvrRos2(const rclcpp::NodeOptions &options, TmDriver &iface)
    : Node("tm_svr", options)
    , svr_(iface.svr)
    , state_(iface.state)
{
    bool rb = svr_.start(5000);
    if (rb) {
        svr_.send_play_cmd();
    }

    pm_.joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    pub_thread_ = std::thread(std::bind(&TmSvrRos2::publisher, this));
}

void TmSvrRos2::publish_msg()
{
    pm_.joint_msg.header.stamp = rclcpp::Node::now();
    pm_.joint_msg.position = state_.joint_angle();
    pm_.joint_pub->publish(pm_.joint_msg);
}

void TmSvrRos2::publisher()
{
    print_info("TM_ROS: publisher thread begin");
    while (rclcpp::ok()) {
        bool reconnect = false;
        while (rclcpp::ok() && svr_.is_connected() && !reconnect) {
            auto rc = svr_.tmsvr_function();
            switch (rc) {
            case TmCommRC::OK:
                // if not running, send play command ..
                publish_msg();
                break;
            case TmCommRC::NOTREADY:
            case TmCommRC::NOTCONNECT:
            case TmCommRC::ERR:
                print_info("TM_ROS: (TM_SVR) rc=%d", int(rc));
                reconnect = true;
                break;
            default: break;
            }
        }
        svr_.Close();
        print_info("TM_ROS: (TM_SVR) reconnect in ");
        int cnt = 5;
        while (rclcpp::ok() && cnt > 0) {
            print_info("%d sec...", cnt);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            --cnt;
        }
        if (rclcpp::ok()) {
            print_info("TM_ROS: (TM_SVR) connect...");
            svr_.Connect(1000);
        }
    }
    svr_.Close();
    print_info("TM_ROS: publisher thread end");
}
