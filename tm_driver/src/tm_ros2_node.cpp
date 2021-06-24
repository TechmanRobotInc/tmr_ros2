#include "tm_driver/tm_svr_communication.h"
#include "tm_driver/tm_driver.h"
#include "tm_driver/tm_print.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


class TmRos2Node : public rclcpp::Node
{
public:
    std::condition_variable svr_cv_;
    std::condition_variable sct_cv_;
    TmDriver iface_;

    std::thread pub_thread_;

    //rclcpp::S
    explicit TmRos2Node(const std::string &host);
    ~TmRos2Node();

    struct PubMsg {
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
        sensor_msgs::msg::JointState joint_msg;
    } pm_;

    void publish_msg();
    void publisher();
};

TmRos2Node::TmRos2Node(const std::string &host) : Node("tm_driver")
    , iface_(host, nullptr, &sct_cv_)
{
    RCLCPP_INFO(this->get_logger(), "TM_ROS: hello!");
    print_info("yoyo");

    iface_.start(5000);

    // topic
    pm_.joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    pub_thread_ = std::thread(std::bind(&TmRos2Node::publisher, this));

    // service
    //auto send_script_srv_ = this->create_service()

}

TmRos2Node::~TmRos2Node()
{
    if (pub_thread_.joinable()) pub_thread_.join();

    iface_.halt();
}

void TmRos2Node::publish_msg()
{
    pm_.joint_msg.header.stamp = rclcpp::Node::now();
    pm_.joint_msg.position = iface_.state.joint_angle();
    pm_.joint_pub->publish(pm_.joint_msg);

}

void TmRos2Node::publisher()
{
    print_info("TM_ROS: publisher thread begin");
    while (rclcpp::ok()) {
        bool reconnect = false;
        while (rclcpp::ok() && iface_.svr.is_connected() && !reconnect) {
            auto rc = iface_.svr.tmsvr_function();
            switch (rc) {
            case TmCommRC::OK:
                // if not running, send play command ..
                publish_msg();
                break;
            case TmCommRC::NOTREADY:
            case TmCommRC::NOTCONNECT:
            case TmCommRC::ERR:
                print_info("TM_ROS2: (TM_SVR) rc=%d", int(rc));
                reconnect = true;
                break;
            default: break;
            }
        }
        iface_.svr.close_socket();
        print_info("TM_ROS: (TM_SVR) reconnect in ");
        int cnt = 5;
        while (rclcpp::ok() && cnt > 0) {
            print_info("%d sec...", cnt);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            --cnt;
        }
        if (rclcpp::ok()) {
            print_info("TM_ROS: (TM_SVR) connect...");
            iface_.svr.connect_socket(1000);
        }
    }
    iface_.svr.close_socket();
    print_info("TM_ROS: publisher thread end");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::string host;
    if (argc > 1) {
        host = argv[1];
    }
    else {
        rclcpp::shutdown();
    }
    auto nh = std::make_shared<TmRos2Node>(host);
    rclcpp::spin(nh);
    rclcpp::shutdown();
    return 0;
}