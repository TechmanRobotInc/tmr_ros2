#include "tm_driver/tm_ros2_svr.h"
#include "tm_driver/tm_ros2_sct.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    // Force flush of the stdout buffer.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    
    std::string host;
    if (argc > 1) {
        host = argv[1];
    }
    else {
        rclcpp::shutdown();
    }

    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    std::condition_variable sct_cv;
    TmDriver iface(host, nullptr, &sct_cv);

    auto tm_svr = std::make_shared<TmSvrRos2>(options, iface);
    exec.add_node(tm_svr);
    auto tm_sct = std::make_shared<TmSctRos2>(options, iface);
    exec.add_node(tm_sct);

    exec.spin();

    iface.halt();

    rclcpp::shutdown();

    return 0;
}