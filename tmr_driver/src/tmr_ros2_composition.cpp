#include "tmr_driver/tmr_ros2_svr.h"
#include "tmr_driver/tmr_ros2_sct.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  bool is_fake = true;
  std::string host;
  if (argc > 1) {
    host = argv[1];
    if (host.find("ip:=") != std::string::npos) {
      host.replace(host.begin(), host.begin() + 4, "");
      is_fake = false;
    }
    if (host.find("robot_ip:=") != std::string::npos) {
      host.replace(host.begin(), host.begin() + 10, "");
      is_fake = false;
    }
  }
  if (is_fake) {
    std::cout << "No. ip or robot_ip, is_fake:=true\n";
  }

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  //std::condition_variable sct_cv;
  tmr::Driver iface(host, nullptr, nullptr);

  auto tm_svr = std::make_shared<TmSvrRos2>(options, iface, is_fake);
  exec.add_node(tm_svr);
  auto tm_sct = std::make_shared<TmSctRos2>(options, iface, is_fake);
  exec.add_node(tm_sct);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}