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
    //std::cout<<"host is "<<host<<std::endl;
    if (host.find("robot_ip:=") != std::string::npos) {
      //std::cout<<"robot_ip is work"<<std::endl;
      host.replace(host.begin(), host.begin() + 10, "");
      is_fake = false;
    } else if (host.find("ip:=") != std::string::npos) {
      //std::cout<<"ip is work"<<std::endl;
      host.replace(host.begin(), host.begin() + 4, "");
      is_fake = false;
    } else{
      //std::cout<<"ip is not found, use fake robot"<<std::endl;
    }    
  }
  if (is_fake) {
    std::cout << "only ip or robot_ip support, but your type is "<<host<<std::endl;
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
