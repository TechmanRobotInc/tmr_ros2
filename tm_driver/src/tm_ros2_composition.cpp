#include "tm_driver/tm_ros2_svr.h"
#include "tm_driver/tm_ros2_sct.h"
#include <rclcpp/rclcpp.hpp>

void debug_function_print(const char* msg){
  printf("%s[TM_DEBUG] %s\n%s", PRINT_CYAN.c_str(), msg, PRINT_RESET.c_str());
}
void info_function_print(const char* msg){
  printf("[TM_INFO] %s\n", msg);
}
void warn_function_print(const char* msg){
  printf("%s[TM_WARN] %s\n%s", PRINT_YELLOW.c_str(), msg, PRINT_RESET.c_str());
}
void error_function_print(const char* msg){
  printf("%s[TM_ERROR] %s\n%s", PRINT_RED.c_str(), msg, PRINT_RESET.c_str());
}
void fatal_function_print(const char* msg){
  printf("%s[TM_FATAL] %s\n%s", PRINT_GREEN.c_str(), msg, PRINT_RESET.c_str());
}

void ros_debug_print(const char* msg){
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),msg);
}
void ros_info_print(const char* msg){
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),msg);
}
void ros_warn_function_print(const char* msg){
  RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),msg);
}
void ros_error_print(const char* msg){
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),msg);
}
void ros_fatal_print(const char* msg){
  std::string str = msg;
  str = "[TM_FATAL] " + str;
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),str.c_str());
}
void ros_once_print(const char* msg){
  RCLCPP_INFO_STREAM_ONCE(rclcpp::get_logger("rclcpp"),msg);
}
void set_up_print_fuction(){
  set_up_print_debug_function(debug_function_print);
  set_up_print_info_function(info_function_print);
  set_up_print_warn_function(warn_function_print);
  set_up_print_error_function(error_function_print);
  set_up_print_fatal_function(fatal_function_print);
  set_up_print_once_function(default_print_once_function_print);
}
void set_up_ros_print_fuction(){
  set_up_print_debug_function(ros_debug_print);
  set_up_print_info_function(ros_info_print);
  set_up_print_warn_function(ros_warn_function_print);
  set_up_print_error_function(ros_error_print);
  set_up_print_fatal_function(ros_fatal_print);
  set_up_print_once_function(ros_once_print);
}

int main(int argc, char *argv[])
{
    // Force flush of the stdout buffer.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    set_up_ros_print_fuction();

    rclcpp::init(argc, argv);

    std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);
    
    if (args.size() < 2)
    {
        ros_fatal_print("Please provide the 'robot_ip_address'");
        rclcpp::shutdown();
        return 1;
    }

    const std::string& host = args[1];

    if (args.size() == 3){
      bool isSetNoLogPrint;
      std::istringstream(args[2]) >> std::boolalpha >> isSetNoLogPrint;
      if(isSetNoLogPrint){
        set_up_print_fuction();
      }
    }

    //std::condition_variable sct_cv;
    TmDriver iface(host, nullptr, nullptr);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("tm_driver_node");

    auto tm_svr = std::make_shared<TmSvrRos2>(node, iface, false);
    auto tm_sct = std::make_shared<TmSctRos2>(node, iface, false);
    rclcpp::spin(node);


    //iface.halt();

    rclcpp::shutdown();
    return 1;
}
