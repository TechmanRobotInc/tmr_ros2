#include "tm_driver/tm_ros2_svr.h"
#include "tm_driver/tm_ros2_sct.h"

#include "rclcpp/rclcpp.hpp"

void debug_function_print(char* msg){
  printf("%s[TM_DEBUG] %s\n%s", PRINT_CYAN.c_str(), msg, PRINT_RESET.c_str());
}
void info_function_print(char* msg){
  printf("[TM_INFO] %s\n", msg);
}
void warn_function_print(char* msg){
  printf("%s[TM_WARN] %s\n%s", PRINT_YELLOW.c_str(), msg, PRINT_RESET.c_str());
}
void error_function_print(char* msg){
  printf("%s[TM_ERROR] %s\n%s", PRINT_RED.c_str(), msg, PRINT_RESET.c_str());
}
void fatal_function_print(char* msg){
  printf("%s[TM_FATAL] %s\n%s", PRINT_GREEN.c_str(), msg, PRINT_RESET.c_str());
}
void set_up_print_fuction(){
  set_up_print_debug_function(debug_function_print);
  set_up_print_info_function(info_function_print);
  set_up_print_warn_function(warn_function_print);
  set_up_print_error_function(error_function_print);
  set_up_print_fatal_function(fatal_function_print);
}

int main(int argc, char *argv[])
{
    // Force flush of the stdout buffer.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    set_up_print_fuction();

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

    //std::condition_variable sct_cv;
    TmDriver iface(host, nullptr, nullptr);

    auto tm_svr = std::make_shared<TmSvrRos2>(options, iface);
    exec.add_node(tm_svr);
    auto tm_sct = std::make_shared<TmSctRos2>(options, iface);
    exec.add_node(tm_sct);

    exec.spin();

    //iface.halt();

    rclcpp::shutdown();
    std::cout<<"shut down is called"<<std::endl;
    return 1;
}
