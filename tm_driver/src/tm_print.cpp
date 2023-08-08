#ifdef NO_INCLUDE_DIR
#include "tm_print.h"
#else
#include "tm_driver/tm_print.h"
#endif

//#include <stdio.h>
#include "iostream"
#include <stdarg.h>

#ifdef _WIN32
#define vsnprintf_s vsprintf_s
#else
#define vsnprintf_s vsnprintf
#endif

#ifdef ROS_BUILD
#include <ros/ros.h>
#endif
#ifdef ROS2_BUILD
//#include "rclcpp/rclcpp.hpp"
#endif

#define MAX_MSG_SIZE 256

void (*print_debug_function)(const char* fmt);
bool isSetPrintDebugFunction = false;
void (*print_info_function)(const char* fmt);
bool isSetPrintInfoFunction = false;
void (*print_warn_function)(const char* fmt);
bool isSetPrintWarnFunction = false;
void (*print_error_function)(const char* fmt);
bool isSetPrintErrorFunction = false;
void (*print_fatal_function)(const char* fmt);
bool isSetPrintFatalFunction = false;
void (*print_once_function)(const char* fmt);
bool isSetPrintOnceFunction = false;


std::set<std::string> printed_string;

bool isPrintDebugOnTerminal = true;
void setup_print_debug(bool isPrintDebug){
  isPrintDebugOnTerminal = isPrintDebug;
}

void default_debug_function_print(const char* msg){
  std::cout<<PRINT_CYAN<<"[DEBUG] "<<msg<<std::endl<<PRINT_RESET;
}
void default_print_info_function_print(const char* msg){
  std::cout<<"[INFO] "<<msg<<std::endl;
}
void default_print_warn_function_print(const char* msg){
  std::cout<<PRINT_YELLOW<<"[WARN] "<<msg<<std::endl<<PRINT_RESET;
}
void default_print_error_function_print(const char* msg){
  std::cout<<PRINT_RED<<"[ERROR] "<<msg<<std::endl<<PRINT_RESET;
}
void default_print_fatal_function_print(const char* msg){
  std::cout<<PRINT_GREEN<<"[FATAL] "<<msg<<std::endl<<PRINT_RESET;
}
void default_print_once_function_print(const char* msg){
  std::string str(msg);
  if (printed_string.count(str) == 0) {
    std::cout<<"[INFO_ONCE] "<< str << std::endl;
	printed_string.insert(str);
  }
}
int print_debug(const char* fmt, ...) {
  char msg[MAX_MSG_SIZE];
  int n;
  va_list vl;
  va_start(vl, fmt);
  n = vsnprintf_s(msg, MAX_MSG_SIZE, fmt, vl);
  va_end(vl);
  
  if(!isPrintDebugOnTerminal){
    return n;
  } else if(isSetPrintDebugFunction){
    print_debug_function(msg);
  } else{
    default_debug_function_print(msg);
  }
  return n;
}
int print_info(const char* fmt, ...) {
	char msg[MAX_MSG_SIZE];
	int n;
	va_list vl;
	va_start(vl, fmt);
	n = vsnprintf_s(msg, MAX_MSG_SIZE, fmt, vl);
	va_end(vl);
    if(isSetPrintInfoFunction){
        print_info_function(msg);
    } else{
        default_print_info_function_print(msg);
    }
	return n;
}
int print_warn(const char* fmt, ...) {
	char msg[MAX_MSG_SIZE];
	int n;
	va_list vl;
	va_start(vl, fmt);
	n = vsnprintf_s(msg, MAX_MSG_SIZE, fmt, vl);
	va_end(vl);
    if(isSetPrintWarnFunction){
		print_warn_function(msg);
	} else{
		default_print_warn_function_print(msg);
	}
	return n;
}
int print_error(const char* fmt, ...) {
	char msg[MAX_MSG_SIZE];
	int n;
	va_list vl;
	va_start(vl, fmt);
	n = vsnprintf_s(msg, MAX_MSG_SIZE, fmt, vl);
	va_end(vl);
    if(isSetPrintErrorFunction){
		print_error_function(msg);
	} else{
		default_print_error_function_print(msg);
	}
	return n;
}
int print_fatal(const char* fmt, ...) {
	char msg[MAX_MSG_SIZE];
	int n;
	va_list vl;
	va_start(vl, fmt);
	n = vsnprintf_s(msg, MAX_MSG_SIZE, fmt, vl);
	va_end(vl);
    if(isSetPrintFatalFunction){
		print_fatal_function(msg);
	} else{
		default_print_fatal_function_print(msg);
	}
	return n;
}
int print_once(const char* fmt, ...){
	char msg[MAX_MSG_SIZE];
	int n;
	va_list vl;
	va_start(vl, fmt);
	n = vsnprintf_s(msg, MAX_MSG_SIZE, fmt, vl);
	va_end(vl);
    if(isSetPrintOnceFunction){
		print_once_function(msg);
	} else{
		default_print_once_function_print(msg);
	}
	return n;
}


void set_up_print_debug_function(void (*function_print)(const char* fmt)){
  print_debug_function = function_print;
  isSetPrintDebugFunction = true;
}
void set_up_print_info_function(void (*function_print)(const char* fmt)){
  print_info_function = function_print;
  isSetPrintInfoFunction = true;
}
void set_up_print_warn_function(void (*function_print)(const char* fmt)){
  print_warn_function = function_print;
  isSetPrintWarnFunction = true;
}
void set_up_print_error_function(void (*function_print)(const char* fmt)){
  print_error_function = function_print;
  isSetPrintErrorFunction = true;
}
void set_up_print_fatal_function(void (*function_print)(const char* fmt)){
  print_fatal_function = function_print;
  isSetPrintFatalFunction = true;
}
void set_up_print_once_function(void (*function_print)(const char* fmt)){
  print_once_function = function_print;
  isSetPrintOnceFunction = true;
}
