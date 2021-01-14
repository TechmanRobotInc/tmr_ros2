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

void (*print_debug_function)(char* fmt);
bool isSetPrintDebugFunction = false;

void default_debug_function_print(char* msg){
  std::cout<<PRINT_CYAN<<"[DEBUG] "<<msg<<std::endl<<PRINT_RESET;
}

int print_debug(const char* fmt, ...) {

	
	char msg[MAX_MSG_SIZE];
	int n;
	va_list vl;
	va_start(vl, fmt);
	n = vsnprintf_s(msg, MAX_MSG_SIZE, fmt, vl);
	va_end(vl);
  if(isSetPrintDebugFunction){
    print_debug_function(msg);
  } else{
    default_debug_function_print(msg);
  }
  return n;
}

void (*print_info_function)(char* fmt);
bool isSetPrintInfoFunction = false;

void default_print_info_function_print(char* msg){
  std::cout<<"[INFO] "<<msg<<std::endl;
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

void (*print_warn_function)(char* fmt);
bool isSetPrintWarnFunction = false;

void default_print_warn_function_print(char* msg){
  std::cout<<PRINT_YELLOW<<"[WARN] "<<msg<<std::endl<<PRINT_RESET;
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

void (*print_error_function)(char* fmt);
bool isSetPrintErrorFunction = false;

void default_print_error_function_print(char* msg){
  std::cout<<PRINT_RED<<"[ERROR] "<<msg<<std::endl<<PRINT_RESET;
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

void (*print_fatal_function)(char* fmt);
bool isSetPrintFatalFunction = false;

void default_print_fatal_function_print(char* msg){
  std::cout<<PRINT_GREEN<<"[FATAL] "<<msg<<std::endl<<PRINT_RESET;
}
int print_fatal(const char* fmt, ...) {
	char msg[MAX_MSG_SIZE];
	int n;
	va_list vl;
	va_start(vl, fmt);
	n = vsnprintf_s(msg, MAX_MSG_SIZE, fmt, vl);
	va_end(vl);
    if(isSetPrintErrorFunction){
		print_fatal_function(msg);
	} else{
		default_print_fatal_function_print(msg);
	}
	return n;

}

void set_up_print_debug_function(void (*function_print)(char* fmt)){
  print_debug_function = function_print;
  isSetPrintDebugFunction = true;
}
void set_up_print_info_function(void (*function_print)(char* fmt)){
  print_info_function = function_print;
  isSetPrintInfoFunction = true;
}
void set_up_print_warn_function(void (*function_print)(char* fmt)){
  print_warn_function = function_print;
  isSetPrintWarnFunction = true;
}
void set_up_print_error_function(void (*function_print)(char* fmt)){
  print_error_function = function_print;
  isSetPrintErrorFunction = true;
}
void set_up_print_fatal_function(void (*function_print)(char* fmt)){
  print_fatal_function = function_print;
  isSetPrintFatalFunction = true;
}
