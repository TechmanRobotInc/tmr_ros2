//#pragma once

#ifndef TM_PRINT_H
#define TM_PRINT_H
#include <string>
#include <rclcpp/rclcpp.hpp>

void set_up_print_debug_function(void (*function_print)(char* fmt));
void set_up_print_info_function(void (*function_print)(char* fmt));
void set_up_print_warn_function(void (*function_print)(char* fmt));
void set_up_print_error_function(void (*function_print)(char* fmt));
void set_up_print_fatal_function(void (*function_print)(char* fmt));
void set_up_print_once_function(void (*function_print)(char* fmt));

void default_print_once_function_print(char* msg);

const std::string PRINT_RED("\033[0;31m");
const std::string PRINT_GREEN("\033[1;32m");
const std::string PRINT_YELLOW("\033[1;33m");
const std::string PRINT_CYAN("\033[0;36m");
const std::string PRINT_RESET("\033[0m");


/*int (*print_info_function)(const char* fmt, ...);
int (*print_warn_function)(const char* fmt, ...);
int (*print_error_function)(const char* fmt, ...);
int (*print_fatal_function)(const char* fmt, ...);
*/
int print_debug(const char* fmt, ...);
int print_info(const char* fmt, ...);
int print_warn(const char* fmt, ...);
int print_error(const char* fmt, ...);
int print_fatal(const char* fmt, ...);
int print_once(const char* fmt, ...);

void setup_print_debug(bool isPrintDebug);

#endif
