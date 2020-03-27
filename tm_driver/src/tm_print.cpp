#ifdef NO_INCLUDE_DIR
#include "tm_print.h"
#else
#include "tm_driver/tm_print.h"
#endif

#include <stdio.h>
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

int print_debug(const char* fmt, ...) {
	char msg[MAX_MSG_SIZE];
	int n;
	va_list vl;
	va_start(vl, fmt);
	n = vsnprintf_s(msg, MAX_MSG_SIZE, fmt, vl);
	va_end(vl);
#ifdef ROS_BUILD
	ROS_DEBUG("%s", msg);
#else
#ifdef ROS2_BUILD
#else
	printf("[DEBUG] %s\n", msg);
#endif
#endif
	return n;
}

int print_info(const char* fmt, ...) {
	char msg[MAX_MSG_SIZE];
	int n;
	va_list vl;
	va_start(vl, fmt);
	n = vsnprintf_s(msg, MAX_MSG_SIZE, fmt, vl);
	va_end(vl);
#ifdef ROS_BUILD
	ROS_INFO("%s", msg);
#else
#ifdef ROS2_BUILD
	printf("[INFO] [tm_driver]: %s\n", msg);
#else
	printf("[ INFO] %s\n", msg);
#endif
#endif
	return n;
}

int print_warn(const char* fmt, ...) {
	char msg[MAX_MSG_SIZE];
	int n;
	va_list vl;
	va_start(vl, fmt);
	n = vsnprintf_s(msg, MAX_MSG_SIZE, fmt, vl);
	va_end(vl);
#ifdef ROS_BUILD
	ROS_WARN("%s", msg);
#else
#ifdef ROS2_BUILD
	printf("[WARN] [tm_driver]: %s\n", msg);
#else
	printf("[ WARN] %s\n", msg);
#endif
#endif
	return n;
}

int print_error(const char* fmt, ...) {
	char msg[MAX_MSG_SIZE];
	int n;
	va_list vl;
	va_start(vl, fmt);
	n = vsnprintf_s(msg, MAX_MSG_SIZE, fmt, vl);
	va_end(vl);
#ifdef ROS_BUILD
	ROS_ERROR("%s", msg);
#else
#ifdef ROS2_BUILD
	printf("[ERROR] [tm_driver]: %s\n", msg);
#else
	printf("[ERROR] %s\n", msg);
#endif
#endif
	return n;
}

int print_fatal(const char* fmt, ...) {
	char msg[MAX_MSG_SIZE];
	int n;
	va_list vl;
	va_start(vl, fmt);
	n = vsnprintf_s(msg, MAX_MSG_SIZE, fmt, vl);
	va_end(vl);
#ifdef ROS_BUILD
	ROS_FATAL("%s", msg);
	ros::shutdown();
#else
#ifdef ROS2_BUILD
	printf("[FATAL] [tm_driver]: %s\n", msg);
#else
	printf("[FATAL] %s\n", msg);
#endif
#endif
	return n;
}