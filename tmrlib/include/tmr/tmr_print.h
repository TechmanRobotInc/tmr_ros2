#pragma once

#ifdef ROS_BUILD
// ROS_BUILD
#include <ros/console.h>

#define tmr_DEBUG_STREAM ROS_DEBUG_STREAM
#define tmr_INFO_STREAM  ROS_INFO_STREAM
#define tmr_WARN_STREAM  ROS_WARN_STREAM
#define tmr_ERROR_STREAM ROS_ERROR_STREAM

#else
#ifdef ROS2_BUILD
// ROS2_BUILD
//#include "rclcpp/rclcpp.hpp"
#include <iostream>

#define tmr_DEBUG_STREAM(msg) std::cout << "[DEBUG] " << msg << "\n"
#define tmr_INFO_STREAM(msg)  std::cout << "[ INFO] " << msg << "\n"
#define tmr_WARN_STREAM(msg)  std::cout << "[ WARN] " << msg << "\n"
#define tmr_ERROR_STREAM(msg) std::cout << "[ERROR] " << msg << "\n"

#else
// NOT ROS
#include <iostream>

#define tmr_DEBUG_STREAM(msg) std::cout << "[DEBUG] " << msg << "\n"
#define tmr_INFO_STREAM(msg)  std::cout << "[ INFO] " << msg << "\n"
#define tmr_WARN_STREAM(msg)  std::cout << "[ WARN] " << msg << "\n"
#define tmr_ERROR_STREAM(msg) std::cout << "[ERROR] " << msg << "\n"

#endif
#endif
