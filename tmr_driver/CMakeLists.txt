cmake_minimum_required(VERSION 3.5)
project(tmr_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DROS2_BUILD)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tmr_msgs REQUIRED)
find_package(tmrlib REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include
  #../tmrlib/include
  ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(tmr_driver
  src/tmr_ros2_composition.cpp
  src/tmr_ros2_sct.cpp
  src/tmr_ros2_svr.cpp
)

ament_target_dependencies(tmr_driver
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  rclcpp_action
  tmr_msgs
  tmrlib
)
target_link_libraries(tmr_driver
  tmrlib::tmrdriver
)

install(TARGETS
  tmr_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
