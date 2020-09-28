cmake_minimum_required(VERSION 3.5)
project(tmrlib)

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
  ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(tmrdriver
  src/tmr_driver.cpp
  src/tmr_command.cpp
  src/tmr_tmsct_communication.cpp
  src/tmr_tmsvr_communication.cpp
  src/tmr_communication.cpp
  src/tmr_robot_state.cpp
  src/tmr_tmpacket.cpp
)

ament_target_dependencies(tmrdriver
  rclcpp
)

#ament_export_interfaces(export_tmrdriver HAS_LIBRARY_TARGET)
ament_export_targets(export_tmrdriver HAS_LIBRARY_TARGET)

#ament_export_dependencies(rclcpp)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS tmrdriver
  EXPORT export_tmrdriver
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
