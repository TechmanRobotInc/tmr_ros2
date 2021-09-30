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

# Please refer to https://moveit.ros.org/install-moveit2/source
# If the MoveIt2 module is not ready for installation, or the user wants to close the relative MoveIt2 module, the user can modify the "true" of moveit2_lib_auto_judge to "false" to compile. 
#set(moveit2_lib_auto_judge false)
set(moveit2_lib_auto_judge true)

# If the user enable this tmr_driver_message_print to print debug messages, more information will be displayed "1 package had stderr output: tmr_driver" on the screen. 
#set(tmr_driver_message_print true)  

# find dependencies
find_package(ament_cmake REQUIRED)
if (moveit2_lib_auto_judge)
find_package(moveit_ros_planning_interface QUIET)
endif()
if (moveit2_lib_auto_judge) 
if ( NOT moveit_ros_planning_interface_FOUND)
  if (tmr_driver_message_print)
  message("--- TMDriver build error : MoveIt2_LIB not found")
  #do you forget to source moveit2 relative setup.bash? example: & ~/COLCON_WS/install/setup.bash
  endif()  
endif (NOT moveit_ros_planning_interface_FOUND)
endif()
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
if (moveit2_lib_auto_judge) 
if (moveit_ros_planning_interface_FOUND)
find_package(control_msgs REQUIRED)
endif (moveit_ros_planning_interface_FOUND)
endif()
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

if (moveit2_lib_auto_judge) 
if (moveit_ros_planning_interface_FOUND)
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

#target_link_libraries(tmr_driver
#  tmrlib::tmrdriver
#)

install(TARGETS
  tmr_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
endif (moveit_ros_planning_interface_FOUND)
endif()

ament_package()
