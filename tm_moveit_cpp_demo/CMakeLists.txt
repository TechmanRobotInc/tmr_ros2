cmake_minimum_required(VERSION 3.5)
project(tm_moveit_cpp_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Please refer to https://moveit.ros.org/install-moveit2/source
# If the MoveIt2 module is not ready for installation, or the user wants to close the relative MoveIt2 module, the user can modify the "true" of moveit2_lib_auto_judge to "false" to compile. 
# set(moveit2_lib_auto_judge false)
set(moveit2_lib_auto_judge true)

# If the user enable this tm_moveit_cpp_demo_message_print to print debug messages, more information will be displayed "1 package had stderr output: tm_moveit_cpp_demo" on the screen. 
# set(tm_moveit_cpp_demo_message_print true)  

# find dependencies
find_package(ament_cmake REQUIRED)
if (moveit2_lib_auto_judge)
find_package(moveit_ros_planning_interface QUIET)
endif()
# This shouldn't be necessary (required by moveit_simple_controller_manager)
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if (moveit2_lib_auto_judge) 
if ( NOT moveit_ros_planning_interface_FOUND)
  if (tm_moveit_cpp_demo_message_print)
  message("${PROJECT_NAME}:--- TMDriver build error : MoveIt2_LIB not found")
  #do you forget to source moveit2 relative setup.bash? example: & ~/COLCON_WS/install/setup.bash
  endif()  
endif (NOT moveit_ros_planning_interface_FOUND)

if (moveit_ros_planning_interface_FOUND)
  if (tm_moveit_cpp_demo_message_print)
  message("--- TMDriver message : MoveIt2_LIB is found")
  endif()

  add_executable(run_moveit_cpp src/run_moveit_cpp.cpp)
  ament_target_dependencies(run_moveit_cpp
    moveit_ros_planning_interface
    Boost
  )
  install(TARGETS run_moveit_cpp
    EXPORT export_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )
endif (moveit_ros_planning_interface_FOUND)
endif()

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )

ament_package()
