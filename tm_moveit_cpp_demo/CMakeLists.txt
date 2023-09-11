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
# If the MoveIt2 module is not ready for installation, or the user wants to close the relative MoveIt2 module, the user can modify the "TRUE" of 'TM_MOVEIT_AUTO_DETECT' to "FALSE" to compile.
option(TM_MOVEIT_AUTO_DETECT "Try to detect moveit2" TRUE)
set(TM_MOVEIT_CPP_DEMO_INCLUDE_MOVEIT ${TM_MOVEIT_AUTO_DETECT})

# If the user enable this tm_moveit_cpp_demo_message_print to print debug messages, more information will be displayed "1 package had stderr output: tm_moveit_cpp_demo" on the screen. 
# set(tm_moveit_cpp_demo_message_print true)  

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

if (TM_MOVEIT_AUTO_DETECT)
  find_package(moveit_ros_planning_interface QUIET)
  if (NOT moveit_ros_planning_interface_FOUND)
    set(TM_MOVEIT_CPP_DEMO_INCLUDE_MOVEIT FALSE)
    if (tm_moveit_cpp_demo_message_print)
      message("${PROJECT_NAME}:--- TMDriver build error : MoveIt2_LIB not found")
      #do you forget to source moveit2 relative setup.bash? example: & ~/COLCON_WS/install/setup.bash
    endif()
  endif (NOT moveit_ros_planning_interface_FOUND)
endif(TM_MOVEIT_AUTO_DETECT)

if (TM_MOVEIT_CPP_DEMO_INCLUDE_MOVEIT)
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
endif (TM_MOVEIT_CPP_DEMO_INCLUDE_MOVEIT)

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )

ament_package()
