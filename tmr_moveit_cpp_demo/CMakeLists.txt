cmake_minimum_required(VERSION 3.5)
project(tmr_moveit_cpp_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Please refer to https://moveit.ros.org/install-moveit2/source
# If MoveIt2 built finished, the user's environment is already.
#set(install_moveit2_ready true)
# If MoveIt2 module not install ready or the user want to close relative MoveIt2 module, you can modify install_moveit2_ready 'true' to 'flase' to compile.
#set(install_moveit2_ready false)
set(install_moveit2_ready false)

# find dependencies
find_package(ament_cmake REQUIRED)
if(install_moveit2_ready)
find_package(moveit_ros_planning_interface REQUIRED)
endif()
# This shouldn't be necessary (required by moveit_simple_controller_manager)
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(install_moveit2_ready)
# message("--- TMDriver message : Assume the user have installed MoveIt2 ready")
if(NOT DEFINED moveit_ros_planning_interface_DIR)
  message("--- TMDriver build error : MoveIt2_LIB not found")
  
else()
  message("--- TMDriver message : MoveIt2_LIB is found")
  add_executable(run_moveit_cpp src/run_moveit_cpp.cpp)
  ament_target_dependencies(run_moveit_cpp
    moveit_ros_planning_interface
    Boost
  )

  install(TARGETS run_moveit_cpp run_moveit_cpp
    EXPORT export_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
