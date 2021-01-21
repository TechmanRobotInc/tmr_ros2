cmake_minimum_required(VERSION 3.10.2)
project(tmr_description)

#find_package(catkin REQUIRED)
find_package(ament_cmake REQUIRED)
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tmr_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
#)
ament_package()
#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY xacro DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes urdf xacro DESTINATION share/${PROJECT_NAME})
