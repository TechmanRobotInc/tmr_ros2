cmake_minimum_required(VERSION 3.10.2)
project(tm_moveit_config_tm12)

#find_package(catkin REQUIRED)
find_package(ament_cmake REQUIRED)
#catkin_package()
ament_package()
#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  PATTERN "setup_assistant.launch" EXCLUDE)
#install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
#  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
