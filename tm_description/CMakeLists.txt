cmake_minimum_required(VERSION 3.10.2)
project(tm_description)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY meshes urdf xacro 
    DESTINATION share/${PROJECT_NAME})
