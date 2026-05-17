cmake_minimum_required(VERSION 3.8)
project(mobile_robot_gz_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  configs
  params
  launch
  worlds
  models
  maps
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
