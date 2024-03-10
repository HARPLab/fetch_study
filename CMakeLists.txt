cmake_minimum_required(VERSION 2.8.12)
project(fetch_study)

find_package(catkin REQUIRED 
  roscpp 
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_core
  pluginlib
  roscpp
  lattice_planner
  pcl_conversions
  tf2
  tf2_ros
)

generate_dynamic_reconfigure_options(
   cfg/PathFollower.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS dynamic_reconfigure
  CATKIN_DEPENDS lattice_planner
  CATKIN_DEPENDS base_local_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/timed_path_follower.cpp
)

add_dependencies(
  ${PROJECT_NAME} 
    ${PROJECT_NAME}_gencfg
)

include_directories(fetch_study/cmake/cfgbuild.cmake) 

install(
  DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY aux_libraries
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
 
target_link_libraries(fetch_study ${catkin_LIBRARIES})

install(
  PROGRAMS scripts/follow_waypoints.py
  PROGRAMS scripts/demo_path.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch)
  roslaunch_add_file_check(launch/fetch_nav.launch)
  roslaunch_add_file_check(launch/study_nav.launch)
endif()
