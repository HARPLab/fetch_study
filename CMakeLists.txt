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
)

generate_dynamic_reconfigure_options(
   cfg/PathFollower.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS dynamic_reconfigure
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

target_link_libraries(
  ${PROJECT_NAME} 
    ${catkin_LIBRARIES}
)

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
