cmake_minimum_required(VERSION 2.8.12)
project(fetch_study)

find_package(catkin REQUIRED)
catkin_package()

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

install(
  PROGRAMS scripts/follow_waypoints.py
  PROGRAMS scripts/demo_path.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(fetch_study planner/timed_path_follower.cpp)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch)
  roslaunch_add_file_check(launch/fetch_nav.launch)
  roslaunch_add_file_check(launch/study_nav.launch)
endif()
