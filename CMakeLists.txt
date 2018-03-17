cmake_minimum_required(VERSION 2.8.3)
project(rsband_local_planner)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    angles
    base_local_planner
    cmake_modules
    costmap_2d
    dynamic_reconfigure
    eband_local_planner
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf
    tf_conversions
)

find_package(OMPL REQUIRED)

include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(cfg/RSBandPlanner.cfg)

catkin_package(
  CATKIN_DEPENDS
    angles
    base_local_planner
    cmake_modules
    costmap_2d
    dynamic_reconfigure
    eband_local_planner
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf
    tf_conversions
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

link_directories(/usr/local/lib ${catkin_INCLUDE_DIRS})

# build the library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}_ros.cpp
  src/fuzzy_ptc.cpp
  src/reeds_shepp_planner.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${OMPL_LIBRARIES}
  fuzzylite
  yaml-cpp
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(
  FILES blp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
