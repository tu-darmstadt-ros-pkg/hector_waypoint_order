cmake_minimum_required(VERSION 3.1)
project(hector_waypoint_order)

set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pluginlib
  roscpp
  message_generation
  rosbag
  visualization_msgs
  )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS graph)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CostMapEntry.msg
  Costmap.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs nav_msgs pluginlib roscpp message_runtime rosbag visualization_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


set(HEADER_FILES

  include/hector_waypoint_order/waypoint_order_computer_base.h
  include/hector_waypoint_order/utils/utils.h
  include/hector_waypoint_order/utils/file_utils.h

  include/hector_waypoint_order/cost_computation/cost_computer_base.h
  include/hector_waypoint_order/cost_computation/path_cost_computer.h

  include/hector_waypoint_order/cost_computation/path_planner/path_planner_base.h

  include/hector_waypoint_order/tsp_solver/boost_tsp_solver.h
  )

set(SOURCES
  src/waypoint_order_computer_base.cpp

  src/cost_computation/cost_computer_base.cpp
  src/cost_computation/path_cost_computer.cpp

  src/cost_computation/path_planner/path_planner_base.cpp

  src/tsp_solver/boost_tsp_solver.cpp
  )


## Declare a C++ library
add_library(${PROJECT_NAME} ${HEADER_FILES} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})


#############
## Install ##
#############

#install(DIRECTORY
#  config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )




#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  include_directories(${rostest_INCLUDE_DIRS})

  ## Add gtest based cpp test target and link libraries
  add_rostest_gtest(test_boost_tsp_solver test/test_boost_tsp_solver.test test/test_boost_tsp_solver.cpp)
  target_link_libraries(test_boost_tsp_solver ${PROJECT_NAME} ${rostest_LIBRARIES})

endif ()
