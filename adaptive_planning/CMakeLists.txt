cmake_minimum_required(VERSION 2.8.3)
project(adaptive_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  octomap_msgs
  octomap_ros
  gazebo_msgs
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  mav_msgs
)

find_package(octomap REQUIRED)
find_package(fcl REQUIRED)
find_package(OMPL REQUIRED)
find_package(PCL REQUIRED)


catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES adaptive_planning ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS message_runtime octomap_msgs octomap_ros geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
  ${fcl_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_libraries(${OCTOMAP_LIBRARIES} ${fcl_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS})

add_library(plannerLib src/Planner.cpp src/PathFinder.cpp)
add_executable(planner src/planner_node.cpp src/Planner.cpp src/PathFinder.cpp)
add_dependencies(plannerLib ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(planner  
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${fcl_LIBRARIES}
  ${OMPL_LIBRARIES}
  fcl
)

target_link_libraries(plannerLib  
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${fcl_LIBRARIES}
  ${OMPL_LIBRARIES}
  fcl
)

#install(TARGETS hole_point
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

cs_install()
cs_export(LIBRARIES ${OMPL_LIBRARIES})
