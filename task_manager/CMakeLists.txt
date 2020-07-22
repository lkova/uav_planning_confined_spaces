cmake_minimum_required(VERSION 2.8.3)
project(task_manager)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  message_generation  
  gazebo_msgs
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  mav_msgs
  tf
  ar_track_alvar_msgs
)

find_package(PCL REQUIRED)


catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES adaptive_planning hole_detection ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS message_runtime geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ../../../devel/include
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(hole_point src/hole_point.cpp)
target_link_libraries(hole_point ${catkin_LIBRARIES})
add_dependencies(hole_point ${catkin_EXPORTED_TARGETS})

add_executable(count_invalid src/count_invalid.cpp)
target_link_libraries(count_invalid ${catkin_LIBRARIES})
add_dependencies(count_invalid ${catkin_EXPORTED_TARGETS})

#add_executable(stopper src/stopper.cpp)
#target_link_libraries(stopper ${PCL_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(stopper ${catkin_EXPORTED_TARGETS})

#add_executable(tt src/tt.cpp)
#target_link_libraries(tt ${catkin_LIBRARIES})
#add_dependencies(tt ${catkin_EXPORTED_TARGETS})

#add_executable(turn src/turn.cpp)
#target_link_libraries(turn ${catkin_LIBRARIES})
#add_dependencies(turn ${catkin_EXPORTED_TARGETS})

add_executable(exploration src/exploration.cpp)
target_link_libraries(exploration ${catkin_LIBRARIES})
add_dependencies(exploration ${catkin_EXPORTED_TARGETS})

add_executable(correction src/correction.cpp)
target_link_libraries(correction ${catkin_LIBRARIES})
add_dependencies(correction ${catkin_EXPORTED_TARGETS})

add_executable(basement_correction src/basement_correction.cpp)
target_link_libraries(basement_correction ${catkin_LIBRARIES})
add_dependencies(basement_correction ${catkin_EXPORTED_TARGETS})

add_executable(test_basement src/test_basement.cpp)
target_link_libraries(test_basement ${catkin_LIBRARIES})
add_dependencies(test_basement ${catkin_EXPORTED_TARGETS})

add_executable(demo_waypoints src/demo_waypoints.cpp)
target_link_libraries(demo_waypoints ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
add_dependencies(demo_waypoints ${catkin_EXPORTED_TARGETS} ${PCL_LIBRARIES} )

add_executable(ar_transform src/ar_transform.cpp)
target_link_libraries(ar_transform ${catkin_LIBRARIES})
add_dependencies(ar_transform ${catkin_EXPORTED_TARGETS})

add_executable(ar_listener src/ar_listener.cpp)
target_link_libraries(ar_listener ${catkin_LIBRARIES})
add_dependencies(ar_listener ${catkin_EXPORTED_TARGETS})

add_executable(alvar src/alvar.cpp)
target_link_libraries(alvar ${catkin_LIBRARIES})
add_dependencies(alvar ${catkin_EXPORTED_TARGETS})

add_executable(absworld src/ar_markers_abs_w.cpp)
target_link_libraries(absworld ${catkin_LIBRARIES})
add_dependencies(absworld ${catkin_EXPORTED_TARGETS})
