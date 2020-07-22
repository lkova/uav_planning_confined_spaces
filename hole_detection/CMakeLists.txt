cmake_minimum_required(VERSION 2.8.3)
project(hole_detection)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  cv_bridge
  roscpp
  sensor_msgs
  image_transport
  image_geometry
  tf
  geometry_msgs
  message_generation
)

find_package(PCL REQUIRED)
find_package(OpenCV)

add_message_files(
   FILES
   Hole.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
#  LIBRARIES my_pcl_tutorial
   CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)



add_library(detection_core
  src/HoleDetection.cpp
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(detection 
  src/hole_detection_package_node.cpp 
  src/HoleDetectionPackage.cpp
)

target_link_libraries(detection_core
  ${PCL_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(detection
    detection_core
    ${PCL_LIBRARIES} 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)


#add_executable(transform_to_world src/transform_to_world.cpp)
#target_link_libraries(transform_to_world ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(ransac src/ransac.cpp)
#target_link_libraries(ransac ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(
    TARGETS detection
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/detection/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)