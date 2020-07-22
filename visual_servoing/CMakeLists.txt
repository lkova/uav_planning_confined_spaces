cmake_minimum_required(VERSION 2.8.3)
project(visual_servoing)

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
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hole_detection
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ../../../devel/include
)


add_library(visual_servoing_core
  src/VisualServoing.cpp
)

link_directories(${PCL_LIBRARY_DIRS} {OpenCV_LIB_DIR})
add_definitions(${PCL_DEFINITIONS})


add_executable(visual_servoing 
  src/visual_servoing_package_node.cpp 
  src/VisualServoingPackage.cpp
)

add_executable(testi 
  src/testi.cpp 
)

target_link_libraries(visual_servoing_core
  ${PCL_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(visual_servoing
    visual_servoing_core
    ${PCL_LIBRARIES} 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)


#add_executable(plane src/plane.cpp)
#target_link_libraries(plane ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(
    TARGETS visual_servoing
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/visual_servoing/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)