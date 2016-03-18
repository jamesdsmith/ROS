cmake_minimum_required(VERSION 2.8.3)
project(utils)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include("FindGlog.cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  OpenCV REQUIRED
)

catkin_package(
  INCLUDE_DIRS include
 # LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    pcl_ros
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}  
  ${GLOG_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

install(DIRECTORY include/${PACKAGE_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

