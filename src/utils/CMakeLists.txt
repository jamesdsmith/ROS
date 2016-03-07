cmake_minimum_required(VERSION 2.8.3)
project(utils)

if (COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

#include("FindGlog.cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
 # OpenCV REQUIRED
)

catkin_package(
  INCLUDE_DIRS include
 # LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    pcl_ros
)

install(DIRECTORY include/${PACKAGE_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
