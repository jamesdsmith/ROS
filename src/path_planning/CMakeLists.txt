cmake_minimum_required(VERSION 2.8.3)
project(path_planning)

if (COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  utils
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    utils
)

install(DIRECTORY include/${PACKAGE_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
