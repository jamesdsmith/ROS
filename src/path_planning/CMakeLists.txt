cmake_minimum_required(VERSION 2.8.3)
project(path_planning)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include("FindGlog.cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  utils
  OpenCV REQUIRED
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    utils
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

file(GLOB_RECURSE path_headers include/path_planning/*.h)
add_library(path_planning ${path_headers})
set_target_properties(path_planning PROPERTIES LINKER_LANGUAGE CXX)

# Compile all tests.
file(GLOB test_srcs test/*.cpp)
foreach(test ${test_srcs})
  get_filename_component(test_no_ext ${test} NAME_WE)
  message("Including test   \"${BoldBlue}${test_no_ext}${ColorReset}\".")
  catkin_add_gtest(${test_no_ext} ${test})
  target_link_libraries(${test_no_ext}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${GLOG_LIBRARIES}
  )
endforeach()

# Define a variable storing the path to test data.
add_definitions(-DPATH_TEST_DATA_DIR="test/test_data")

# If you want a test to generate temporary files (ie, testing file io) then store 
# them in this ignored folder
add_definitions(-DGENERATED_TEST_DATA_DIR="test/test_data/generated")
