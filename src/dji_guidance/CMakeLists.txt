cmake_minimum_required(VERSION 2.8.3)
project(dji_guidance)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  set_camera_id.msg
  set_exposure_param.msg
  depth_image.msg
  stereo_pair.msg
  multi_image.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

MESSAGE("System is: " ${CMAKE_SYSTEM_PROCESSOR})
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	if (CMAKE_SIZEOF_VOID_P EQUAL 4)
		MESSAGE("-- 32bit detected")	
		link_directories(${PROJECT_SOURCE_DIR}/lib/x86)
	elseif (CMAKE_SIZEOF_VOID_P EQUAL 8)
		MESSAGE("-- 64bit detected")
		link_directories(${PROJECT_SOURCE_DIR}/lib/x64)
	endif ()
else()
	MESSAGE("-- Non-linux platform detected but sorry we do not support :D")
endif ()

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7l" )
	MESSAGE("-- " ${CMAKE_SYSTEM_PROCESSOR} " detected")
	link_directories(${PROJECT_SOURCE_DIR}/lib/XU3)
else ()
endif ()

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(guidanceNode ${PROJECT_SOURCE_DIR}/src/GuidanceNode.cpp ${PROJECT_SOURCE_DIR}/src/DJI_utility.cpp)
add_executable(guidanceDepthNode ${PROJECT_SOURCE_DIR}/src/GuidanceDepthNode.cpp ${PROJECT_SOURCE_DIR}/src/DJI_utility.cpp)
add_executable(guidanceNodeTest ${PROJECT_SOURCE_DIR}/src/GuidanceNodeTest.cpp ${PROJECT_SOURCE_DIR}/src/DJI_utility.cpp)
add_executable(guidanceDepthNodeViewer ${PROJECT_SOURCE_DIR}/src/GuidanceDepthNodeViewer.cpp ${PROJECT_SOURCE_DIR}/src/DJI_utility.cpp)
add_executable(guidanceNodeCalibration ${PROJECT_SOURCE_DIR}/src/GuidanceNodeCalibration.cpp ${PROJECT_SOURCE_DIR}/src/DJI_utility.cpp)

target_link_libraries(guidanceNode DJI_guidance usb-1.0 ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(guidanceDepthNode DJI_guidance usb-1.0 ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(guidanceNodeTest DJI_guidance ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(guidanceDepthNodeViewer DJI_guidance ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(guidanceNodeCalibration DJI_guidance usb-1.0 yaml-cpp ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(guidanceNode dji_guidance_generate_messages_cpp)
add_dependencies(guidanceDepthNode dji_guidance_generate_messages_cpp)
add_dependencies(guidanceNodeTest dji_guidance_generate_messages_cpp)
add_dependencies(guidanceDepthNodeViewer dji_guidance_generate_messages_cpp)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")