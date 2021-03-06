cmake_minimum_required(VERSION 2.8.3)
project(lwr_testing)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf
  tf_conversions
  message_runtime
  pcl_ros
  kdl_parser
  lwr_controllers
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_INCLUDE_PATH}
)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/cmake/Modules/")
set(CMAKE_INCLUDE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include/")

#PCL
find_package(PCL REQUIRED  )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#OpenCV
find_package(OpenCV REQUIRED)
if (OPENCV_FOUND)
  set(HAVE_OPENCV ON)
  include_directories(SYSTEM ${OPENCV_INCLUDE_DIRS})
  message("OPENCV INCLUDE DIR:")
  message(${OpenCV_LIBS})
endif(OPENCV_FOUND)

#Aruco
set(ARUCO_LIBS /usr/local/lib/libaruco.so)

#VisionSystemLegacy
#set(VISION_SYSTEM_LIBS /usr/local/lib/libVisionSystemLegacy.so)


catkin_package(
   CATKIN_DEPENDS
   message_runtime
   geometry_msgs
   std_msgs
)


#SOPHUS
#set(SOPHUS_INCLUDE_DIRS /usr/local/include/sophus)
#include_directories(${SOPHUS_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#    MarkerImageService.srv
# )


# lwr hw fri node
add_executable(lwr_test  src/lwr_test.cpp)
target_link_libraries(lwr_test ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#add_dependencies(lwr_test lwr_testing_generate_messages_cpp)
