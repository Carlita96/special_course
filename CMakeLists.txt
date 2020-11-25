cmake_minimum_required(VERSION 3.0.2)
project(special_course)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
  tf_conversions
  tf2_ros
  tf2_eigen
  message_generation
)


find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)
message(STATUS "***** PCL version: ${PCL_VERSION} *****")
message(STATUS "***** Open CV version: ${OpenCV_VERSION} *****")
####

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ./include/tools/
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
)


add_executable(pcdRecorder source/recorder.cpp)
target_link_libraries(pcdRecorder ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${pluginlib_DIR} -lstdc++fs)
