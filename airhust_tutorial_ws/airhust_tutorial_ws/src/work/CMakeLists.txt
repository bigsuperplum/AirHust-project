cmake_minimum_required(VERSION 3.0.2)
project(work)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(OpenCV REQUIRED)

find_package(PkgConfig)
pkg_check_modules(JSONC REQUIRED json-c)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  cv_bridge
  nav_msgs
  tf
)
# 新增：查找 curl 和 jsoncpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(CURL REQUIRED libcurl)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${JSONC_INCLUDE_DIRS}
)


add_executable(task src/task.cpp)
target_link_libraries(task ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${JSONC_LIBRARIES} ${CURL_LIBRARIES} )


