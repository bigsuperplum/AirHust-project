cmake_minimum_required(VERSION 3.0.2)
project(vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 catkin 和 ROS 依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  mavros_msgs
  geometry_msgs
  nav_msgs
  tf
)

# 查找 OpenCV（仍需要用于图像接收和编码）
find_package(OpenCV REQUIRED)

# 新增：查找 curl 和 jsoncpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(CURL REQUIRED libcurl)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

# 声明 catkin 包
catkin_package(
  CATKIN_DEPENDS 
    roscpp std_msgs sensor_msgs cv_bridge 
    mavros_msgs geometry_msgs nav_msgs tf
)

# 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

# 编译可执行文件
add_executable(vision_node src/vision.cpp)

# 链接库
target_link_libraries(vision_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CURL_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
