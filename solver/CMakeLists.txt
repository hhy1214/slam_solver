cmake_minimum_required(VERSION 2.8.3)
project(slamsolver LANGUAGES CXX)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    )

find_package(OpenCV REQUIRED)
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")
find_package(Ceres REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

find_package(glog REQUIRED)
set(glog_LIBS glog::glog)
message(INFO " mymap_VERSION: ${mymap_VERSION}")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

add_executable(test1
    src/test/hello.cpp
    src/pose.cpp
    src/slam_solver.cpp
    src/utility.cpp
    src/parameters.cpp
    )


target_link_libraries(test1 ${catkin_LIBRARIES} ${glog_LIBS} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${EIGEN3_INCLUDE_DIR}) 


