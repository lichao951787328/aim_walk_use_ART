cmake_minimum_required(VERSION 3.0.2)
project(mutiThread_perception_tag)
# set(CMAKE_CXX_STANDARD 17)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(FootstepPlanner REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)
find_package(PythonLibs REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/mutiThread_perception
  include/ART
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)
link_libraries(FootstepPlanner)
# add_compile_options(-pthread)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
aux_source_directory(src/ART DIR_SRCS)
message(${DIR_SRCS})
aux_source_directory(src DIR_SRCS)
message(${DIR_SRCS})

add_executable(aim_walk_ART ${DIR_SRCS})


target_link_libraries(aim_walk_ART
  ${EIGEN3_LIBRARIES}
  ${PYTHON_LIBRARIES}
  glog::glog
)
