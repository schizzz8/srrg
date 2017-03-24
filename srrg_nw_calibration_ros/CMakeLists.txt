cmake_minimum_required(VERSION 2.8.3)
project(srrg_nw_calibration_ros)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=gnu++11")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_BUILD_TYPE Release)

message(STATUS "PROCESSOR_TYPE: [${CMAKE_HOST_SYSTEM_PROCESSOR}]")
if (${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES armv7l)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -funsafe-math-optimizations")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -funsafe-math-optimizations")
  message("ENABLING ARM NEON OPTIMIZATIONS")
endif ()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  srrg_nw_calibration
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_filters
  tf
)

# Find Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS src
# LIBRARIES
#  CATKIN_DEPENDS srrg_core_ros
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIRS}/src
)

add_subdirectory(src)
