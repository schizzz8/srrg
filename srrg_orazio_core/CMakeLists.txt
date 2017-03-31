cmake_minimum_required(VERSION 2.8.3)
project(srrg_orazio_core)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=gnu++11 -Wall")

find_package(srrg_cmake_modules REQUIRED)
set(CMAKE_MODULE_PATH ${srrg_cmake_modules_INCLUDE_DIRS})

message("modules in ${CMAKE_MODULE_PATH}")

FIND_PACKAGE(LibWEBSOCKETS REQUIRED)

execute_process(COMMAND lsb_release -rs
    OUTPUT_VARIABLE LSB_RELEASE_VERSION
)

message("UBUNTU VERSION" ${LSB_RELEASE_VERSION})
if( LSB_RELEASE_VERSION MATCHES "16.04" ) 
  add_definitions(-D__UBUNTU_16_04__)
endif ()


FIND_PACKAGE(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES orazio_core_library
#  CATKIN_DEPENDS srrg_boss srrg_core srrg_core_map
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)

add_subdirectory(src)
