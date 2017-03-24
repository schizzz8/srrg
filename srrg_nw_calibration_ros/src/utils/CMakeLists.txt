add_library(nwc_ros_utils
             utils.h utils.cpp)

target_link_libraries(nwc_ros_utils
                      ${catkin_LIBRARIES})

add_dependencies(nwc_ros_utils
                 ${catkin_EXPORTED_TARGETS})