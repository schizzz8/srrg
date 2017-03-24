# **************************************
#
#  NEW_WORLD_CALIBRATION_ROS_NODE
#
# **************************************
add_executable(new_world_calibration_ros_node
        new_world_calibration_ros_node.cpp)
target_link_libraries(new_world_calibration_ros_node
                      nwc_ros_utils
                      ${catkin_LIBRARIES})
add_dependencies(new_world_calibration_ros_node
                 nwc_ros_utils
                 ${catkin_EXPORTED_TARGETS})


# **************************************
#
#  NEW_WORLD_CALIBRATION_ROS_AUTO_NODE
#
# **************************************
add_executable(new_world_calibration_ros_auto_node
        new_world_calibration_auto_node.cpp)
target_link_libraries(new_world_calibration_ros_auto_node
        nwc_ros_utils
        ${catkin_LIBRARIES})
add_dependencies(new_world_calibration_ros_auto_node
        nwc_ros_utils
        ${catkin_EXPORTED_TARGETS})

# **************************************
#
#  DATASET_DUMPER_NODE
#
# **************************************
add_executable(dataset_dumper_node
               dataset_dumper_node.cpp)
target_link_libraries(dataset_dumper_node
                      nwc_ros_utils
                      ${catkin_LIBRARIES})
add_dependencies(dataset_dumper_node
                 nwc_ros_utils
                 ${catkin_EXPORTED_TARGETS})