add_library(srrg_ros_wrappers_library 
  image_message_listener.cpp image_message_listener.h
  imu_interpolator.cpp imu_interpolator.h
  imu_message_listener.cpp imu_message_listener.h
  joint_state_message_listener.cpp joint_state_message_listener.h
  laser_message_listener.cpp laser_message_listener.h
  multiecholaser_message_listener.cpp multiecholaser_message_listener.h
  odom_tf_publisher.cpp odom_tf_publisher.h
  ros_utils.cpp ros_utils.h
  spherical_image_message_listener.cpp spherical_image_message_listener.h
)
target_link_libraries(srrg_ros_wrappers_library
  ${catkin_LIBRARIES}
)
