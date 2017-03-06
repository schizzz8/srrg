add_library(srrg_txt_io_library SHARED
  base_image_message.cpp base_image_message.h
  base_message.cpp base_message.h
  base_sensor_message.cpp base_sensor_message.h
  imu_message.cpp imu_message.h
  joint_state_message.cpp joint_state_message.h
  laser_message.cpp laser_message.h
  message_dumper_trigger.cpp message_dumper_trigger.h
  message_enlister_trigger.cpp message_enlister_trigger.h
  message_factory.cpp message_factory.h
  message_reader.cpp message_reader.h
  message_seq_synchronizer.cpp message_seq_synchronizer.h
  message_timestamp_synchronizer.cpp message_timestamp_synchronizer.h
  message_writer.cpp message_writer.h
  pinhole_image_message.cpp pinhole_image_message.h
  pose_message.cpp pose_message.h
  property.cpp property.h
  sensor_message_sorter.cpp sensor_message_sorter.h
  spherical_image_message.cpp spherical_image_message.h
  static_transform_message.cpp static_transform_message.h
  static_transform_tree.cpp static_transform_tree.h
  tf_overrider_trigger.cpp tf_overrider_trigger.h  
)
target_link_libraries(srrg_txt_io_library
  ${OpenCV_LIBS}
)
