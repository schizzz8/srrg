add_library(srrg_depth2laser_library SHARED
  depth2laser.h depth2laser.cpp
  )

target_link_libraries(srrg_depth2laser_library
  srrg_txt_io_library
  ${catkin_LIBRARIES}
  )
