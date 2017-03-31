add_executable(srrg_depth2laser_gui_app srrg_depth2laser_gui_app.cpp)
target_link_libraries(srrg_depth2laser_gui_app 
  srrg_system_utils_library
  srrg_txt_io_library
  srrg_depth2laser_library
  srrg_depth2laser_viewers_library
  ${catkin_LIBRARIES}
)

