add_executable(srrg_nicp_tracker_gui_app srrg_nicp_tracker_gui_app.cpp)
target_link_libraries(srrg_nicp_tracker_gui_app 
  srrg_nicp_tracker_viewers_library
  ${catkin_LIBRARIES}
)
