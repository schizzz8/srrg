add_executable(srrg_cloud_viewer_gui_app srrg_cloud_viewer_gui_app.cpp)
target_link_libraries(srrg_cloud_viewer_gui_app
  srrg_core_map_viewers_library
  ${catkin_LIBRARIES}
)

add_executable(srrg_trajectory_loader_gui_app srrg_trajectory_loader_gui_app.cpp)
target_link_libraries(srrg_trajectory_loader_gui_app 
  srrg_core_map_viewers_library
  ${catkin_LIBRARIES}
)

add_executable(srrg_camera_trajectory_loader_gui_app srrg_camera_trajectory_loader_gui_app.cpp)
target_link_libraries(srrg_camera_trajectory_loader_gui_app 
  srrg_core_map_viewers_library
  ${catkin_LIBRARIES}
)
