add_executable(srrg_trajectory_loader_example srrg_trajectory_loader_example.cpp)
target_link_libraries(srrg_trajectory_loader_example 
  srrg_core_map_library
  ${catkin_LIBRARIES}
)
