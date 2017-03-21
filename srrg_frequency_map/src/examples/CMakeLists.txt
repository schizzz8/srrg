add_executable(srrg_frequency_map_example srrg_frequency_map_example.cpp)

target_link_libraries(srrg_frequency_map_example
  srrg_system_utils_library
  srrg_txt_io_library
  srrg_frequency_map_library
  ${catkin_LIBRARIES}
)
