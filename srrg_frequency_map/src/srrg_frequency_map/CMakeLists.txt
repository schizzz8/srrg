add_library(srrg_frequency_map_library SHARED
  frequency_map.cpp frequency_map.h
  grid_line_traversal.h grid_line_traversal.cpp
)
target_link_libraries(srrg_frequency_map_library 
  srrg_txt_io_library
  ${catkin_LIBRARIES}
)
