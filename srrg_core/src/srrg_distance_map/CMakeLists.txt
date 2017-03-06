add_library(srrg_distance_map_library SHARED
  distance_map.cpp distance_map.h
)

target_link_libraries(srrg_distance_map_library
  ${OpenCV_LIBS}
)
