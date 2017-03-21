add_library(srrg_localizer2d_library
  localization_filter.cpp localization_filter.h
)

target_link_libraries(srrg_localizer2d_library
  srrg_distance_map_library
  ${OpenCV_LIBS}
)
