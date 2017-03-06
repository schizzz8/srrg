add_library(srrg_bresenham_library SHARED
  bresenham.cpp bresenham.h
)

target_link_libraries(srrg_bresenham_library
  ${OpenCV_LIBS}
)
