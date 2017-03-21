add_library(srrg_scan_extractor_library SHARED
  scan_extractor.h scan_extractor.cpp
  )

target_link_libraries(srrg_scan_extractor_library
  srrg_txt_io_library
  ${catkin_LIBRARIES}
  )
