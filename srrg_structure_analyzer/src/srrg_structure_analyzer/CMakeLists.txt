ADD_LIBRARY(srrg_structure_analyzer_library SHARED
  structure_analyzer.cpp structure_analyzer.h
)
TARGET_LINK_LIBRARIES(srrg_structure_analyzer_library
  ${catkin_LIBRARIES}
)
