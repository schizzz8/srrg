ADD_LIBRARY(srrg_octree_library SHARED
  octree.cpp octree.h
)
TARGET_LINK_LIBRARIES(srrg_octree_library
  ${catkin_LIBRARIES}
)
