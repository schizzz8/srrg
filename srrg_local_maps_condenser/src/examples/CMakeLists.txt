add_executable(merging_test merging_test.cpp)
target_link_libraries(merging_test 
  srrg_local_maps_condenser_library
  ${catkin_LIBRARIES}
)
