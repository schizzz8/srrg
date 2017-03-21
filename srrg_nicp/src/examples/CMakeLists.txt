add_executable(srrg_nicp_projective_aligner_example srrg_nicp_projective_aligner_example.cpp)
target_link_libraries(srrg_nicp_projective_aligner_example 
  srrg_nicp_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(srrg_nicp_spherical_aligner_example srrg_nicp_spherical_aligner_example.cpp)
target_link_libraries(srrg_nicp_spherical_aligner_example 
  srrg_nicp_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(srrg_nicp_3dscan2cloud_example srrg_nicp_3dscan2cloud_example.cpp)
target_link_libraries(srrg_nicp_3dscan2cloud_example 
  srrg_nicp_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(srrg_nicp_nn_aligner_example srrg_nicp_nn_aligner_example.cpp)
target_link_libraries(srrg_nicp_nn_aligner_example 
  srrg_nicp_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
