add_library(srrg_nicp_library SHARED 
  base_aligner.cpp base_aligner.h
  base_correspondence_finder.cpp base_correspondence_finder.h
  base_projector.cpp base_projector.h
  camera_info_manager.cpp camera_info_manager.h
  depth_utils.cpp depth_utils.h
  multi_projector.cpp multi_projector.h
  nn_aligner.cpp nn_aligner.h
  nn_correspondence_finder.cpp nn_correspondence_finder.h
  pinhole_projector.cpp pinhole_projector.h
  projective_aligner.cpp projective_aligner.h
  projective_correspondence_finder.cpp projective_correspondence_finder.h
  solver.cpp solver.h
  spherical_projector.cpp spherical_projector.h
)
target_link_libraries(srrg_nicp_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
