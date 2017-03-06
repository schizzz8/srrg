add_library(srrg_core_map_library SHARED
  base_camera_info.cpp base_camera_info.h
  binary_node_relation.cpp binary_node_relation.h
  cloud.cpp cloud.h
  image_map_node.cpp image_map_node.h
  local_map.h local_map.cpp
  map_node.cpp map_node.h
  map_node_list.cpp map_node_list.h
  multi_camera_info.cpp multi_camera_info.h
  multi_image_map_node.cpp multi_image_map_node.h  
  pinhole_camera_info.cpp pinhole_camera_info.h
  spherical_camera_info.cpp spherical_camera_info.h
  traversability_map.cpp traversability_map.h
)
target_link_libraries(srrg_core_map_library
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${catkin_LIBRARIES}    
)
