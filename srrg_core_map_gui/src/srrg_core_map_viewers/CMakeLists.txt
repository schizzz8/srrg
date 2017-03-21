add_library(srrg_core_map_viewers_library SHARED
  cloud_viewer.cpp cloud_viewer.h
  trajectory_viewer.cpp trajectory_viewer.h
)
target_link_libraries(srrg_core_map_viewers_library
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${catkin_LIBRARIES}
)
