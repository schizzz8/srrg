add_library(srrg_nicp_tracker_viewers_library SHARED
  tracker_viewer.cpp tracker_viewer.h
  localizer3d_viewer.cpp localizer3d_viewer.h
)
target_link_libraries(srrg_nicp_tracker_viewers_library
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${catkin_LIBRARIES}
)


