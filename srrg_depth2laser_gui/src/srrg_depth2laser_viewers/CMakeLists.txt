add_library(srrg_depth2laser_viewers_library SHARED
  depth2laser_viewer.h depth2laser_viewer.cpp
)
target_link_libraries(srrg_depth2laser_viewers_library
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${catkin_LIBRARIES}
)


