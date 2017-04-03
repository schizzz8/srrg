add_library(srrg_core_viewers_library SHARED
  simple_viewer.cpp simple_viewer.h
)

target_link_libraries(srrg_core_viewers_library
  ${QGLVIEWER_LIBRARY} 
  ${SRRG_QT_LIBRARIES}
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
)
