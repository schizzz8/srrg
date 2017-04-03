add_library(srrg_core_viewers_library SHARED
  simple_viewer.cpp simple_viewer.h
)

target_link_libraries(srrg_core_viewers_library
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${QGLVIEWER_LIBRARY} 
)

#ds package specific linkage: qt
if(${SRRG_CORE_VIEWERS_USE_QT_5})
  target_link_libraries(srrg_core_viewers_library
    Qt5::Widgets
  )
else()
  target_link_libraries(srrg_core_viewers_library
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${QT_QTXML_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}
  )
endif()
