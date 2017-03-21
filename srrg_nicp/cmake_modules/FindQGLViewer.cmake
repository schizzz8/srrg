# Need to find both Qt4 and QGLViewer if the QQL support is to be built
find_package(Qt4 COMPONENTS QtXml QtOpenGL QtGui)

find_path(QGLVIEWER_INCLUDE_DIR qglviewer.h
  /usr/include/QGLViewer
  /opt/local/include/QGLViewer
  /usr/local/include/QGLViewer
  /sw/include/QGLViewer
)

find_library(QGLVIEWER_LIBRARY NAMES  qglviewer-qt4 QGLViewer
  PATHS
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  /sw/lib
)

if(QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARY)
  set(QGLVIEWER_FOUND TRUE)
else(QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARY)
  set(QGLVIEWER_FOUND FALSE)
endif(QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARY)
