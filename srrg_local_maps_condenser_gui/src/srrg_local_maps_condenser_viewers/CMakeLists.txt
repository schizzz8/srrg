add_library(srrg_local_maps_condenser_viewers_library SHARED   
	condenser_viewer.cpp condenser_viewer.h  
) 

target_link_libraries(srrg_local_maps_condenser_viewers_library   
	${OPENGL_gl_LIBRARY}   
	${OPENGL_glu_LIBRARY}   
	${QGLVIEWER_LIBRARY}   
	${catkin_LIBRARIES} 
	${QT_QTXML_LIBRARY}     
	${QT_QTOPENGL_LIBRARY}     
	${QT_QTGUI_LIBRARY}     
	${QT_QTCORE_LIBRARY}   
) 
