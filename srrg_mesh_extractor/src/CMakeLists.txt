aux_source_directory(. SRC_LIST)

add_executable(${PROJECT_NAME} 
	${SRC_LIST}
)

target_link_libraries(${PROJECT_NAME} 
	${catkin_LIBRARIES} 
	${PCL_LIBRARIES} 
	${VTK_LIBRARIES} 
	${OpenCV_LIBS}
        srrg_system_utils_library
        srrg_txt_io_library
)

