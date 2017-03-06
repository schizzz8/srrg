add_library(srrg_local_maps_condenser_library SHARED
	local_maps_clusterizer.h local_maps_clusterizer.cpp
	#connectivity_refiner.h connectivity_refiner.cpp
)

target_link_libraries(srrg_local_maps_condenser_library
	${catkin_LIBRARIES}
)
