add_library(srrg_nicp_tracker_library SHARED
  base_triggers.cpp base_triggers.h
  call_tracker_trigger.cpp call_tracker_trigger.h
  multi_tracker.cpp multi_tracker.h
  tracker.cpp tracker.h  
)
target_link_libraries(srrg_nicp_tracker_library 
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
