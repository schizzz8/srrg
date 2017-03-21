add_library(srrg_gl_helpers_library SHARED
  opengl_primitives.cpp opengl_primitives.h
)
target_link_libraries(srrg_gl_helpers_library
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
)
