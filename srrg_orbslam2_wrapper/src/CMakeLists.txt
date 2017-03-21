
add_executable(srrg_file_converter srrg_file_converter.cpp)
target_link_libraries(srrg_file_converter
  srrg_system_utils_library
  srrg_txt_io_library
)

