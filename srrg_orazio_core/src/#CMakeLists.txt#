add_library(orazio_core_library
  orazio_robot_connection.cpp orazio_robot_connection.h
  joint_controller.cpp joint_controller.h
  serial.c serial.h
  packet_encoder.cpp packet_encoder.h
  packet_decoder.cpp packet_decoder.h
  differential_drive_kinematics.cpp differential_drive_kinematics.h
)

target_link_libraries(orazio_core_library
  ${CMAKE_THREAD_LIBS_INIT}
)

FIND_PACKAGE(Curses REQUIRED)
include_directories(CURSES_INCLUDE_DIR)
add_executable(
  orazio_robot_monitor
  orazio_robot_monitor.cpp
)

target_link_libraries(
  orazio_robot_monitor
  orazio_core_library
  ${CURSES_CURSES_LIBRARY}
  ${CMAKE_THREAD_LIBS_INIT}
)


add_executable(
  orazio_robot_shell
  command_parser.cpp
  orazio_robot_shell.cpp
)

target_link_libraries(
  orazio_robot_shell
  orazio_core_library
  ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(
  orazio_robot_websocket_server
  command_parser.cpp
  orazio_robot_websocket_server.cpp
)

target_link_libraries(
  orazio_robot_websocket_server
  orazio_core_library
  ${CMAKE_THREAD_LIBS_INIT}
  ${LIBWEBSOCKETS_LIBRARIES}
)

