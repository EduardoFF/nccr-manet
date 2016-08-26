
set(gpsclient_HEADERS
  gps_client.h
)

set(gpsclient_IMPLEMENTATIONS
  gps_client.cc
)

#include_directories(${CMAKE_CURRENT_SRC

add_library(gps_client ${gpsclient_IMPLEMENTATIONS})
add_executable(gps_client_test gps_client_test.cc)

#target_link_libraries(gps_client)
target_link_libraries(gps_client_test gps_client gps)

