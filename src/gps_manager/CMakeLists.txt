
set(gpsmanager_HEADERS
  gps_manager.h
)

set(gpsmanager_IMPLEMENTATIONS
  gps_manager.cc
)

#include_directories(${CMAKE_CURRENT_SRC

add_library(gps_manager ${gpsmanager_IMPLEMENTATIONS})
add_executable(gps_manager_test gps_manager_test.cc)

#target_link_libraries(gps_manager)
target_link_libraries(gps_manager_test gps_manager gps)

install(TARGETS gps_manager
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )

install(FILES gps_manager.h DESTINATION include)

