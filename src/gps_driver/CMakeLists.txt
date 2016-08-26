
include_directories(${PROJECT_SOURCE_DIR}/gps_client)
set(gpsdriver_HEADERS
  gps_driver.h
)

set(gpsdriver_IMPLEMENTATIONS
  gps_driver.cc
)

add_library(gps_driver ${gpsdriver_IMPLEMENTATIONS})

target_link_libraries(gps_driver)

