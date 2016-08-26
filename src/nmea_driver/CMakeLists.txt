
set(NmeaDriver_HEADERS
  nmea_driver.h
)

set(NmeaDriver_IMPLEMENTATIONS
  nmea_driver.cc
)


add_executable(nmea_driver ${NmeaDriver_IMPLEMENTATIONS})

target_link_libraries(nmea_driver

  )

install(TARGETS nmea_driver
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )

install(FILES 
  nmea_driver.h DESTINATION include)

