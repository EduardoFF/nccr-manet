
set(XBeeAppData_HEADERS
  xbee_app_data.h
)

set(XBeeAppData_IMPLEMENTATIONS
  xbee_app_data.cc
)


add_library(xbee_app_data ${XBeeAppData_IMPLEMENTATIONS})

target_link_libraries(xbee_app_data
  xbeep.a
  xbee.a
  )


