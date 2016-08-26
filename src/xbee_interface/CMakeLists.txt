
set(XBeeInterface_HEADERS
  xbee_interface.h
)

set(XBeeInterface_IMPLEMENTATIONS
  xbee_interface.cc
)


add_library(xbee_interface ${XBeeInterface_IMPLEMENTATIONS})

target_link_libraries(xbee_interface
  xbeep.a
  xbee.a
  )


