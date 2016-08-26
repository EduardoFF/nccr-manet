
set(routingdriver_HEADERS
  routing_driver.h
)

set(routingdriver_IMPLEMENTATIONS
  routing_driver.cc
)


add_library(routing_driver ${routingdriver_IMPLEMENTATIONS})

target_link_libraries(routing_driver)


