#****************** INCLUDES **************************

include_directories(${PROJECT_SOURCE_DIR}/gps_driver)
include_directories(${PROJECT_SOURCE_DIR}/xbee_interface)
include_directories(${PROJECT_SOURCE_DIR}/xbee_app_data)
include_directories(${PROJECT_SOURCE_DIR}/routing_driver)
include_directories(${PROJECT_SOURCE_DIR}/timer)
include_directories(${PROJECT_SOURCE_DIR}/gps_client)

#*******************************************************

set(EndNode_HEADERS
  end_node.h
)

set(EndNode_IMPLEMENTATIONS
  end_node.cc
)

add_executable(end_node ${EndNode_IMPLEMENTATIONS})

target_link_libraries(end_node
  gps_driver
  routing_driver
  planning_driver
  flow_notifier
  xbee_interface
  xbee_app_data
  timer
  pthread
  lcm
  gps_client
  gps
  ${Glog_LIBRARIES}
  )

install(TARGETS end_node
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )

