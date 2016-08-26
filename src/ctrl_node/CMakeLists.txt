
include_directories(${PROJECT_SOURCE_DIR}/routing_driver)
include_directories(${PROJECT_SOURCE_DIR}/gps_driver)
include_directories(${PROJECT_SOURCE_DIR}/xbee_interface)
include_directories(${PROJECT_SOURCE_DIR}/xbee_app_data)
include_directories(${PROJECT_SOURCE_DIR}/timer)
set(CtrlNode_HEADERS
  ctrl_node.h
)

set(CtrlNode_IMPLEMENTATIONS
  ctrl_node.cc
)


add_executable(ctrl_node ${CtrlNode_IMPLEMENTATIONS})

target_link_libraries(ctrl_node
  xbee_interface
  xbee_app_data
  timer
  pthread
  lcm
  routing_driver
  planning_driver
  gps_driver
  gps_client
  flow_notifier
  debug_manager
  gps
  ${Glog_LIBRARIES}
#  xbeep.a
#  xbee.a
  )

install(TARGETS ctrl_node
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )

#install(FILES 
#  ctrl_node.h DESTINATION include)

