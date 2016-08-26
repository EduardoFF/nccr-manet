
set(debugmanager_HEADERS
  debug_manager.h
)

set(debugmanager_IMPLEMENTATIONS
  debug_manager.cc
)

#include_directories(${CMAKE_CURRENT_SRC

add_library(debug_manager ${debugmanager_IMPLEMENTATIONS})
#add_executable(debug_manager_test debug_manager_test.cc)

#target_link_libraries(debug_manager)
#target_link_libraries(debug_manager_test debug_manager)

install(TARGETS debug_manager
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )

install(FILES debug_manager.h DESTINATION include)

