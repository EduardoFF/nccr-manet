# some useful stand-alone libraries
# to install for the footbots do:
# 	cmake \
#       -DCMAKE_TOOLCHAIN_FILE=/home/eduardo/Dropbox/cmake_modules/moboard.cmake
# 	 -DCMAKE_INSTALL_PREFIX=/usr/local/angstrom/arm
#   and remember to set build type release to avoid overhead of debugging msgs

cmake_minimum_required (VERSION 2.6)
project (rnp_xbee_bridge)

#-------------To add a 3rd party library-----------
find_package(PkgConfig)
pkg_check_modules(Glog libglog)

link_directories(${Glog_LIBRARY_DIRS})
include_directories(${Glog_INCLUDE_DIRS})
#--------------------------------------------------

set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})

SET (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin CACHE PATH

  "Single Directory for all executables"
  )

SET (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/libs CACHE PATH

  "Single Directory for all dynamic Libraries"
  )

MESSAGE( STATUS "CMAKE_C_FLAGS: " ${CMAKE_C_FLAGS} )
MESSAGE( STATUS "CMAKE_C_FLAGS_DEBUG: " ${CMAKE_C_FLAGS_DEBUG} )
MESSAGE( STATUS "CMAKE_C_FLAGS_RELEASE: " ${CMAKE_C_FLAGS_RELEASE} )

MESSAGE( STATUS "CMAKE_CXX_FLAGS: " ${CMAKE_C_FLAGS} )
MESSAGE( STATUS "CMAKE_CXX_FLAGS_DEBUG: " ${CMAKE_C_FLAGS_DEBUG} )
MESSAGE( STATUS "CMAKE_CXX_FLAGS_RELEASE: " ${CMAKE_C_FLAGS_RELEASE} )


include_directories( ${PROJECT_SOURCE_DIR} )

set(bitness 32)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(bitness 64)  
endif()

if( bitness EQUAL 32 )
    MESSAGE("DETECTED 32 bit ")
    set(COMPILE_32 TRUE)
    set(COMPILE_64 FALSE)
else()
    MESSAGE("DETECTED 64 bit ")
    set(COMPILE_32 FALSE)
    set(COMPILE_64 TRUE)
endif()

if( NO_XBEE_TEST )
  add_definitions("-DNO_XBEE_TEST=1")
endif( NO_XBEE_TEST )
add_subdirectory(debug_manager)
add_subdirectory(routing_driver)
add_subdirectory(planning_driver)
add_subdirectory(flow_notifier)
add_subdirectory(gps_driver)
add_subdirectory(gps_client)
add_subdirectory(xbee_interface)
add_subdirectory(xbee_app_data)
add_subdirectory(timer)
add_subdirectory(end_node)
add_subdirectory(ctrl_node)
#add_subdirectory(scripts)




