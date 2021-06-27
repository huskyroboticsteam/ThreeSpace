cmake_minimum_required(VERSION 3.6)
include(GNUInstallDirs)

project(ThreeSpace
  LANGUAGES CXX C
  VERSION 3.0.6)

set(CXX_FILES
  # src/ directory
  src/threespace_api.cpp
  src/threespace_api_export.cpp
  src/threespace_device.cpp
  src/threespace_dongle.cpp
  src/threespace_sensor.cpp
  # src/yost directory
  src/yost/yost_core_api.cpp
  src/yost/yost_fair_mutex.cpp
  src/yost/yost_math.cpp
  # src/serial directory
  src/serial/serial_enumerator.cpp
  src/serial/serial.cc)

if(WIN32)
  list(APPEND CXX_FILES
	src/serial/serial_enumerator_win.cpp
	src/serial/impl/win.cc)
elseif(APPLE)
  list(APPEND CXX_FILES
	src/serial/serial_enumerator_osx.cpp
	src/serial/impl/unix.cc)
elseif(UNIX)
  list(APPEND CXX_FILES
	src/serial/serial_enumerator_linux.cpp
	src/serial/impl/unix.cc)
endif()

add_library(threespace SHARED ${CXX_FILES})
target_include_directories(threespace
  PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(threespace PRIVATE cxx_std_14)
#target_compile_options(threespace PRIVATE -Werror)

install(TARGETS threespace
  EXPORT threespace-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(EXPORT threespace-targets
  FILE ThreeSpaceTargets.cmake
  NAMESPACE ThreeSpace::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ThreeSpace)
