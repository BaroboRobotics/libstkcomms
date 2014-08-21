# STK Communications library
# Barobo, Inc.

cmake_minimum_required(VERSION 2.8.9)
project(LIBSTKCOMMS C CXX)

set(target stkcomms)
if(TARGET ${target})
  # FIXME make sure that the current target's version or configuration matches?
  # this feels rather dangerous. find_package would solve this conundrum :/
  message(STATUS "libstkcomms already configured")
  return()
endif()

set(VERSION_MAJOR 0)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

# Targets

set(SOURCES src/libstkcomms.cpp src/libstkcomms++.cpp)

add_library(${target} ${SOURCES})

set_target_properties(${target} PROPERTIES
  VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}
  SOVERSION ${VERSION_MAJOR}
)
target_compile_definitions(${target} PRIVATE BUILD_CSTKCOMMS)

# Platform Specific Setup

if(WIN32)
  # FIXME find a place for findwindowssdk.cmake to live
  include("../libbarobo/findwindowssdk.cmake")
  if(NOT WINSDK_VERSION)
    message(FATAL_ERROR "libstkcomms requires the Windows SDK")
  endif()

  # Prior to Windows SDK 8.0, there was only a single flat Include directory.
  # Thereafter, the headers were broken out into three separate directories:
  # winrt (Windows Store API), shared, and um (user mode). User mode is the
  # one which contains the classic headers we care about, like windows.h and
  # ws2bth.h. This command will figure out which one we're dealing with.
  find_path(WINSDK_UM_INCLUDE_DIR windows.h
    PATHS "${WINSDK_INSTALL_DIR}/Include"
          "${WINSDK_INSTALL_DIR}/Include/um"
    NO_DEFAULT_PATH)

  if(NOT WINSDK_UM_INCLUDE_DIR)
    message(FATAL_ERROR "Unable to find Windows SDK user mode headers under ${WINSDK_INSTALL_DIR}")
  endif()


  set_target_properties(${target} PROPERTIES
    COMPILE_FLAGS "-idirafter \"${WINSDK_UM_INCLUDE_DIR}\" -D_WIN32_WINNT=0x0502 -DWIN32_LEAN_AND_MEAN"
  )


  target_link_libraries(${target} ws2_32)
  #set_target_properties(${target} PROPERTIES COMPILE_FLAGS "/GS-")
  # FIXME: There's no install() call. Are we relying on fixup_bundle? Is
  # that bad?
elseif(APPLE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ")
  target_link_libraries(${target} pthread)
  set_target_properties(${target} PROPERTIES
    INSTALL_NAME_DIR "@executable_path")
endif()

# Staging
if(WIN32)
  set(INSTALL_LIB_DIR .)
  set(INSTALL_BIN_DIR .)
  set(INSTALL_INCLUDE_DIR .)
elseif(APPLE)
  set(INSTALL_LIB_DIR Contents/MacOS)
  set(INSTALL_BIN_DIR Contents/MacOS)
  set(INSTALL_INCLUDE_DIR Contents/Headers)
else()
  set(INSTALL_LIB_DIR lib)
  set(INSTALL_BIN_DIR bin)
  set(INSTALL_INCLUDE_DIR include)
endif()

install(TARGETS ${target}
  ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
  LIBRARY DESTINATION ${INSTALL_LIB_DIR}
  RUNTIME DESTINATION ${INSTALL_BIN_DIR})
install(FILES include/libstkcomms.h include/libstkcomms.hpp DESTINATION ${INSTALL_INCLUDE_DIR})

# Configuration for dependent projects

set(PUBLIC_INCLUDE_DIR include)
include_directories(${PUBLIC_INCLUDE_DIR})

get_filename_component(${PROJECT_NAME}_INCLUDE_DIRS ${PUBLIC_INCLUDE_DIR} ABSOLUTE CACHE)
set(${PROJECT_NAME}_LIBRARIES ${target} CACHE INTERNAL "")
set(${PROJECT_NAME}_VERSION ${VERSION} CACHE INTERNAL "")
