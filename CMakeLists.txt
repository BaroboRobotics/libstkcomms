# STK Communications library
# Barobo, Inc.

cmake_minimum_required(VERSION 2.8.9)

set(target stkcomms)

project(${target})

if(TARGET ${target})
  # FIXME make sure that the current target's version or configuration matches?
  # this feels rather dangerous. find_package would solve this conundrum :/
  message(FATAL_ERROR "stkcomms already configured")
endif()

set(VERSION_MAJOR 0)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

# Targets and properties

add_library(${target}
  src/libstkcomms.cpp src/libstkcomms++.cpp)

set_target_properties(${target} PROPERTIES
  VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}
  SOVERSION ${VERSION_MAJOR}
  MACOSX_RPATH ON
)
target_compile_definitions(${target}
  PRIVATE "-DBUILD_CSTKCOMMS"
)

target_include_directories(${target} PUBLIC include)

if(WIN32)
  target_compile_definitions(${target}
    PRIVATE "-D_WIN32_WINNT=0x0502 -DWIN32_LEAN_AND_MEAN"
  )
  target_link_libraries(${target} ws2_32)
else()
  target_link_libraries(${target} pthread)
  if(NOT APPLE)
    # The BlueZ header requires GNU extensions.
    target_compile_options(${target} INTERFACE "-std=gnu++11")
  endif()
endif()

# Staging

if(WIN32)
  set(dest .)
elseif(APPLE)
  set(dest ../MacOS)
else()
  set(dest lib)
endif()

install(TARGETS ${target} DESTINATION ${dest})
