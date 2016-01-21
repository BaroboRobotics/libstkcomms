# STK Communications library
# Barobo, Inc.

cmake_minimum_required(VERSION 3.2.1)
project(stkcomms VERSION 0.1.0)

if(TARGET stkcomms)
    # FIXME make sure that the current target's version or configuration matches?
    # this feels rather dangerous. find_package would solve this conundrum :/
    message(FATAL_ERROR "stkcomms already configured")
endif()

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.59.0 REQUIRED COMPONENTS system filesystem log log_setup thread date_time regex)

# Targets and properties

add_library(stkcomms STATIC
    src/libstkcomms.cpp
    src/libstkcomms++.cpp
    src/system_error.cpp)

set_target_properties(stkcomms PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    MACOSX_RPATH ON
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
)
target_compile_definitions(stkcomms
    PRIVATE "-DBUILD_CSTKCOMMS"
)

target_include_directories(stkcomms PUBLIC include ${Boost_INCLUDE_DIRS})
target_link_libraries(stkcomms PUBLIC baromesh-common cxx-util ${Boost_LIBRARIES})

if(WIN32)
    target_compile_definitions(stkcomms
      PUBLIC _WIN32_WINNT=0x0501 WIN32_LEAN_AND_MEAN
    )
    target_compile_options(stkcomms PRIVATE /wd4100 # these two warnings are only because of VERBOSE macro
                                             /wd4189
                                             )
    target_link_libraries(stkcomms PUBLIC ws2_32)
else()
    target_link_libraries(stkcomms PUBLIC pthread)
endif()

# Staging

if(WIN32)
    set(dest .)
elseif(APPLE)
    set(dest ../MacOS)
else()
    set(dest lib)
endif()

#install(TARGETS stkcomms DESTINATION ${dest})

if(LIBSTKCOMMS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
