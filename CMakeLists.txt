# STK Communications library
# Barobo, Inc.

cmake_minimum_required(VERSION 3.2.1)
project(stkcomms VERSION 0.1.0)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.54.0 REQUIRED COMPONENTS system filesystem log log_setup thread date_time regex)

find_package(cxx-util)
find_package(Threads)

# Targets and properties

add_library(stkcomms STATIC
    src/system_error.cpp
    src/blob.cpp)

set_target_properties(stkcomms PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    MACOSX_RPATH ON
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

target_include_directories(stkcomms PRIVATE include PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(stkcomms
    PUBLIC barobo::cxx-util
           ${Boost_LIBRARIES}
           ${CMAKE_THREAD_LIBS_INIT})

option(LIBSTKCOMMS_BUILD_TESTS "Build libstkcomms tests" OFF)
if(LIBSTKCOMMS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

install(TARGETS stkcomms EXPORT barobo
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

# Boilerplate follows

export(EXPORT barobo
    FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake"
    NAMESPACE barobo::
)

install(EXPORT barobo
    FILE ${PROJECT_NAME}-targets.cmake
    NAMESPACE barobo::
    DESTINATION cmake
)

file(WRITE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    "include(\"\${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}-targets.cmake\")"
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-configVersion.cmake"
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-configVersion.cmake"
    DESTINATION cmake
)
