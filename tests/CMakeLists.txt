set_source_files_properties(programall.cpp PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED ON)
add_executable(programall programall.cpp)
target_link_libraries(programall PRIVATE stkcomms)
add_test(NAME programall COMMAND programall)
