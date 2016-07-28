add_executable(intelhex intelhex.cpp)
set_target_properties(intelhex PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
target_link_libraries(intelhex PRIVATE stkcomms)
add_test(NAME intelhex COMMAND intelhex)
