
set(SOURCES test_position.c)
add_executable(test_position ${SOURCES})
target_link_libraries(test_position soem m)
install(TARGETS test_position DESTINATION bin)
