
set(SOURCES test_speed.c)
add_executable(test_speed ${SOURCES})
target_link_libraries(test_speed soem m)
install(TARGETS test_speed DESTINATION bin)
