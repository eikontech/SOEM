
set(SOURCES1 test_position_old.c)
add_executable(test_position_old ${SOURCES1})
target_link_libraries(test_position_old soem m)
install(TARGETS test_position_old DESTINATION bin)

set(SOURCES2 test_position.c)
add_executable(test_position ${SOURCES2})
target_link_libraries(test_position soem m)
install(TARGETS test_position DESTINATION bin)

