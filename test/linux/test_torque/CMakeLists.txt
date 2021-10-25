
set(SOURCES test_torque.c)
add_executable(test_torque ${SOURCES})
target_link_libraries(test_torque soem m)
install(TARGETS test_torque DESTINATION bin)
