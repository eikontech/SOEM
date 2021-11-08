
set(SOURCES test_speed_torque.c)
add_executable(test_speed_torque ${SOURCES})
target_link_libraries(test_speed_torque soem m)
install(TARGETS test_speed_torque DESTINATION bin)
