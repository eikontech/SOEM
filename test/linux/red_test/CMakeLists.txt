set(SOURCES_CONTROL realtime_control.c)
add_executable(realtime_control ${SOURCES_CONTROL})
target_link_libraries(realtime_control soem)
install(TARGETS realtime_control DESTINATION bin)

set(SOURCES_SPEED red_test_speed.c)
add_executable(red_test_speed ${SOURCES_SPEED})
target_link_libraries(red_test_speed soem)
install(TARGETS red_test_speed DESTINATION bin)

set(SOURCES_TORQUE red_test_torque.c)
add_executable(red_test_torque ${SOURCES_TORQUE})
target_link_libraries(red_test_torque soem)
install(TARGETS red_test_torque DESTINATION bin)