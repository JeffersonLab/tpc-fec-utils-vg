configure_file(tpc_initialize.sh.in ${CMAKE_BINARY_DIR}/tpc_initialize.sh @ONLY)
install(PROGRAMS ${CMAKE_BINARY_DIR}/tpc_initialize.sh DESTINATION bin)

configure_file(tpc_print_status.sh.in ${CMAKE_BINARY_DIR}/tpc_print_status.sh @ONLY)
install(PROGRAMS ${CMAKE_BINARY_DIR}/tpc_print_status.sh DESTINATION bin)
