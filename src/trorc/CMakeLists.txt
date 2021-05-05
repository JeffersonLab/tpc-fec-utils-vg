set (LIB_TRORC_SOURCES
  gbt_link
  hdlc_sw_trorc
  trigger
  file_writer
  sampa_tester
)

set (UTIL_LIST
  trorc_init
  trorc_gbtctrl
  trorc_dma_in
  trorc_reset
  trorc_fec_readout
  trorc_fec_readout_control
  trorc_trigger_control
  trorc_status_dump
  trorc_dma_monitor
  trorc_rw
  treadout
  tcheck
)

add_library(trorc STATIC ${LIB_TRORC_SOURCES})

# SAMPA test stand specific -- only on T-RORC setups
if(BUILD_SAMPA_TESTER)
  list(APPEND UTIL_LIST ttester)
  list(APPEND UTIL_LIST tsampa-dft)
  list(APPEND UTIL_LIST tsampa-jtag)
endif()

# Build T-RORC specific utils
foreach(UTIL ${UTIL_LIST})
  add_executable(${UTIL} ${UTIL}.cpp)
endforeach(UTIL)

# Build utils related to FEC
foreach(UTIL ${UTIL_FEC_LIST})
  add_executable(${UTIL} ../${UTIL}.cpp)
endforeach(UTIL)

list(APPEND EXTRA_LIBS boost_thread boost_filesystem boost_system ${CMAKE_THREAD_LIBS_INIT})
list(INSERT LOCAL_LIBS 0 trorc)

# Link / install
foreach(UTIL IN LISTS UTIL_LIST UTIL_FEC_LIST)
  target_link_libraries(${UTIL} ${READOUTCARD_LIBS} ${EXTRA_LIBS} ${LOCAL_LIBS})
  install(TARGETS ${UTIL} RUNTIME DESTINATION bin)
endforeach(UTIL)
