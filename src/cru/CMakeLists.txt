set (LIB_CRU_SOURCES
  hdlc_cern_me
)

add_library(cru STATIC ${LIB_CRU_SOURCES})

list(INSERT LOCAL_LIBS 0 cru)

# Build utils related to FEC
foreach(UTIL ${UTIL_FEC_LIST})
  add_executable(${UTIL} ../${UTIL}.cpp)
endforeach(UTIL)

# Link / install
foreach(UTIL IN LISTS UTIL_LIST UTIL_FEC_LIST)
  target_link_libraries(${UTIL} ${READOUTCARD_LIBS} ${EXTRA_LIBS} ${LOCAL_LIBS})
  install(TARGETS ${UTIL} RUNTIME DESTINATION bin)
endforeach(UTIL)
