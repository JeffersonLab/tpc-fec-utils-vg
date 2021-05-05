#-------------------------------------------------------------------------------
set (LIB_FEC_SOURCES
  utils
  simple_log
  hdlc_core
  gbt_sca_i2c
  fec_revision
  sampa
  fec
)

# List utils related to FEC here, build & link in subdirectory
set (UTIL_FEC_LIST
  tdbg
  tfec
  tsca
  tsampa
)

add_library(fec STATIC ${LIB_FEC_SOURCES})

set(EXTRA_LIBS boost_program_options)
set(LOCAL_LIBS fec)

include_directories(${Boost_INCLUDE_DIR})

#-------------------------------------------------------------------------------
# Setup for card-specific build
#
# T-RORC specific
if(BUILD_FOR_TRORC)
  include_directories(${LIBRORC_INCLUDE_DIR})
  set(READOUTCARD_LIBS ${LIBRORC_LIBRARY})
  include_directories(trorc)
  add_subdirectory(trorc)

# CRU specific
elseif(BUILD_FOR_CRU)
  include_directories(${LIBREADOUTCARD_INCLUDE_DIRS} ${LIBINFOLOGGER_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS}) # Make sure O2-provided boost is found first
  set(READOUTCARD_LIBS ${LIBREADOUTCARD_LIBRARIES} ${LIBINFOLOGGER_LIBRARIES})
  include_directories(cru)
  add_subdirectory(cru)
endif()

#-------------------------------------------------------------------------------
add_executable(sampa_raw_data_decoder sampa_raw_data_decoder.cpp)
target_link_libraries(sampa_raw_data_decoder boost_program_options)
install(TARGETS sampa_raw_data_decoder RUNTIME DESTINATION bin)
