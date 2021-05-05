set(BUILD_FOR_READOUT_CARD_VALUES "AUTODETECT;CRU;TRORC")
set(BUILD_FOR_READOUT_CARD "AUTODETECT" CACHE STRING
    "Readout card to build for. Values: ${BUILD_FOR_READOUT_CARD_VALUES}")
# Although it should support it, cmake 2.8 does not show options for readout card in GUI
#   However, this works with cmake3, i.e. ccmake3
set_property(CACHE BUILD_FOR_READOUT_CARD PROPERTY STRINGS ${BUILD_FOR_READOUT_CARD_VALUES})

#-------------------------------------------------------------------------------
string(TOUPPER ${BUILD_FOR_READOUT_CARD} BUILD_FOR_READOUT_CARD_UPPER)

if (BUILD_FOR_READOUT_CARD_UPPER STREQUAL "AUTODETECT")
  # Autodetect
  execute_process(
      COMMAND ${CMAKE_SOURCE_DIR}/contrib/findInstalledReadoutCards.sh
      RESULT_VARIABLE CMAKE_INSTALLED_READOUT_CARDS_RC
      OUTPUT_VARIABLE CMAKE_INSTALLED_READOUT_CARDS
    )

  set(CMAKE_INSTALLED_READOUT_CARDS ${CMAKE_INSTALLED_READOUT_CARDS} CACHE STRING
      "Readout cards detected in system" FORCE)
  set(CMAKE_INSTALLED_READOUT_CARDS_RC ${CMAKE_INSTALLED_READOUT_CARDS_RC} CACHE STRING
      "Readout cards detected in system, return code" FORCE)

  set(TMP_CARDS ${CMAKE_INSTALLED_READOUT_CARDS})
else()
  # Manual entry
  set(TMP_CARDS ${BUILD_FOR_READOUT_CARD_UPPER})
endif()


# Is there more elegant way to do this?
set(TMP_CRU_DETECTED OFF)
set(TMP_TRORC_DETECTED OFF)

if (TMP_CARDS MATCHES "CRU")
  set(TMP_CRU_DETECTED ON)
  message(STATUS "CRU detected in system")
elseif (TMP_CARDS MATCHES "TRORC")
  set(TMP_TRORC_DETECTED ON)
  message(STATUS "T-RORC detected in system")
endif()

# Set/update cmake cache variables
set(BUILD_FOR_CRU ${TMP_CRU_DETECTED} CACHE BOOL
    "Build CRU related tools and libraries" FORCE)
set(BUILD_FOR_TRORC ${TMP_TRORC_DETECTED} CACHE BOOL
    "Build T-RORC related tools and libraries" FORCE)
