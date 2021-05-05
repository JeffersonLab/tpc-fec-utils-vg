CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11 FATAL_ERROR)

SET(VERSION "0.0.1")
SET(CPACK_RPM_PACKAGE_RELEASE 1)
PROJECT(tpc-fec-utils)

set(PROJECT_LOGO_PATH ${CMAKE_SOURCE_DIR}/contrib/trorc_logo.png)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/contrib/")
MESSAGE(STATUS "Install directory   = ${CMAKE_INSTALL_PREFIX}")

#
#--- GCC ---------------------------------------------------------------------------
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.0)
    message(FATAL_ERROR "GCC version must be at least 5.0, found ${CMAKE_CXX_COMPILER_VERSION}!")
  endif()
else()
  message(WARNING "Unsupported compiler, your on your own...")
endif()

#
#--- FLAGS ---------------------------------------------------------------------------
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g -O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

#
#--- BUILD SELECTION (for CRU or T-RORC) ------------------------------------------
# Based on the current value of BUILD_FOR_READOUT_CARD
#   this will either autodetect cards in the system (AUTODETECT)
#   or set up the system to build for one of the readout cards
# Package will set either BUILD_FOR_TRORC or BUILD_FOR_CRU
find_package(UpdateBuildReadoutCardOptions)

#
#--- GIT ---------------------------------------------------------------------------
find_package(GitInfo)

#
#--- PACKAGES / LIBS --------------------------------------------------------------
find_package(Doxygen)
set(USE_DOXYGEN TRUE CACHE BOOL "Generate documentation using doxygen.")
if(USE_DOXYGEN AND NOT DOXYGEN_FOUND)
  message(STATUS "Binary not found: Doxygen. Not building documentation.")
endif()

find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

include_directories(submodules/gbt-hdlc/src)
include_directories(src)

# T-RORC specific
if(BUILD_FOR_TRORC)
  find_package(librorc 15.1.1 REQUIRED)
  add_compile_options(-std=c++1y)   # Check if c++14 works for T-RORC as well
# CRU specific
elseif(BUILD_FOR_CRU)
  add_compile_options(-std=c++14)
  find_package(InfoLogger REQUIRED)
  find_package(ReadoutCard REQUIRED)
else()
  add_compile_options(-std=c++14)
endif()


#
#--- OPTIONS / DEFINITIONS -----------------------------------------------------------
#
option(BUILD_SAMPA_TESTER "Build binaries (and libraries) specific to SAMPA test stand")
option(DFT_DEBUG "Enable verbose register access output to debug SAMPA test stand DFT test" OFF)
if (${DFT_DEBUG} AND ${BUILD_SAMPA_TESTER})
  add_definitions(-DDFT_DEBUG)
endif()

# Common
option(HDLC_DEBUG "Enable HDLC debug output" OFF)
if (${HDLC_DEBUG})
  add_definitions(-DHDLC_DEBUG)
endif()
option(HDLC_DEBUG1 "Enable detailed HDLC debug output" OFF)
if (${HDLC_DEBUG1})
  add_definitions(-DHDLC_DEBUG1)
endif()
option(SCA_DEBUG "Enable GBT SCA related debug output (pass -DSCA_DEBUG)" OFF)
if (${SCA_DEBUG})
  add_definitions(-DSCA_DEBUG)
endif()
option(SCA_I2C_DEBUG "Enable GBT SCA I2C debug output (pass -DSCA_I2C_DEBUG)" OFF)
if (${SCA_I2C_DEBUG})
  add_definitions(-DSCA_I2C_DEBUG)
endif()

# T-RORC specific
if(BUILD_FOR_TRORC)
  add_definitions(-DBUILD_FOR_TRORC)

  option(MODELSIM "Compile for MODELSIM (pass -DMODELSIM)" OFF)
  if(${MODELSIM})
      add_definitions(-DMODELSIM)
  endif(${MODELSIM})

  option(TRORC_DEBUG "Enable T-RORC debug output (pass -DTRORC_DEBUG)" OFF)
  if (${TRORC_DEBUG})
    add_definitions(-DTRORC_DEBUG)
  endif()

# CRU-specific
elseif(BUILD_FOR_CRU)
  add_definitions(-DBUILD_FOR_CRU)
endif()

#
#--- ADD DIRECTORIES -----------------------------------------------------------------
add_subdirectory(src)
add_subdirectory(script)
add_subdirectory(config)

if (USE_DOXYGEN AND DOXYGEN_FOUND)
  add_subdirectory(doc)
endif()

#
#--- CPACK ---------------------------------------------------------------------------
SET(CPACK_PACKAGE_VERSION ${VERSION})
SET(CPACK_GENERATOR "RPM")
SET(CPACK_PACKAGE_NAME "tpc-run3-readout-utils")
SET(CPACK_RPM_PACKAGE_LICENSE "BSD")
SET(CPACK_RPM_PACKAGE_GROUP "System Environment/Base")
SET(CPACK_PACKAGE_CONTACT "Heiko Engel <hengel@cern.ch>")
SET(CPACK_PACKAGE_VENDOR "CERN")
SET(CPACK_PACKAGING_INSTALL_PREFIX /usr)
SET(CPACK_PACKAGE_FILE_NAME
  "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_RPM_PACKAGE_RELEASE}.${CMAKE_SYSTEM_PROCESSOR}")
INCLUDE(CPack)
