#-------------------------------------------------------------------------------
# Try to find libReadoutCard
# once found this module will define:
#  LIBREADOUTCARD_LIBRARIES
#  LIBREADOUTCARD_INCLUDE_DIRS

find_path(LIBREADOUTCARD_INCLUDE_DIR NAMES "ReadoutCard/ReadoutCard.h"
          PATHS ENV READOUTCARD_ROOT PATH_SUFFIXES "include")
find_library(LIBREADOUTCARD_LIBRARY NAMES "ReadoutCard"
             PATHS ENV READOUTCARD_ROOT PATH_SUFFIXES "lib")

# message(STATUS "$ENV{READOUTCARD_ROOT} -> ${LIBREADOUTCARD_INCLUDE_DIR}")
# message(STATUS "$ENV{READOUTCARD_ROOT} -> ${LIBREADOUTCARD_LIBRARY}")

set(LIBREADOUTCARD_LIBRARIES ${LIBREADOUTCARD_LIBRARY}
    CACHE STRING "Location of DAQ-provided readout card library" FORCE)
set(LIBREADOUTCARD_INCLUDE_DIRS ${LIBREADOUTCARD_INCLUDE_DIR}
    CACHE STRING "Location of DAQ-provided readout card library headers" FORCE)

mark_as_advanced(FORCE LIBREADOUTCARD_INCLUDE_DIR LIBREADOUTCARD_LIBRARY)
