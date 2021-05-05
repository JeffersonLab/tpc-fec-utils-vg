set(DOXYGEN_SOURCE_DIRS ${CMAKE_SOURCE_DIR}
                        ${CMAKE_SOURCE_DIR}/src
                        ${CMAKE_SOURCE_DIR}/src/trorc
                        ${CMAKE_SOURCE_DIR}/src/cru)
set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})

string(REGEX REPLACE ";" " " DOXYGEN_INPUT "${DOXYGEN_SOURCE_DIRS}")
configure_file(Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

set(HTML_TARGET ${DOXYGEN_OUTPUT_DIR}/html/index.html)
add_custom_target(doc
                  COMMAND ${DOXYGEN} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
                  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
