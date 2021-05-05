add_custom_target(updateGitInfo ALL
    ${CMAKE_SOURCE_DIR}/contrib/updateGitInfo.sh ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Updating git information"
    VERBATIM
  )

set_source_files_properties(
    ${CMAKE_CURRENT_BINARY_DIR}/git_info.txt
    ${CMAKE_CURRENT_BINARY_DIR}/git_info.hpp
    PROPERTIES GENERATED TRUE
  )

execute_process(
    COMMAND ${CMAKE_SOURCE_DIR}/contrib/updateGitInfo.sh ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} 1
    OUTPUT_VARIABLE CMAKE_GIT_INFO_TXT
  )

include_directories(${CMAKE_CURRENT_BINARY_DIR})
