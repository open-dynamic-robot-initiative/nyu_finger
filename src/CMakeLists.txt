
set(all_src_targets)
set(all_exe_targets)

#
# Declare the project library.
#
macro(create_robots_library lib_name)
  add_library(${lib_name} SHARED
    ${lib_name}.cpp
  )

  target_link_libraries(${PROJECT_NAME}
                        odri_control_interface::odri_control_interface)
  target_link_libraries(${PROJECT_NAME}
                        master_board_sdk::master_board_sdk)
  target_link_libraries(${PROJECT_NAME}
                        real_time_tools::real_time_tools)
  target_link_libraries(${PROJECT_NAME} yaml_utils::yaml_utils)
  target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

  if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(${lib_name} rt pthread)
  endif()

  # Includes. Add the include dependencies
  target_include_directories(
    ${lib_name} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include>)

  list(APPEND all_src_targets ${lib_name})
endmacro()

#
# build the Robots #
#

create_robots_library(nyu_finger)

#
# Install and Export the libraries.
#
install(
  TARGETS ${all_src_targets}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)

# we also need to install the header files
install(DIRECTORY ../include/ DESTINATION include)

foreach(target ${all_exe_targets})
  # install a symlink of the executable in lib/bolt
  string(
    CONCAT symlink_command
           "execute_process("
           "  COMMAND ${CMAKE_COMMAND} -E make_directory "
           "  ${CMAKE_INSTALL_PREFIX}/bin/)\n"
           "execute_process("
           "  COMMAND ${CMAKE_COMMAND} -E create_symlink "
           "  ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${target}"
           "  ${CMAKE_INSTALL_PREFIX}/bin/${target} )"
  )
  install(CODE ${symlink_command})
endforeach()
