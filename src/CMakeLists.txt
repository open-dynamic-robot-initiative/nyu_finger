
set(all_src_targets)
set(all_exe_targets)

#
# Declare the project library.
#
macro(create_robots_library lib_name)
  add_library(${lib_name} SHARED
    ${lib_name}.cpp
  )

  target_link_libraries(${PROJECT_NAME} blmc_drivers::blmc_drivers)
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
    ${lib_name} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include>)

  list(APPEND all_src_targets ${lib_name})
endmacro()

#
# build the Robots #
#

create_robots_library(nyu_finger)


#
# Optionally build the DynamiGraphManager main programs.
#
if(${dynamic_graph_manager_FOUND})
  message(STATUS "Build the dynamic_graph_manager main programs.")
  macro(build_dgm_programs robot_name)
    add_executable(dg_main_${robot_name}
      dynamic_graph_manager/dgm_${robot_name}.cpp
      dynamic_graph_manager/dg_main_${robot_name}.cpp
    )
    # Add the include dependencies.
    target_include_directories(
      dg_main_${robot_name} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>)
    # Link the dependencies.
    target_link_libraries(dg_main_${robot_name}
      ${robot_name} dynamic_graph_manager::dynamic_graph_manager)
    # Get the yaml configuration for the DGM.
    string(
    CONCAT dgm_yaml_path
         "${PythonModules_robot_properties_nyu_finger_PATH}/"
         "robot_properties_nyu_finger/robot_properties_nyu_finger/dynamic_graph_manager/")
    target_compile_definitions(dg_main_${robot_name}
      PUBLIC ROBOT_PROPERTIES_YAML_PATH="${dgm_yaml_path}")
    # Export the target.
    list(APPEND all_src_targets dg_main_${robot_name})
    list(APPEND all_exe_targets dg_main_${robot_name})
  endmacro()
  build_dgm_programs(nyu_finger)
  build_dgm_programs(nyu_finger_double)
endif(${dynamic_graph_manager_FOUND})

#
# Install and Export the libraries.
#
install(
  TARGETS ${all_src_targets}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)

# we also need to install the header files
install(DIRECTORY include/ DESTINATION include)

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
