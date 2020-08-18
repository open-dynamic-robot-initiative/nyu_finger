###############################
# Declare the project library #
###############################

###############################################
# Declare a simple macro to build the library #
###############################################

macro(create_robots_library lib_name)
  add_library(${lib_name} SHARED
    ${lib_name}.cpp
  )

  target_link_libraries(${lib_name} ${catkin_LIBRARIES} ${PROJECT_NAME})

  if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(${lib_name} rt pthread)
  endif()

  set(LIBRARIES_TO_BE_EXPORTED ${LIBRARIES_TO_BE_EXPORTED} ${lib_name})
endmacro()

####################
# build the Robots #
####################

create_robots_library(nyu_finger)

################################################
# Declare a simple macro to build the programs #
################################################

macro(build_programs program_name robot_name)
  add_executable(${program_name}
    programs/${program_name}.cpp
  )
  target_link_libraries(${program_name}
    ${robot_name}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
endmacro(build_programs program_name robot_name)

######################
# Build the programs #
######################

######################################################
# Export the libraries in the devel folder of catkin #
######################################################

set(EXPORTED_LIBRAIRIES ${EXPORTED_LIBRAIRIES} ${LIBRARIES_TO_BE_EXPORTED}
    PARENT_SCOPE)
