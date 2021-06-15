#
# Copyright (c) 2020-2021, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(nyu_finger)

# specify the C++ 17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#
# Dependencies
#
find_package(pybind11 CONFIG REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# local depends
find_package(mpi_cmake_modules REQUIRED)
find_package(blmc_drivers REQUIRED)
find_package(odri_control_interface REQUIRED)
find_package(master_board_sdk REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(yaml_utils REQUIRED)

#
# Build main library and binaries
#

find_package(dynamic_graph_manager QUIET)

###############################################
# Declare a simple macro to build the library #
###############################################

macro(create_robots_library lib_name)
  add_library(${lib_name} SHARED
    src/${lib_name}.cpp
  )

  target_link_libraries(${PROJECT_NAME} INTERFACE blmc_drivers::blmc_drivers)
  target_link_libraries(${PROJECT_NAME}
                        INTERFACE odri_control_interface::odri_control_interface)
  target_link_libraries(${PROJECT_NAME}
                        INTERFACE master_board_sdk::master_board_sdk)
  target_link_libraries(${PROJECT_NAME}
                        INTERFACE real_time_tools::real_time_tools)
  target_link_libraries(${PROJECT_NAME} INTERFACE yaml_utils::yaml_utils)
  target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)

  if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(${lib_name} INTERFACE rt pthread)
  endif()

  # Includes. Add the include dependencies
  target_include_directories(
    ${lib_name} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)
endmacro()

#
# build the Robots #
#

create_robots_library(nyu_finger)

# command to install the library and binaries
install(
  TARGETS nyu_finger
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

# we also need to install the header files
install(DIRECTORY include/ DESTINATION include)

# #
# # Python bindings
# #
# set(py_nyu_finger_SRC_FILES
#   src/nyu_finger.cpp
#   srcpy/py_nyu_finger.cpp
# )
# pybind11_add_module(py_nyu_finger MODULE ${py_nyu_finger_SRC_FILES})
# target_link_libraries(py_nyu_finger PRIVATE pybind11::module)
# target_link_libraries(py_nyu_finger PRIVATE nyu_finger)

# # install the bindings
# get_python_install_dir(python_install_dir)
# install(TARGETS py_nyu_finger DESTINATION ${python_install_dir}/${PROJECT_NAME})

# install the python package too
install(
  DIRECTORY python/${PROJECT_NAME}
  DESTINATION "${python_install_dir}"
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE)

#
# create the cmake package
#
generate_cmake_package()

#
# building documentation
#
add_documentation()
