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
# Options
#
OPTION(BUILD_WITH_ROS "Build with ROS" OFF)

#
# Dependencies
#
find_package(pybind11 CONFIG REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# local depends
find_package(mpi_cmake_modules REQUIRED)
find_package(odri_control_interface REQUIRED)
find_package(master_board_sdk REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(yaml_utils REQUIRED)

if (BUILD_WITH_ROS)
  find_package(rclcpp REQUIRED)
endif()

# Find resources from robot_properties packages.
find_package(PythonModules COMPONENTS robot_properties_nyu_finger)

#
# Optional dependency on dynamic graph manager.
#
find_package(dynamic_graph_manager QUIET)

#
# Manage the creation of the libraries and executables.
#
add_subdirectory(src)

#
# Python bindings
#
set(py_nyu_finger_SRC_FILES
  srcpy/py_nyu_finger.cpp
)
pybind11_add_module(py_nyu_finger MODULE ${py_nyu_finger_SRC_FILES})
target_link_libraries(py_nyu_finger PRIVATE pybind11::module)
target_link_libraries(py_nyu_finger PRIVATE nyu_finger)

# install the bindings
get_python_install_dir(python_install_dir)
install(TARGETS py_nyu_finger DESTINATION ${python_install_dir}/${PROJECT_NAME})

# If dynamic graph manager was found, then build the hardware process bindings.
if(${dynamic_graph_manager_FOUND})
  set(nyu_finger_hwp_SRC_FILES
    src/dynamic_graph_manager/dgm_nyu_finger.cpp
    srcpy/nyu_finger_hwp.cpp
  )
  pybind11_add_module(nyu_finger_hwp_cpp MODULE ${nyu_finger_hwp_SRC_FILES})
  target_link_libraries(nyu_finger_hwp_cpp PRIVATE pybind11::module)
  target_link_libraries(nyu_finger_hwp_cpp PRIVATE nyu_finger)
  target_link_libraries(nyu_finger_hwp_cpp PRIVATE dynamic_graph_manager::dynamic_graph_manager)

  # Define flag if building with ros.
  if (BUILD_WITH_ROS)
    target_compile_definitions(nyu_finger_hwp_cpp PUBLIC BUILD_WITH_ROS="true")
  endif()

  install(TARGETS nyu_finger_hwp_cpp DESTINATION ${python_install_dir}/${PROJECT_NAME})
endif(${dynamic_graph_manager_FOUND})


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
