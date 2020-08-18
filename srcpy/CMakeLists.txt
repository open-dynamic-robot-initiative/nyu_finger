###################
# python bindings #
###################

##
set(py_nyu_finger_SRC_FILES
  ../src/nyu_finger.cpp
  py_nyu_finger.cpp
)
pybind11_add_module(py_nyu_finger ${py_nyu_finger_SRC_FILES})
target_link_libraries(py_nyu_finger PRIVATE nyu_finger ${catkin_LIBRARIES} ${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
set_target_properties(py_nyu_finger PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
install(TARGETS py_nyu_finger DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

