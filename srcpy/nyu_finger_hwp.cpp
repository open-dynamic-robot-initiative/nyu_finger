/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2021, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the nyu finger hardware process.
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "nyu_finger/dynamic_graph_manager/dgm_nyu_finger.hpp"

namespace nyu_finger
{

namespace py = pybind11;

PYBIND11_MODULE(nyu_finger_hwp_cpp, m)
{
    m.doc() = R"pbdoc(
        NYUFinger HardWareProcess bindigns
        ---------------------------------
        .. currentmodule:: mim_control
        .. autosummary::
           :toctree: _generate
           add
           subtract
    )pbdoc";

    py::class_<DGMNYUFinger>(m, "NYUFingerHWP")
        .def(py::init<>())
        .def("initialize", &DGMNYUFinger::initialize)
        .def("calibrate", &DGMNYUFinger::calibrate_joint_position)
        .def("run", &DGMNYUFinger::run)
    ;
}

}
