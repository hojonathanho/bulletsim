#pragma once
#include "numpy_boost.hpp"
#include <boost/python.hpp>
#include <iostream>
#include <string>
using namespace std;
namespace py = boost::python;
typedef numpy_boost<float, 1> npVectorf;
typedef numpy_boost<float, 2> npMatrixf;
typedef numpy_boost<double, 1> npVectord;
typedef numpy_boost<double, 2> npMatrixd;

struct PyGlobals {
  static py::object main_module;
  static py::object main_namespace;
  static py::object builtin_module;
};

void setup_python();

template <class npArrayType>
py::object toObject(npArrayType array) {
    PyObject* ptr = array.py_ptr();
    py::object arrayobj(py::handle<>(py::borrowed(ptr)));
    return arrayobj;
}
