#pragma once
#include <boost/python.hpp>
#include <iostream>
#include <string>
using namespace std;
namespace py = boost::python;



struct PyGlobals {
  static py::object main_module;
  static py::object main_namespace;
  static py::object builtin_module;
  static py::object numpy_module;
};

static inline float ff(py::object o) {return py::extract<float>(o);}
static inline int ii(py::object o) {return py::extract<int>(PyGlobals::builtin_module.attr("int")(o));}


#define NP PyGlobals::numpy_module

void setup_python();
