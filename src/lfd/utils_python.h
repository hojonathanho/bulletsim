#pragma once
#include <boost/python.hpp>
#include <iostream>
#include <string>
using namespace std;
namespace py = boost::python;

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

struct PyGlobals {
  static py::object main_module;
  static py::object main_namespace;
  static py::object builtin_module;
  static py::object numpy_module;
};

static inline float ff(py::object o) {return py::extract<float>(o);}
static inline int ii(py::object o) {return py::extract<int>(PyGlobals::builtin_module.attr("int")(o));}

#define NP PyGlobals::numpy_module

void Python_setup();
py::object Python_importFile(const fs::path &path);

class PyModule {
protected:
  PyModule(const fs::path &path);
  py::object module;
  py::object getModule() const { return module; }
};
