#include "utils_python.h"
#include "utils/logging.h"
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

py::object PyGlobals::main_module;
py::object PyGlobals::main_namespace;
py::object PyGlobals::builtin_module;
py::object PyGlobals::numpy_module;

void Python_setup() {
  Py_Initialize();
  //    _import_array(); // no idea what this does but I got the idea here http://www.mail-archive.com/numpy-discussion@scipy.org/msg09372.html

  PyGlobals::main_module = py::import("__main__");
  PyGlobals::main_namespace = PyGlobals::main_module.attr("__dict__");
  PyGlobals::builtin_module = py::import("__builtin__");
  PyGlobals::numpy_module = py::import("numpy");

  py::exec("import sys", PyGlobals::main_namespace);
  py::exec("sys.argv = ['use_pr2_from_cpp']", PyGlobals::main_namespace); // otherwise sys.argv is none and ros imports give errors

  // so we can make py::lists out of vector<double>
  py::class_<std::vector<double> >("double_vector")
    .def(py::vector_indexing_suite<std::vector<double> >());
}

// Tries to import a python module given the path to the .py file
// from http://wiki.python.org/moin/boost.python/EmbeddingPython
py::object Python_importFile(const fs::path &path) {
  try {
    py::dict locals;
    locals["modulename"] = path.stem().string(); // foo -> module name
    locals["path"] = path.string(); // /Users/whatever/blah/foo.py
    py::exec(
      "import imp; newmodule = imp.load_module(modulename,open(path),path,('py','U',imp.PY_SOURCE))",
      PyGlobals::main_namespace, locals
    );
    LOG_DEBUG("loaded python module " << path.string());
    return locals["newmodule"];
  } catch (const py::error_already_set &e) {
    PyErr_Print();
    throw e;
  }
}

PyModule::PyModule(const fs::path &path) {
  module = Python_importFile(path);
}
