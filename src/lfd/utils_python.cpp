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

// Tries to import a python module from the given file (using Python_importFile).
// If the file isn't found, then this will search for the file in the parent directory.
// If it isn't there, then this keeps going up.
PyModule::PyModule(const fs::path &path) {
  fs::path p = fs::path(".") / path;
  const int MAX_DEPTH = 10;
  for (int i = 0; i < MAX_DEPTH; ++i) {
    LOG_DEBUG("searching for " << p);
    if (fs::exists(p)) {
      module = Python_importFile(p);
      return;
    }
    p = fs::path("..") / p;
  }
  throw runtime_error("could not load python module");
}

py::object pointVecToNP(const vector<btVector3> &v) {
  py::object o = NP.attr("empty")(py::make_tuple(v.size(), 3));
  for (int i = 0; i < v.size(); ++i) {
    o[i][0] = v[i].x();
    o[i][1] = v[i].y();
    o[i][2] = v[i].z();
  }
  return o;
}

vector<btVector3> NPnx3ToPointVec(py::object n) {
  vector<btVector3> v;
  for (int i = 0; i < py::len(n); ++i) {
    v.push_back(btVector3(
      py::extract<btScalar>(n[i][0]),
      py::extract<btScalar>(n[i][1]),
      py::extract<btScalar>(n[i][2])
    ));
  }
  return v;
}
