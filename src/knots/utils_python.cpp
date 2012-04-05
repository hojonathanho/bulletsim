#include "utils_python.h"

py::object PyGlobals::main_module;
py::object PyGlobals::main_namespace;
py::object PyGlobals::builtin_module;
py::object PyGlobals::numpy_module;

void setup_python() {
    Py_Initialize();
    //    _import_array(); // no idea what this does but I got the idea here http://www.mail-archive.com/numpy-discussion@scipy.org/msg09372.html

    PyGlobals::main_module = py::import("__main__");
    PyGlobals::main_namespace = PyGlobals::main_module.attr("__dict__");
    PyGlobals::builtin_module = py::import("__builtin__");
    PyGlobals::numpy_module = py::import("numpy");

    py::exec("import sys", PyGlobals::main_namespace);
    py::exec("sys.argv = ['use_pr2_from_cpp']", PyGlobals::main_namespace); // otherwise sys.argv is none and ros imports give errors
}
