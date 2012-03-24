#include "utils_python.h"

void setup_python() {
    Py_Initialize();
    _import_array(); // no idea what this does but I got the idea here http://www.mail-archive.com/numpy-discussion@scipy.org/msg09372.html

    main_module = py::import("__main__");
    main_namespace = main_module.attr("__dict__");

    py::exec("import sys", main_namespace);
    py::exec("sys.argv = ['use_pr2_from_cpp']", main_namespace); // otherwise sys.argv is none and ros imports give errors
}
