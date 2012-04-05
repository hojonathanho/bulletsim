#include "utils_python.h"
#include <iostream>
#include <string>
using namespace std;

struct Point {
  double x;
  double y;
};

int main() {
  try {
    cout << "hi" << endl;
    setup_python();
    py::object L = PyGlobals::builtin_module.attr("list")();
    L.attr("append")(234);
    L.attr("append")(32394);
    int x = py::extract<int>(L[0]);
    cout << "should be 234: " << x << endl;
    cout << "should be 32394: " << py::extract<int>(L[1]) << endl;
    
    
    Point parr[2];
    parr[0].x=0;
    parr[0].y=0;
    parr[1].x=1;
    parr[1].y=1;
    char* carr = (char*) parr;
    string s(carr, sizeof(Point)*2);

    py::object numpy = py::import("numpy");
    py::exec("import numpy as np; Point = np.dtype([('x',float), ('y',float)]);", PyGlobals::main_namespace);
    py::object point_dtype = PyGlobals::main_namespace["Point"];
    py::object arr = numpy.attr("fromstring")(s, point_dtype);
    PyGlobals::builtin_module.attr("print")(arr);
    
  }
  catch (...) {
    
    PyErr_Print();
    PyErr_Clear();    
    
  }
  
}