/** Boost::python utils. Borrowed from John Schulman. */

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
  static py::object lfd_registration_module;
};

#define NP PyGlobals::numpy_module

/** function to extract c++ floats and ints from python objects. */
static inline float ff(py::object o) {return py::extract<float>(o);}
static inline int   ii(py::object o) {return py::extract<int>(o);}

void setup_python();

/** Converts a vector of btVector3 (list of 3d points) to a 2D numpy array.*/
py::object ptsToNumpy(const vector<btVector3>& pts);

/** Reverse of ptsToNumpy. Converts a 2D numpy array into a vector of btVector3.
 *  Assuming that there are THREE columns. */
vector<btVector3> ptsFromNumpy(const py::object &py_pts);


/** Converts a vector of btTransforms to a 3D numpy array.*/
py::object rotationsToNumpy(const vector<btMatrix3x3>& rots);

/** Reverse of ptsToNumpy. Converts a 2D numpy array into a vector of btVector3.
 *  Assuming that there are THREE columns. */
vector<btMatrix3x3> rotationsFromNumpy(const py::object &py_rots);
