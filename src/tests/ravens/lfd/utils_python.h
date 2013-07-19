/** Boost::python utils. Borrowed from John Schulman. */

#pragma once

#include <boost/python.hpp>
#include <iostream>
#include <string>
#include <simulation/simplescene.h>

using namespace std;
namespace py = boost::python;

struct PyGlobals {
  static py::object main_module;
  static py::object main_namespace;
  static py::object builtin_module;
  static py::object numpy_module;
  static py::object lfd_registration_module;
  static py::object joschu_lfd_registration_module;
  static py::object resampling_module;
  static py::object openrave_module;
  static py::object math_module;
  static py::object iros_utils_module;
  static py::object None;
};

#define NP PyGlobals::numpy_module

/** function to extract c++ floats and ints from python objects. */
static inline float ff(py::object o) {return py::extract<float>(o);}
static inline int   ii(py::object o) {return py::extract<int>(o);}
static inline double dd(py::object o) {return py::extract<double>(o);}

void setup_python();

/** Converts a vector of btVector3 (list of 3d points) to a 2D numpy array.*/
py::object pointsToNumpy(const std::vector<btVector3>& pts);

/** Reverse of ptsToNumpy. Converts a 2D numpy array into a vector of btVector3.
 *  Assuming that there are THREE columns. */
vector<btVector3> pointsFromNumpy(const py::object &py_pts);

/** Converts a vector of doubles to a numpy array.*/
py::object vectorToNumpy(const std::vector<double>& vec);

/** Converts a numpy array to a vector of doubles.*/
//vector<double> vectorFromNumpy(const py::object &py_vec);

/** Converts a vector of floats to a numpy array.*/
py::object vectorToNumpy(const std::vector<float>& vec);

/** Converts a numpy array to a vector of floats.*/
vector<float> vectorFromNumpy(const py::object &py_vec);


/** Reverse of ptsToNumpy. Converts a 2D numpy array into a vector of btVector3.
 *  Assuming that there are THREE columns. */
vector<btVector3> pointsFromNumpy(const py::object &py_pts);

/** Converts a vector of btMatrix3x3 to a 3D numpy array.*/
py::object rotationsToNumpy(const vector<btMatrix3x3>& rots);

/** Reverse of ptsToNumpy. Converts a 2D numpy array into a vector of btVector3.
 *  Assuming that there are THREE columns. */
vector<btMatrix3x3> rotationsFromNumpy(const py::object &py_rots);

/** Converts a vector of btTransform to a 3D numpy array.*/
py::object transformsToNumpy(const vector<btTransform>& trans);

/** Reverse of above. */
vector<btTransform> transformsFromNumpy(const py::object &py_tfms);

/** Converts a 2D Numpy array into a vector of vector of doubles. */
vector<vector<double> > jointsFromNumpy(py::object array2d);

/** Converts a vector of vectors of doubles into a numpy array. */
py::object jointsToNumpy( const vector< vector<dReal> > &joints);

/** Does linear interpolation on data to return samples as times specified in SAMPLE_TIMES.
 *  The input data is 2D, at TIME_STAMPS. */
vector<vector<double> > interpolate(const vector<float> & sample_times, const vector<vector<double > > & data, const vector<float> & time_stamps);
vector<vector<double> > interpolateD(const vector<double> & sample_times, const vector<vector<double > > & data, const vector<double> & time_stamps);


/** Resample original signal with a small number of waypoints so that the the sparsely sampled function,
 *   when linearly interpolated, deviates from the original function by less than TOL at every time
 *
 *   @params:
 *       x         : 2D array in R^(t x k)  where t is the number of timesteps
 *       tol       : tolerance. either a single scalar or a vector of length k
 *       max_change: max change in the sparsely sampled signal at each timestep
 *       min_steps : minimum number of timesteps in the new trajectory. (usually irrelevant)
 *
 *  output:
 *  new_times, new_x
 *
 *  assuming that the old signal has times 0,1,2,...,len(x)-1
 *  this gives the new times, and the new signal
 *
 *
 *  @return:
 *    returns a pair: - first is a vector of times the data was resampled.
 *                      note: the times are such that in the input data the ith vector is assumed to be a sample at time =i.
 *                    - second is the actual data (2 dimensional)*/
pair< vector<float>, vector< vector <double> > >
	adaptive_resample (const vector < vector <double> > & in_signal, double tol, double max_change=-1, int min_steps=3);


/** saves the point-clouds in a numpy .npz file.
 *  fname       :  name of the .npz file
 *  cloud_names :  names of the clouds
 *  clouds      :  the point-clouds to be saved. */
void saveClouds(const std::string fname, const vector<std::string> &cloud_names, const vector<vector<btVector3> > &clouds);


