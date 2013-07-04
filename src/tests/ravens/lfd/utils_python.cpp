#include "utils_python.h"

py::object PyGlobals::main_module;
py::object PyGlobals::main_namespace;
py::object PyGlobals::builtin_module;
py::object PyGlobals::numpy_module;
py::object PyGlobals::lfd_registration_module;
py::object PyGlobals::openrave_module;
py::object PyGlobals::resampling_module;
py::object PyGlobals::math_module;
py::object PyGlobals::iros_utils_module;
py::object PyGlobals::None;

void setup_python() {
	Py_Initialize();

	PyGlobals::main_module    = py::import("__main__");
	PyGlobals::main_namespace = PyGlobals::main_module.attr("__dict__");
	PyGlobals::builtin_module = py::import("__builtin__");
	PyGlobals::numpy_module   = py::import("numpy");

	py::exec("import sys", PyGlobals::main_namespace);
	py::exec("sys.argv = ['call_lfd_from_cpp']", PyGlobals::main_namespace); // otherwise sys.argv is none and ros imports give errors

	PyGlobals::lfd_registration_module =  py::import("rapprentice.registration");
	PyGlobals::resampling_module = py::import("rapprentice.resampling");
	PyGlobals::openrave_module   = py::import("openravepy");
	PyGlobals::math_module       = py::import("jds_utils.math_utils");
	PyGlobals::iros_utils_module = py::import("iros.iros_utils");
	PyGlobals::None              = py::api::object();
}

/** Converts a vector of btVector3 (list of 3d points) to a 2D numpy array.*/
py::object pointsToNumpy(const vector<btVector3>& pts) {
	int nRows = pts.size();
	int nCols = 3;
	py::object out = NP.attr("zeros")(py::make_tuple(nRows,nCols));

	for (int i=0; i < nRows; i++) {
		out[i][0] = pts[i].x();
		out[i][1] = pts[i].y();
		out[i][2] = pts[i].z();
	}
	return out;
}


/** Reverse of ptsToNumpy. Converts a 2D numpy array into a vector of btVector3.
 *  Assuming that there are THREE columns. */
vector<btVector3> pointsFromNumpy(const py::object &py_pts) {
	int n = ii(py_pts.attr("__len__")());
	vector<btVector3> out_pts(n);

	for (int i=0; i < n; i++) {
		py::object py_pt = py_pts[i];

		float x = ff(py_pt[0]);
		float y = ff(py_pt[1]);
		float z = ff(py_pt[2]);

		btVector3 pt(x,y,z);
		out_pts[i] = pt;
	}
	return out_pts;
}

/** Converts a vector of doubles to a numpy array.*/
py::object vectorToNumpy(const std::vector<double>& vec) {
	int len = vec.size();
	py::object out = NP.attr("zeros")(len);

	for (int i=0; i<len; ++i) out[i] = vec[i];

	return out;
}

/** Converts a numpy array to a vector of doubles.*\/
vector<double> vectorFromNumpy(const py::object &py_vec) {
	int n = ii(py_vec.attr("__len__")());
	vector<double> out_vec(n);

	for (int i=0; i < n; i++) out_vec.push_back(dd(py_vec[i]));

	return out_vec;
}*/

/** Converts a vector of floats to a numpy array.*/
py::object vectorToNumpy(const std::vector<float>& vec) {
	int len = vec.size();
	py::object out = NP.attr("zeros")(len);

	for (int i=0; i<len; ++i) out[i] = vec[i];

	return out;
}

/** Converts a numpy array to a vector of doubles.*/
vector<float> vectorFromNumpy(const py::object &py_vec) {
	int n = ii(py_vec.attr("__len__")());
	vector<float> out_vec(n);

	for (int i=0; i < n; i++) out_vec[i] = ff(py_vec[i]);

	return out_vec;
}

/** Converts a vector of btMatrix3x3 to a 3D numpy array.*/
py::object rotationsToNumpy(const vector<btMatrix3x3>& rots) {
	int n = rots.size();
	py::object out = NP.attr("zeros")(py::make_tuple(n, 3, 3));

	for (int i=0; i < n; i++) {
		const btMatrix3x3 & mat = rots[i];
		for (int j=0; j < 3; j+= 1) {
			for (int k=0; k <3; k+=1) {
				out[i][j][k] = mat[j][k];
			}
		}
	}
	return out;
}


/** Reverse of rotationsToNumpy. Converts a 3D numpy array into a vector of btMatrix3x3.
 *  Assumes that there the dimensions are nx3x3. */
vector<btMatrix3x3> rotationsFromNumpy(const py::object &py_rots) {
	int n = ii(py_rots.attr("__len__")());
	vector<btMatrix3x3> out_rots(n);

	for (int i=0; i < n; i++) {
		py::object py_mat = py_rots[i];
		for (int j=0; j < 3; j+= 1) {
			for (int k=0; k <3; k+=1) {
				out_rots[i][j][k] = ff(py_mat[j][k]);
			}
		}
	}
	return out_rots;
}


/** Converts a vector of btTransform to a 3D numpy array.*/
py::object transformsToNumpy(const vector<btTransform>& trans) {
	int n = trans.size();
	py::object out = NP.attr("zeros")(py::make_tuple(n, 4, 4));

	for (int i=0; i < n; i++) {
		const btTransform & mat = trans[i];
		for (int j=0; j < 3; j+= 1) {
			for (int k=0; k <3; k+=1) {
				out[i][j][k] = mat.getBasis()[j][k];
			}
		}
		out[i][3][0] = 0; out[i][3][1] = 0; out[i][3][2] = 0; out[i][3][3] = 1;
		out[i][0][3] = mat.getOrigin()[0];
		out[i][1][3] = mat.getOrigin()[1];
		out[i][2][3] = mat.getOrigin()[2];
	}
	return out;
}

/** Reverse of rotationsToNumpy. Converts a 3D numpy array into a vector of btMatrix3x3.
 *  Assumes that there the dimensions are nx3x3. */
vector<btTransform> transformsFromNumpy(const py::object &py_tfms) {
	int n = ii(py_tfms.attr("__len__")());
	vector<btTransform> out_frames(n);

	for (int i=0; i < n; i++) {
		py::object py_tfm = py_tfms[i];

		// fill in rotation:
		btMatrix3x3 tmp_rot;
		for (int j=0; j < 3; j+= 1) {
			for (int k=0; k <3; k+=1) {
				tmp_rot[j][k] = ff(py_tfm[j][k]);
			}
		}

		// fill in translation:
		btVector3 tmp_trans;
		for(int j=0; j < 3; j+=1) {
			tmp_trans[j] = ff(py_tfm[j][3]);
		}
		out_frames[i].setIdentity();
		out_frames[i].setBasis(tmp_rot);
		out_frames[i].setOrigin(tmp_trans);
	}
	return out_frames;
}



/** Converts a vector of vectors of doubles into a numpy array. */
py::object jointsToNumpy( const vector< vector<dReal> > &joints) {
	const int N = joints.size();
	if (N > 0) { // ensure that there is at least one vector inside
		const int M = joints[0].size();
		py::object out = NP.attr("zeros")(py::make_tuple(N, M));
		 for (int i =0 ; i < N; i+=1) {
			 assert(("Numbers of elements in the second dimension are not the same.", joints[i].size()==M));
			 const vector<dReal> & joint_set = joints[i];
			 for (int j=0; j < M; j+=1)
				 out[i][j] = joint_set[j];
		 }
		return out;
	}
	return NP.attr("zeros")(py::make_tuple(0, 0));;
}


/** Converts a 2D Numpy array into a vector of vector of doubles. */
vector<vector<double> > jointsFromNumpy(py::object array2d) {
	int N = ii(array2d.attr("__len__")());
	vector <vector <dReal> > array;
	for (int i=0; i < N; i+=1) {
		vector<double> vals;
		py::object sub_array = array2d[i];
		int num_vals = ii(sub_array.attr("__len__")());

		for (int j=0; j<num_vals; j+=1)
			vals.push_back(dd(sub_array[j]));

		array.push_back(vals);
	}
	assert(("Extracted vectors' length does not match the array length.", array.size()==N));
	return array;
}

/** Does linear interpolation on data to return samples as times specified in SAMPLE_TIMES.
 *  The input data is 2D, at TIME_STAMPS. */
vector<vector<double> > interpolate(const vector<float> & sample_times, const vector<vector<double > > & data, const vector<float> & time_stamps) {

	py::object py_sample_times = vectorToNumpy(sample_times);
	py::object py_data         = jointsToNumpy(data);
	py::object py_time_stamps  = vectorToNumpy(time_stamps);

	py::object interp2d          = PyGlobals::math_module.attr("interp2d");
	py::object interpolated_data;

	try {
		interpolated_data = interp2d(py_sample_times, py_time_stamps, py_data);
	} catch(...) {
		PyErr_Print();
	}

	vector<vector<double> > out = jointsFromNumpy(interpolated_data);
	return out;
}


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
	adaptive_resample (const vector < vector <double> > & in_signal, double tol, double max_change, int min_steps) {


	py::object py_signal     = jointsToNumpy(in_signal);
	py::object times_samples;

	try {
		times_samples = PyGlobals::iros_utils_module.attr("adaptive_resample")(py_signal, tol, max_change, min_steps);
	} catch(...) {
		PyErr_Print();
	}

	py::object py_times      = times_samples[0];
	py::object py_samples    = times_samples[1];

	vector<float> times = vectorFromNumpy(py_times);
	vector<vector<double> > samples = jointsFromNumpy(py_samples);
	pair <vector<float>, vector<vector<double> > > out = make_pair(times, samples);
	return out;
}


/** Saves the point-clouds in a numpy .npz file.
 *  fname       :  name of the .npz file
 *  cloud_names :  names of the clouds
 *  clouds      :  the point-clouds to be saved. */
void saveClouds(const std::string fname,
		const vector<std::string> &cloud_names, const vector<vector<btVector3> > &clouds) {
	assert(("saveClouds : Number of names and clouds do not match.", clouds.size()==cloud_names.size()));

	py::dict pyclouds;
	for (int c=0; c < clouds.size(); c++)
		pyclouds[cloud_names[c]] = pointsToNumpy(clouds[c]);

	try {
		py::list args; args.append(fname);
		NP.attr("savez")(*py::tuple(args), **pyclouds);
	} catch (...) {
		PyErr_Print();
	}
}
