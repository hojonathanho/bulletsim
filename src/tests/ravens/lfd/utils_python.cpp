#include "utils_python.h"

py::object PyGlobals::main_module;
py::object PyGlobals::main_namespace;
py::object PyGlobals::builtin_module;
py::object PyGlobals::numpy_module;

void setup_python() {
	Py_Initialize();

	PyGlobals::main_module    = py::import("__main__");
	PyGlobals::main_namespace = PyGlobals::main_module.attr("__dict__");
	PyGlobals::builtin_module = py::import("__builtin__");
	PyGlobals::numpy_module   = py::import("numpy");
	PyGlobals::lfd_registration_module =  py::import("lfd.registration");

	//py::exec("import lfd.registration", PyGlobals::main_namespace);
	//py::exec("sys.argv = ['use_pr2_from_cpp']", PyGlobals::main_namespace); // otherwise sys.argv is none and ros imports give errors
}


/** Converts a vector of btVector3 (list of 3d points) to a 2D numpy array.*/
py::object ptsToNumpy(const vector<btVector3>& pts) {
	int nRows = pts.size();
	int nCols = 3;
	py::object out = NP.attr("zeros")(py::make_tuple(nRows,nCols));

	for (int i=0; i < pts.size(); i++) {
		out[i][0] = pts[i].x();
		out[i][1] = pts[i].y();
		out[i][2] = pts[i].z();
	}
	return out;
}


/** Reverse of ptsToNumpy. Converts a 2D numpy array into a vector of btVector3.
 *  Assuming that there are THREE columns. */
vector<btVector3> ptsfromNumpy(py::object py_pts) {
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


/** Converts a vector of btMatrix3x3 to a 3D numpy array.*/
py::object rotationsToNumpy(const vector<btMatrix3x3>& rots) {
	int n = rots.size();
	py::object out = NP.attr("zeros")(py::make_tuple(n, 3, 3));

	for (int i=0; i < n; i++) {
		btMatrix3x3 & mat = rots[i];
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
