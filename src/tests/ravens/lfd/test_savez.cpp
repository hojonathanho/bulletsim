#include "utils_python.h"
#include <simulation/simplescene.h>
#include <vector>
#include <iostream>


using namespace std;


/** saves the point-clouds in a numpy .npz file.*/
void saveClouds(const std::string fname,
		const vector<std::string> &cloud_names, const vector<vector<btVector3> > &clouds) {
	assert(("saveClouds : Number of names and clouds do not match.", clouds.size()==cloud_names.size()));

	py::dict pyclouds;
	for (int c=0; c < clouds.size(); c++)
		pyclouds[cloud_names[c]] = pointsToNumpy(clouds[c]);

	try {
		py::list args; args.append(fname);
		PyGlobals::numpy_module.attr("savez")(*py::tuple(args), **pyclouds);
	} catch (...) {
		PyErr_Print();
	}
}


int main(int argc, char**argv) {

	setup_python();

	vector<btVector3> c1;
	c1.push_back(btVector3(0,0,0));
	c1.push_back(btVector3(1,1,1));

	vector<btVector3> c2;
	c2.push_back(btVector3(-1,0,-1));

	vector<vector<btVector3> > clouds;
	clouds.push_back(c1);
	clouds.push_back(c2);

	vector<string> names;
	names.push_back("c1");
	names.push_back("c2");

	saveClouds("test_clouds", names, clouds);

	return 0;
}
