#include "simulation/simplescene.h"
#include "simulation/plotting.h"
#include <vector>
#include <iostream>

using namespace std;


void linspace(float a, float b, int n, std::vector<float> &out) {
	out.clear(); out.resize(n);
	if (n<1) return;

	float m = (b-a)/n;
	for (int i=0; i<n; i++)
		out[i] = a + i*m;
}


void grid3d (btVector3 mins, btVector3 maxs, int ncoarse, int nfine, std::vector<vector<btVector3> > &out) {

	vector<float> xcoarse, xfine;
	linspace(mins.x(), maxs.x(), ncoarse, xcoarse);
	linspace(mins.x(), maxs.x(), nfine, xfine);

	vector<float> ycoarse, yfine;
	linspace(mins.y(), maxs.y(), ncoarse, ycoarse);
	linspace(mins.y(), maxs.y(), nfine, yfine);

	vector<float> zcoarse, zfine;
	linspace(mins.z(), maxs.z(), ncoarse, zcoarse);
	linspace(mins.z(), maxs.z(), nfine, zfine);

	out.clear();

	for(int iz=0; iz< ncoarse; iz++) {

		// generate x-lines
		for (int ix = 0; ix < ncoarse; ix++) {
			vector<btVector3> xline(nfine);
			for(int iy = 0; iy < nfine; iy++)
				xline[iy] = btVector3(xcoarse[ix], yfine[iy], zcoarse[iz]);
			out.push_back(xline);
		}

		// generate y-lines
		for (int iy = 0; iy < ncoarse; iy++) {
			vector<btVector3> yline(nfine);
			for(int ix = 0; ix < nfine; ix++)
				yline[ix] = btVector3(xfine[ix], ycoarse[iy], zcoarse[iz]);
			out.push_back(yline);
		}
	}

	//generate z-lines
	for(int ix=0 ; ix < ncoarse; ix++) {
		for(int iy= 0; iy < ncoarse; iy++) {
			vector<btVector3> zline(nfine);
			for(int iz = 0; iz < nfine; iz++) {
				zline[iz] = btVector3(xcoarse[ix], ycoarse[iy], zfine[iz]);
			}
			out.push_back(zline);
		}
	}
}


int main(int argc, char *argv[]) {

	srand(0);

	// construct the scene
	Scene scene;

	btVector3 mins(0,0,0);
	btVector3 maxs(1,1,1);

	float ncoarse = 5;
	float nfine   = 30;

	vector<vector<btVector3> > grid(2);
	grid3d(mins, maxs, ncoarse, nfine, grid);

	PlotLinesSet::Ptr plines(new PlotLinesSet);
	scene.env->add(plines);

	for(int i=0; i < grid.size(); i++) {
		plines->addLineSet(grid[i]);
	}

	// start the simulation
	scene.startViewer();
	scene.startLoop();

	return 0;
}

