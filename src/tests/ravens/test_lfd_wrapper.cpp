#include <vector>
#include "lfd/LfdRpmWrapper.h"

using namespace std;

int main(int argc, char* argv[]) {

	setup_python();

	int NUM_POINTS = 100;

	//	 create a sample source data set
	vector<btVector3> src(NUM_POINTS);
	for (int i=0; i< NUM_POINTS; i+=1) {
		src[i] = btVector3(i/100.0, i/100.0, 0);
	}

	//	 create a sample target data set
	vector<btVector3> target(NUM_POINTS);
	for (int i=0; i< NUM_POINTS; i+=1) {
		target[i] = btVector3(i/100.0, i/200.0, 0);
	}

	RegistrationModule lfdrpm(src, target,20);

	vector<btVector3> query_pts(2);
	query_pts[0] = btVector3(0,0,0);
	query_pts[1] = btVector3(0.5,0.5,0);

	vector<btVector3> t_pts = lfdrpm.transform_points(query_pts);

	cout <<" transformed pt1: "<< t_pts[0].getX()<<","<< t_pts[0].getY()<<","<< t_pts[0].getZ()<<endl;
	cout <<" transformed pt2: "<< t_pts[1].getX()<<","<< t_pts[1].getY()<<","<< t_pts[1].getZ()<<endl;

	vector<btTransform> query_tfms(2);
	query_tfms[0] = btTransform::getIdentity();
	query_tfms[0].setOrigin(btVector3(1,0,0));

	query_tfms[1] = btTransform::getIdentity();
	query_tfms[1].setOrigin(btVector3(0.5,0.5,0));

	vector<btTransform> t_tfms = lfdrpm.transform_frames(query_tfms);
	cout <<" transformed frame1: "<< util::toRaveTransform(t_tfms[0])<<endl;
	cout <<" transformed frame2: "<< util::toRaveTransform(t_tfms[1])<<endl;

	return 0;
}
