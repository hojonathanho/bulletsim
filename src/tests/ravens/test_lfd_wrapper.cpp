#include <vector>
#include "lfd/LfdRpmWrapper.h"

using namespace std;
namespace py = boost::python;


/* NOTE: This test file in no way rigoursly tests lfd code; it is
 * only good for basic sanity checks.*/


/** Small test to see how to instantiate python lists. */
void np_test_wrapper(const vector<vector<btVector3> > &dat) {
	cout << "cpp : length : "<<dat.size();
	py::list lst;
	for (int i=0; i<dat.size(); i+=1) {
		py::object point_cloud = pointsToNumpy(dat[i]);
		lst.append(point_cloud);
	}
	py::object np_test = py::import("lfd.np_test").attr("p");
	np_test(lst);
}

void test_py_list() {
	int NUM_POINTS = 100;

	//	 create a sample source data set
	vector<btVector3> src(NUM_POINTS);
	for (int i=0; i< NUM_POINTS; i+=1) {
		src[i] = btVector3(i/100.0, i/100.0, 0);
	}

	//	 create a sample target data set
	vector<btVector3> target(3*NUM_POINTS/4);
	for (int i=0; i< 3*NUM_POINTS/4; i+=1) {
		target[i] = btVector3(i/100.0, i/200.0, 0);
	}

	vector<vector<btVector3> > point_clouds;
	point_clouds.push_back(src);
	point_clouds.push_back(target);
	np_test_wrapper(point_clouds);
}


void test_tps_rpm() {
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

	RegistrationModule lfdrpm(src, target,10);


	// ------------ query ---------------------
	vector<btVector3> query_pts(2);
	query_pts[0] = btVector3(0,0,0);
	query_pts[1] = btVector3(0.5,0.5,0);

	vector<btVector3> t_pts = lfdrpm.transform_points(query_pts);


	vector<btTransform> query_tfms(2);
	query_tfms[0] = btTransform::getIdentity();
	query_tfms[0].setOrigin(btVector3(1,0,0));

	query_tfms[1] = btTransform::getIdentity();
	query_tfms[1].setOrigin(btVector3(0.5,0.5,0));

	vector<btTransform> t_tfms = lfdrpm.transform_frames(query_tfms);
	cout <<"-------------------- TPS-RPM TEST ------------------------"<<endl;
	cout <<" transformed pt1: "<< t_pts[0].getX()<<","<< t_pts[0].getY()<<","<< t_pts[0].getZ()<<endl;
	cout <<" transformed pt2: "<< t_pts[1].getX()<<","<< t_pts[1].getY()<<","<< t_pts[1].getZ()<<endl;
	cout <<" transformed frame1: "<< util::toRaveTransform(t_tfms[0])<<endl;
	cout <<" transformed frame2: "<< util::toRaveTransform(t_tfms[1])<<endl;
	cout <<endl;

}


void test_tps_rpm_multi() {
	int NUM_POINTS = 100;

	//	 create a sample source data set
	vector<btVector3> src1(NUM_POINTS/2), src2(NUM_POINTS/2);
	for (int i=0; i< NUM_POINTS/2; i+=1) {
		src1[i] = btVector3(i/100.0, i/100.0, 0);
		src2[i] = btVector3((NUM_POINTS/2 + i)/100.0, (NUM_POINTS/2 + i)/100.0, 0);
	}

	//	 create a sample target data set
	vector<btVector3> target1(NUM_POINTS/2), target2(NUM_POINTS/2);

	for (int i=0; i< NUM_POINTS/2; i+=1) {
		target1[i] = btVector3(i/100.0, i/200.0, 0);
	}
	for (int i=0; i< NUM_POINTS/2; i+=1) {
		target2[i] = btVector3((i+NUM_POINTS/2)/100.0, (i+NUM_POINTS/2)/200.0, 0);
	}

	vector<vector<btVector3> > src_clouds, target_clouds;
	src_clouds.push_back(src1); src_clouds.push_back(src2);
	target_clouds.push_back(target1); target_clouds.push_back(target2);


	RegistrationModule lfdrpm(src_clouds, target_clouds, 50);

	// -------- query ------------------------
	vector<btVector3> query_pts(2);
	query_pts[0] = btVector3(0,0,0);
	query_pts[1] = btVector3(0.5,0.5,0);

	vector<btVector3> t_pts = lfdrpm.transform_points(query_pts);

	vector<btTransform> query_tfms(2);
	query_tfms[0] = btTransform::getIdentity();
	query_tfms[0].setOrigin(btVector3(1,0,0));

	query_tfms[1] = btTransform::getIdentity();
	query_tfms[1].setOrigin(btVector3(0.5,0.5,0));
	vector<btTransform> t_tfms = lfdrpm.transform_frames(query_tfms);

	cout <<"------------------ TPS-RPM-MULTI TEST --------------------"<<endl;
	cout <<" transformed pt1: "<< t_pts[0].getX()<<","<< t_pts[0].getY()<<","<< t_pts[0].getZ()<<endl;
	cout <<" transformed pt2: "<< t_pts[1].getX()<<","<< t_pts[1].getY()<<","<< t_pts[1].getZ()<<endl;
	cout <<" transformed frame1: "<< util::toRaveTransform(t_tfms[0])<<endl;
	cout <<" transformed frame2: "<< util::toRaveTransform(t_tfms[1])<<endl;
	cout <<endl;
}


int main(int argc, char* argv[]) {

	setup_python();
	test_tps_rpm();
	test_tps_rpm_multi();

	return 0;
}
