#include "geom.h"
#include <boost/foreach.hpp>
#include <iostream>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//using namespace Wm5;
using namespace std;
using namespace Eigen;

void perpBasis(const Vector3f& v1, Vector3f& v2, Vector3f& v3) {
	v2 = Vector3f(-v1[1], v1[0], 0);
	if (v2.norm() == 0)
		v2 = Vector3f(1, 0, 0);
	else
		v2.normalize();
	v3 = v1.cross(v2);
}

void minEncRect(const vector<Vector3f>& pts3d, const Vector4f& abcd, vector<Vector3f>& verts3d) {
	// ax+by+cz + d = 0
	// project points to 2d

	Vector3f abc(abcd[0], abcd[1], abcd[2]);
	Vector3f normal = abc / abc.norm();
	float q = abcd[3] / abc.norm();
	//normal . (x,y,z) + q = 0

	Vector3f va, vb;
	perpBasis(normal, va, vb);
	int nPts = pts3d.size();

	vector<cv::Point2f> pts2d;

	float MULTIPLIER = 10000; // becacuse cv assumes everything's an int
	for (int i = 0; i < nPts; ++i)
		pts2d.push_back(MULTIPLIER * cv::Point2f(pts3d[i].dot(va),
				pts3d[i].dot(vb)));

	cv::RotatedRect rect = cv::minAreaRect(pts2d);
	cv::Point2f verts2d[4];
	rect.points(verts2d);

	verts3d.clear();
	for (int i = 0; i < 4; i++) {
		verts3d.push_back(  (va * verts2d[i].x + vb * verts2d[i].y)/MULTIPLIER - normal * q  );
	}

}

float angBetween(const Vector3f& v1, const Vector3f& v2) {
	return acos(v1.dot(v2));
}


void minRot(const btVector3& v1, const btVector3& v2, btMatrix3x3& m) {
	btScalar ang = v1.angle(v2);
	btVector3 ax = v2.cross(v1);
	m = btMatrix3x3(btQuaternion(ax, ang));
}

void minRot(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2,
		Eigen::Matrix3f& m) {
	float ang = angBetween(v1, v2);
	Eigen::Vector3f ax = v2.cross(v1);
	m = Eigen::AngleAxisf(ang, ax).toRotationMatrix();
}

std::vector<Eigen::Vector3f> getCorners(const std::vector<Eigen::Vector3f>& pts) {
	std::vector<Eigen::Vector3f> out;
	const Eigen::Vector4f abcd(0, 0, 1, -pts[0][2]);
	cout << "warning: assuming all points have same z" << endl;
	minEncRect(pts, abcd, out);
	return out;
}

