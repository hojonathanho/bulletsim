#include "testing.h"
#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include <math.h>

using namespace std;
using namespace Eigen;

bool isApproxEq(float x, float y) {
	const double EPSILON = 1E-4;
	if (x == 0) return fabs(y) <= EPSILON;
	if (y == 0) return fabs(x) <= EPSILON;
	return (fabs(x - y) / max(fabs(x), fabs(y)) <= EPSILON) || (isnan(x) && isnan(y));
}

bool isApproxEq(MatrixXf m0, MatrixXf m1) {
	assert(m0.rows() == m1.rows());
	assert(m0.cols() == m1.cols());
	int error_cnt = 0;
	int error_i = 0;
	int error_j = 0;
	for (int i=0; i<m0.rows(); i++)
		for (int j=0; j<m0.cols(); j++)
			if (!isApproxEq(m0(i,j), m1(i,j))) {
				error_cnt++;
				error_i = i;
				error_j = j;
			}
	cout << "errors: " << error_cnt << " " << error_i << " " << error_j << " " << m0(error_i, error_j) << " " << m1(error_i, error_j) << endl;
	return (error_cnt == 0);
}

bool isApproxEq(const Affine3f& t0, const Affine3f& t1) {
	for (int i=0; i<16; i++) {
		if (!isApproxEq(t0.data()[i], t1.data()[i])) return false;
	}
	return true;
}

bool isApproxEq(const btVector3& v0, const btVector3& v1) {
	for (int i=0; i<3; i++)
		if (!isApproxEq(v0.m_floats[i], v1.m_floats[i])) return false;
	return true;
}

bool isApproxEq(const btMatrix3x3& m0, const btMatrix3x3& m1) {
	for (int i=0; i<3; i++)
		if (!isApproxEq(m0[i], m1[i])) return false;
	return true;
}

bool isApproxEq(const btTransform& t0, const btTransform& t1) {
	float t0_m[16];
	float t1_m[16];
	t0.getOpenGLMatrix(t0_m);
	t1.getOpenGLMatrix(t1_m);
	for (int i=0; i<16; i++) {
		if (!isApproxEq(t0_m[i], t1_m[i])) return false;
	}
	return true;
}

void infinityDebug(const MatrixXf& mat, const char* mat_name) {
	int r=0;
	for (int row=0; row < mat.rows(); row++) {
		int c=0;
		int first_col=-1;
		for (int col=0; col<mat.cols(); col++) {
			if (!isfinite(mat(row,col))) {
				if (first_col<0) first_col = col;
				c++;
				cout << col << " ";
			}
		}
		if (c>0) {
			cout << endl;
			printf("%s: %d/%d infinite cols \t%s(%d,%d)=%f\n", mat_name, c, (int) mat.cols(), mat_name, row, first_col, mat(row,first_col));
			r++;
		}
	}
	if (r>0) printf("%s: %d/%d infinite rows\n", mat_name, r, (int) mat.rows());
}
