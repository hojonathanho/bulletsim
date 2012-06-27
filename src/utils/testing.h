#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>

using namespace std;
using namespace Eigen;

#define TEST_FUNC(func)\
  cout << "function: " << #func << endl;		       \
  func();						       \
  cout << "success!" << endl

#define MAT_DIMS(mat)\
 	cout << "dimension " << #mat << " " << mat.rows() << " " << mat.cols() << endl;

bool isApproxEq(float x, float y) {
	const double EPSILON = 1E-4;
	if (x == 0) return fabs(y) <= EPSILON;
	if (y == 0) return fabs(x) <= EPSILON;
	return fabs(x - y) / max(fabs(x), fabs(y)) <= EPSILON;
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
