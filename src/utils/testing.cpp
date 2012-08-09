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

void infinityDebug(const MatrixXf& mat, const char* mat_name = "mat") {
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
