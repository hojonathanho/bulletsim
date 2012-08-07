#pragma once
#include <iostream>
#include <Eigen/Dense>

using namespace std;

#ifndef TEST_FUNC
#define TEST_FUNC(func)\
  cout << "function: " << #func << endl;		       \
  func();						       \
  cout << "success!" << endl
#endif

#ifndef MAT_DIMS
#define MAT_DIMS(mat)\
 	cout << "dimension " << #mat << " " << mat.rows() << " " << mat.cols() << endl;
#endif

#ifndef PF
#define PF(exp)\
	StartClock(); \
	exp; \
	cout << "PF " << GetClock() << "\t" << #exp << endl;
#endif

bool isApproxEq(float x, float y);

bool isApproxEq(Eigen::MatrixXf m0, Eigen::MatrixXf m1);

void infinityDebug(const Eigen::MatrixXf& mat, const char* mat_name = "mat");
