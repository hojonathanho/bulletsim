#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btTransform.h>

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
bool isApproxEq(const Eigen::Affine3f& t0, const Eigen::Affine3f& t1);

bool isApproxEq(const btVector3& v0, const btVector3& v1);
bool isApproxEq(const btMatrix3x3& m0, const btMatrix3x3& m1);
bool isApproxEq(const btTransform& t0, const btTransform& t1);

void infinityDebug(const Eigen::MatrixXf& mat, const char* mat_name = "mat");
