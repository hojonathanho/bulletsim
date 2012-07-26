#pragma once
#include "clouds/utils_pcl.h"
using namespace Eigen;

void alignClouds(const CloudPtr& src, const CloudPtr& targ, Eigen::Matrix4f& transformation, float& score);
void alignCloudsMultiInit(const CloudPtr& src, const CloudPtr& targ, Eigen::Matrix4f& transformation, float& score);

void tps_rpm(std::vector<ColorCloudPtr> srcs, std::vector<ColorCloudPtr> targs);
void tps_rpm_multi

class ThinPlateSpline {
  ThinPlateSpline();
  ThinPlateSpline(const ThinPlateSpline&);
  static ThinPlateSpline Identity(const int d);
  void fit(const MatrixXf& x_nd, const MatrixXf& y_nd, const float nonaff_cost, const float rot_cost);
  void transformPoints(const MatrixXf& x_md);
  void transformFrames(const MatrixXf& x_md, const MatriXf& rot_md, const bool orthogonalize, const MatrixXf& ypred_md, const MatrixXf& newrot_mdd);
};

static ThinPlateSpline ThinPlateSpline::Identity(int d) {
    ThinPlateSpline tps;
    tps.n = 0;
    tps.d = d;
    tps.x_nd = Eigen::Zero(0,d);
    tps.w_nd = Eigen::Zero(0,d);
    tps.a_Dd = Eigen::Zero(d+1,d);
    tps.topLeftCorner(d,d).setIdentity();
}

void sinkhornFactorize(const MatrixXf& P, float outlierParam, MatrixXf& Q, VectorXf& D_left, VectorXf& D_right);
void calcCorrespondenceSinkhorn(const MatrixXf& x_nd, const MatrixXf& y_md, float radius, float outlierParam, float epsilon, float maxIter);
Matrix3f orthogonalize(const Matrix3f&);
MatrixXf tpsKernel(const Eigen::MatrixXf& A);
