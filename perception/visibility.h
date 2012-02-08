#pragma once
#include <Eigen/Dense>
#include <vector>
#include "rope.h"
#include "plotting.h"
#include "bullet_typedefs.h"

std::vector<float> calcVisibility(const Eigen::MatrixXf& pts, const Eigen::MatrixXf& depth, const cv::Mat& ropeMask);
std::vector<float> calcVisibility(const vector<RigidBodyPtr> bodies, btDynamicsWorld* world, const btVector3& cameraPos, float stdev=0, int nSamples=1);
std::vector<float> calcVisibility(btSoftBody* softBody, btDynamicsWorld* world, const btVector3& cameraPos);
void colorByVisibility(CapsuleRope::Ptr rope, const vector<float>& pVis);
void colorByVisibility(btSoftBody* psb, const vector<float>& pVis, PlotPoints::Ptr plot);
