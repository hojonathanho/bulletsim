#pragma once
#include <Eigen/Dense>
#include <vector>
#include "simulation/rope.h"
#include "simulation/plotting.h"
#include "simulation/bullet_typedefs.h"
#include <opencv2/core/core.hpp>

std::vector<float> calcVisibility(const Eigen::MatrixXf& pts, const Eigen::MatrixXf& depth, const cv::Mat& ropeMask);
std::vector<float> calcVisibility(const vector<RigidBodyPtr> bodies, btDynamicsWorld* world, const btVector3& cameraPos, float stdev=0, int nSamples=1);
std::vector<float> calcVisibility(btSoftBody* softBody, btDynamicsWorld* world, const btVector3& cameraPos);
void colorByVisibility(CapsuleRope::Ptr rope, const vector<float>& pVis);
void colorByVisibility(btSoftBody* psb, const vector<float>& pVis, PlotPoints::Ptr plot);
