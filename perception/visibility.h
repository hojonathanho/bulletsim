#pragma once
#include <Eigen/Dense>
#include <vector>
#include "rope.h"

std::vector<float> calcVisibility(const Eigen::MatrixXf& pts, const Eigen::MatrixXf& depth, const cv::Mat& ropeMask);
std::vector<float> calcVisibility(const vector<RigidBodyPtr> bodies, btDynamicsWorld* world, const btVector3& cameraPos, float stdev, int nSamples);
void colorByVisibility(CapsuleRope::Ptr rope, const vector<float>& pVis);
