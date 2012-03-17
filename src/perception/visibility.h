#pragma once
#include <Eigen/Dense>
#include <vector>
#include "simulation/rope.h"
#include "simulation/plotting.h"
#include "simulation/bullet_typedefs.h"
#include <opencv2/core/core.hpp>
#include "plotting_perception.h"

Eigen::VectorXf calcVisibility(const Eigen::MatrixXf& pts, const Eigen::MatrixXf& depth, const cv::Mat& ropeMask);
Eigen::VectorXf calcVisibility(const vector<RigidBodyPtr> bodies, btDynamicsWorld* world, const btVector3& cameraPos, float stdev=0, int nSamples=1);
Eigen::VectorXf calcVisibility(btSoftBody* softBody, btDynamicsWorld* world, const btVector3& cameraPos, const std::vector<int>& nodeInds);
Eigen::VectorXf calcVisibility(btSoftBody* softBody, btDynamicsWorld* world, const btVector3& cameraPos);
void colorByVisibility(CapsuleRope::Ptr rope, const Eigen::VectorXf& pVis);
void colorByVisibility(btSoftBody* psb, const Eigen::VectorXf& pVis, PointCloudPlot::Ptr plot);
