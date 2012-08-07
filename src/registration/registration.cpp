#include "registration.h"
#include "pcl/registration/registration.h"
#include "pcl/registration/icp.h"

void alignClouds(const CloudPtr& src, const CloudPtr& targ, Eigen::Matrix4f& transformation, float& score) {
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> reg;
	reg.setInputCloud(src);
	reg.setInputTarget(targ);
	reg.setMaximumIterations (50);
	reg.setTransformationEpsilon (1e-8);
	reg.setMaxCorrespondenceDistance (0.05);

	CloudPtr cloud_reg(new Cloud());
	  // Register
	reg.align (*cloud_reg);

	transformation = reg.getFinalTransformation();
	score = reg.getFitnessScore();

}


