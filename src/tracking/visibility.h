#pragma once
#include "utils_tracking.h"
#include "tracked_object.h"
#include <opencv2/core/core.hpp>

class VisibilityInterface {
public:
	virtual Eigen::VectorXf checkNodeVisibility(TrackedObject::Ptr) = 0;
};

class EverythingIsVisible : public VisibilityInterface {
public:
	Eigen::VectorXf checkNodeVisibility(TrackedObject::Ptr);
};

class DepthImageVisibility : public VisibilityInterface {
public:
  CoordinateTransformer* m_transformer;
  Eigen::MatrixXf m_depth;
  DepthImageVisibility(CoordinateTransformer* transformer) : m_transformer(transformer) {}  
	Eigen::VectorXf checkNodeVisibility(TrackedObject::Ptr);
	void updateInput(const cv::Mat&);
};
