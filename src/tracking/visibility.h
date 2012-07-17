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
  cv::Mat m_depth;
  DepthImageVisibility(CoordinateTransformer* transformer) : m_transformer(transformer) {}  
	Eigen::VectorXf checkNodeVisibility(TrackedObject::Ptr);
	void updateInput(const cv::Mat&);
};

class OSGVisibility : public VisibilityInterface {
public:
  CoordinateTransformer* m_transformer;
  OSGVisibility(CoordinateTransformer* transformer) : m_transformer(transformer) {}
  Eigen::VectorXf checkNodeVisibility(TrackedObject::Ptr);
  vector<btVector3> getIntersectionPoints(TrackedObject::Ptr);
	void updateInput(const cv::Mat&) {};
};

class BulletVisibility : public VisibilityInterface {
public:
  CoordinateTransformer* m_transformer;
  BulletVisibility(CoordinateTransformer* transformer) : m_transformer(transformer) {}
  Eigen::VectorXf checkNodeVisibility(TrackedObject::Ptr);
	Eigen::VectorXf calcVisibility(const vector<btVector3> nodes, btDynamicsWorld* world, const btVector3& cameraPos);
	void updateInput(const cv::Mat&) {};
};

class MultiVisibility : public VisibilityInterface {
public:
	vector<VisibilityInterface*> visibilities;
	void addVisibility(VisibilityInterface* visibility) { visibilities.push_back(visibility); }
  Eigen::VectorXf checkNodeVisibility(TrackedObject::Ptr);
};
