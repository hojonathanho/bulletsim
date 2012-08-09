#pragma once
#include "utils_tracking.h"
#include "tracked_object.h"
#include <opencv2/core/core.hpp>

class VisibilityInterface {
public:
  typedef boost::shared_ptr<VisibilityInterface> Ptr;
	virtual Eigen::VectorXf checkNodeVisibility(TrackedObject::Ptr) = 0;
	virtual void updateInput(const cv::Mat&) {}
};

class EverythingIsVisible : public VisibilityInterface {
public:
	typedef boost::shared_ptr<EverythingIsVisible> Ptr;
	Eigen::VectorXf checkNodeVisibility(TrackedObject::Ptr obj);
};

class DepthImageVisibility : public VisibilityInterface {
public:
  typedef boost::shared_ptr<DepthImageVisibility> Ptr;
	CoordinateTransformer* m_transformer;
	cv::Mat m_depth;
	DepthImageVisibility(CoordinateTransformer* transformer) : m_transformer(transformer) {}  
	Eigen::VectorXf checkNodeVisibility(TrackedObject::Ptr obj);
	void updateInput(const cv::Mat&);
};

class OSGVisibility : public VisibilityInterface {
public:
	CoordinateTransformer* m_transformer;
	OSGVisibility(CoordinateTransformer* transformer) : m_transformer(transformer) {}
	Eigen::VectorXf checkNodeVisibility(TrackedObject::Ptr obj);
	vector<btVector3> getIntersectionPoints(TrackedObject::Ptr obj);
};

class BulletRaycastVisibility : public VisibilityInterface {
public:
  typedef boost::shared_ptr<BulletRaycastVisibility> Ptr;
	btDynamicsWorld* m_world;
	CoordinateTransformer* m_transformer;
	BulletRaycastVisibility(btDynamicsWorld* world, CoordinateTransformer* transformer);
	Eigen::VectorXf checkNodeVisibility(TrackedObject::Ptr obj);
};

//A node is visible if it is visible by DepthImageVisibility and BulletRaycastVisibility
class AllOcclusionsVisibility : public VisibilityInterface {
public:
  typedef boost::shared_ptr<AllOcclusionsVisibility> Ptr;
  
	DepthImageVisibility::Ptr m_depth_image_visibility;
	BulletRaycastVisibility::Ptr m_bullet_raycast_visibility;
	AllOcclusionsVisibility(btDynamicsWorld* world, CoordinateTransformer* transformer) :
		m_depth_image_visibility(new DepthImageVisibility(transformer)),
		m_bullet_raycast_visibility(new BulletRaycastVisibility(world, transformer))
	{}
	Eigen::VectorXf checkNodeVisibility(TrackedObject::Ptr obj) {
		return m_depth_image_visibility->checkNodeVisibility(obj).cwiseMin(m_bullet_raycast_visibility->checkNodeVisibility(obj));
	}
	void updateInput(const cv::Mat& in) {
		m_depth_image_visibility->updateInput(in);
	}
};

class MultiVisibility : public VisibilityInterface {
public:
  typedef boost::shared_ptr<MultiVisibility> Ptr;
  
	vector<VisibilityInterface::Ptr> visibilities;
	void addVisibility(VisibilityInterface::Ptr visibility) { visibilities.push_back(visibility); }
	Eigen::VectorXf checkNodeVisibility(TrackedObject::Ptr obj) {
		Eigen::VectorXf vis = Eigen::VectorXf::Zero(obj->m_nNodes);
		for (int i=0; i<visibilities.size(); i++)
			vis = vis.cwiseMax(visibilities[i]->checkNodeVisibility(obj));
		return vis;
	}
};
