#pragma once
#include "physics_tracker.h"
#include "robots/PR2Object.h"
#include <time.h>

class StochasticPhysicsTracker : public PhysicsTracker {
public:
	typedef boost::shared_ptr<StochasticPhysicsTracker> Ptr;
	Environment::Ptr m_env;
	PR2Object::Ptr m_pr2;

	enum Distribution { UNIFORM, NORMAL, NONE };
	Distribution m_dist;
	float m_param;

	float sample() {
		switch (m_dist) {
			case UNIFORM:
				return (2.0*rand()/RAND_MAX - 1.0)*m_param;
				break;
			default:
				std::runtime_error("Tried to sample from an unknown distribution.");
				break;
		}
		return 0.0;
	}

	StochasticPhysicsTracker(TrackedObjectFeatureExtractor::Ptr object_features, FeatureExtractor::Ptr observation_features, VisibilityInterface::Ptr visibility_interface, Distribution dist, float param)
		: PhysicsTracker(object_features, observation_features, visibility_interface)
		, m_env(object_features->m_obj->m_sim->getEnvironment())
		, m_dist(dist)
		, m_param(param)
	{
		for (Environment::ObjectList::iterator obj_it = m_env->objects.begin(); obj_it!= m_env->objects.end(); ++obj_it) {
			m_pr2 = boost::dynamic_pointer_cast<PR2Object>(*obj_it);
			if (m_pr2) break;
		}
		if (!m_pr2) std:runtime_error("The tracked object should be in an Environment with a PR2Object");

	  srand ( time(NULL) );
	  m_pr2->addPreGrabCallback(boost::bind(&PR2Object::moveByIK, m_pr2.get(), _1, sample(), sample(), sample()));
	  m_pr2->addPreReleaseCallback(boost::bind(&PR2Object::moveByIK, m_pr2.get(), _1, sample(), sample(), sample()));
	}
};

class MultiHypTracker : public PhysicsTracker {
public:
	typedef boost::shared_ptr<MultiHypTracker> Ptr;
  std::vector<StochasticPhysicsTracker::Ptr> m_trackers;
  StochasticPhysicsTracker::Distribution m_dist;
	float m_param;

  MultiHypTracker(TrackedObjectFeatureExtractor::Ptr object_features, FeatureExtractor::Ptr observation_features,
  		VisibilityInterface::Ptr visibility_interface, StochasticPhysicsTracker::Distribution dist, float param);
  void updateFeatures();
  void expectationStep();
  void maximizationStep(bool apply_evidence=true);
};

/*
class MultiHypTracker {
public:
  std::vector<SimplePhysicsTracker::Ptr> m_trackers;
  std::vector< std::vector<float> > m_costs;
  
  void doIteration();
  void updateInput(ColorCloudPtr obsPts);
  void calcCosts();
  
};

VisibilityInterface::Ptr makeVisibilityWithNewWorld(VisibilityInterface::Ptr, btDynamicsWorld*);
SimplePhysicsTracker::Ptr makeTrackerWithNewObject(SimplePhysicsTracker::Ptr, TrackedObject::Ptr);
*/
