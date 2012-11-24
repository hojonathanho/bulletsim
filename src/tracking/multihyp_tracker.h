#pragma once
#include "physics_tracker.h"
#include "robots/grab_detection.h"

class MultiHypTracker : public PhysicsTracker {
public:
	typedef boost::shared_ptr<MultiHypTracker> Ptr;
  std::vector<PhysicsTracker::Ptr> m_trackers;
  std::vector<GrabManager::Ptr> m_grab_managers;
  std::vector<bool> m_grabbing_last;

  MultiHypTracker(TrackedObjectFeatureExtractor::Ptr object_features, FeatureExtractor::Ptr observation_features,
  		VisibilityInterface::Ptr visibility_interface, std::vector<GrabManager::Ptr> grab_managers);
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
