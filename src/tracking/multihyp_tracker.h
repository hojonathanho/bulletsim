#pragma once
#include "simple_physics_tracker.h"
#include "visibility.h"

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
