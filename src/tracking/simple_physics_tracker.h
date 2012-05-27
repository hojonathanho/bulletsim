#pragma once
#include "tracked_object.h"
#include "clouds/utils_pcl.h"
#include "visibility.h"

class SimplePhysicsTracker {
public:
  Environment::Ptr m_env;
  TrackedObject::Ptr m_obj;
  VisibilityInterface* m_visInt;
  Eigen::MatrixXf m_obsPts;
  Eigen::MatrixXf m_estPts;
  Eigen::VectorXf m_stdev;
  
  SimplePhysicsTracker(TrackedObject::Ptr, VisibilityInterface*);
  void updateInput(ColorCloudPtr filteredCloud);
  void doIteration();
};
