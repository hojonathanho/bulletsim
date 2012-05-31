#pragma once
#include "tracked_object.h"
#include "clouds/utils_pcl.h"
#include "visibility.h"
#include "plotting_tracking.h"


class SimplePhysicsTracker {
public:
  Environment::Ptr m_env;
  TrackedObject::Ptr m_obj;
  VisibilityInterface* m_visInt;
  vector<btVector3> m_obsPts;
  vector<btVector3> m_estPts;
  Eigen::VectorXf m_stdev;
  
  PointCloudPlot::Ptr m_obsPlot;
  PlotSpheres::Ptr m_estPlot;
  PlotLines::Ptr m_corrPlot;
  bool m_enableObsPlot, m_enableEstPlot, m_enableCorrPlot;

  SimplePhysicsTracker(TrackedObject::Ptr, VisibilityInterface*, Environment::Ptr);
  void updateInput(ColorCloudPtr filteredCloud);
  void doIteration();
};
