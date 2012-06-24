#pragma once
#include "tracked_object.h"
#include "clouds/pcl_typedefs.h"
#include "visibility.h"
#include "plotting_tracking.h"

class SimplePhysicsTracker {
public:
  Environment::Ptr m_env;
  TrackedObject::Ptr m_obj;
  VisibilityInterface* m_visInt;
  Eigen::MatrixXf M_estPts;
  Eigen::MatrixXf M_obsPts;
  Eigen::MatrixXf M_stdev;
  Eigen::MatrixXf M_obsDebug;

  PointCloudPlot::Ptr m_obsInlierPlot;
  PointCloudPlot::Ptr m_obsPlot;
  PointCloudPlot::Ptr m_obsTransPlot;
  PlotSpheres::Ptr m_estPlot;
  PlotSpheres::Ptr m_estTransPlot;
  PlotLines::Ptr m_corrPlot;
  PointCloudPlot::Ptr m_debugPlot;
  bool m_enableObsInlierPlot, m_enableObsPlot, m_enableObsTransPlot, m_enableEstPlot, m_enableEstTransPlot, m_enableCorrPlot, m_enableDebugPlot;

  SimplePhysicsTracker(TrackedObject::Ptr, VisibilityInterface*, Environment::Ptr);
  void updateInput(ColorCloudPtr filteredCloud);
  void doIteration();
};
