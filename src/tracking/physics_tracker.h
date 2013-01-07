#pragma once
#include "tracked_object.h"
#include "sparse_utils.h"
#include "clouds/pcl_typedefs.h"
#include "visibility.h"
#include "plotting_tracking.h"
#include "simulation/simplescene.h"
#include "feature_extractor.h"
#include <cv.h>

class PhysicsTracker {
public:
	typedef boost::shared_ptr<PhysicsTracker> Ptr;

protected:
	TrackedObjectFeatureExtractor::Ptr m_objFeatures;
  FeatureExtractor::Ptr m_obsFeatures;

  VisibilityInterface::Ptr m_visInt;
  ObservationVisibility::Ptr m_obsVis; // empty if the free space model is not used

  // latent parameters
  Eigen::MatrixXf m_estPts;
  Eigen::MatrixXf m_stdev;
  Eigen::VectorXf m_priorDist;

  // observed variables
  Eigen::MatrixXf m_obsPts;
public:
  Eigen::MatrixXf m_noObsPts;
  ColorCloud::Ptr noObsCloud;

  Eigen::MatrixXf m_pZgivenNoC;
protected:

  // posterior probabilities
  Eigen::MatrixXf m_pZgivenC;

  // outliers and unmodelled occluders
  Eigen::VectorXf m_outlierDist; //m_obsPts - m_estPts for the fake node responsible for an outlier observation. same for all obsPts.
  Eigen::VectorXf m_vis;

public:
  PhysicsTracker(TrackedObjectFeatureExtractor::Ptr object_features, FeatureExtractor::Ptr observation_features,
  		VisibilityInterface::Ptr visibility_interface, ObservationVisibility::Ptr observation_visibility=ObservationVisibility::Ptr());

  PhysicsTracker(TrackedObjectFeatureExtractor::Ptr object_features, PhysicsTracker::Ptr physics_tracker) :
  	m_objFeatures(object_features),
  	m_obsFeatures(physics_tracker->m_obsFeatures),
  	m_visInt(physics_tracker->m_visInt),
  	m_estPts(physics_tracker->m_estPts),
  	m_stdev(physics_tracker->m_stdev),
		m_priorDist(physics_tracker->m_priorDist),
		m_obsPts(physics_tracker->m_obsPts),
		m_pZgivenC(physics_tracker->m_pZgivenC),
		m_outlierDist(physics_tracker->m_outlierDist),
		m_vis(physics_tracker->m_vis)
  {}

  virtual ~PhysicsTracker() {}

  virtual void getState(Eigen::MatrixXf& estPts, Eigen::MatrixXf& stdev, Eigen::MatrixXf& obsPts, Eigen::MatrixXf& pZgivenC, Eigen::VectorXf& vis,
  		FeatureExtractor::Ptr& obsFeatures, TrackedObjectFeatureExtractor::Ptr& objFeatures) {
		estPts = m_estPts;
		stdev = m_stdev;
		obsPts = m_obsPts;
		pZgivenC = m_pZgivenC;
		vis = m_vis;
		obsFeatures = m_obsFeatures;
		objFeatures = m_objFeatures;
  }

  virtual void updateFeatures();
  virtual void expectationStep();
  virtual void expectationStepExtended();
  virtual void maximizationStep(bool apply_evidence=true);
};


class PhysicsTrackerVisualizer {
public:

  typedef boost::shared_ptr<PhysicsTrackerVisualizer> Ptr;

  Scene* m_scene;
  PhysicsTracker::Ptr m_tracker;

  PhysicsTrackerVisualizer(Scene* scene, PhysicsTracker::Ptr tracker);
  virtual ~PhysicsTrackerVisualizer() {}
  virtual void update();

  PointCloudPlot::Ptr m_obsInlierPlot;
  PointCloudPlot::Ptr m_obsPlot;
  PointCloudPlot::Ptr m_obsTransPlot;
  PointCloudPlot::Ptr m_noObsPlot;
  PlotSpheres::Ptr m_estPlot;
  PlotSpheres::Ptr m_estTransPlot;
  PlotSpheres::Ptr m_estCalcPlot;
  PlotSpheres::Ptr m_visPlot;
  PlotLines::Ptr m_corrPlot;
  PlotLines::Ptr m_hiddenNodeCorrPlot;
  bool m_enableObsInlierPlot, m_enableObsPlot, m_enableObsTransPlot, m_enableNoObsPlot, m_enableEstPlot, m_enableEstTransPlot, m_enableEstCalcPlot,
  m_enableVisPlot, m_enableCorrPlot, m_enableHiddenCorrPlot;
  int m_nodeCorrPlot;
};
