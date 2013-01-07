#pragma once
#include "stochastic_tracker.h"

class MultiHypTracker : public PhysicsTracker {
public:
	typedef boost::shared_ptr<MultiHypTracker> Ptr;
  std::vector<StochasticPhysicsTracker::Ptr> m_trackers;
  int m_active_tracker_ind;
  StochasticPhysicsTracker::DistributionType m_dist;
	float m_param;
	int m_num_splits;
	int m_max_num_trackers;

  MultiHypTracker(TrackedObjectFeatureExtractor::Ptr object_features, FeatureExtractor::Ptr observation_features,
  		VisibilityInterface::Ptr visibility_interface, ObservationVisibility::Ptr observation_visibility,
  		StochasticPhysicsTracker::DistributionType dist, float param, int num_splits=2, int max_num_trackers=4);

	// the active tracker is the tracker with the highest probability of being correct
	inline StochasticPhysicsTracker::Ptr getActiveTracker() { return m_trackers[m_active_tracker_ind]; }

	// returns the state of the active tracker
	void getState(Eigen::MatrixXf& estPts, Eigen::MatrixXf& stdev, Eigen::MatrixXf& obsPts, Eigen::MatrixXf& pZgivenC, Eigen::VectorXf& vis,
  		FeatureExtractor::Ptr& obsFeatures, TrackedObjectFeatureExtractor::Ptr& objFeatures) {
		getActiveTracker()->getState(estPts, stdev, obsPts, pZgivenC, vis, obsFeatures, objFeatures);
	}

	vector<float> computeScores();
	void splitTrackers();
	void reduceTrackers();
	void updateActiveTracker();
	void preGrabCallback(PR2Object::ManipId manip_id);

  void updateFeatures();
  void expectationStep();
  void maximizationStep(bool apply_evidence=true);
};


class MultiHypTrackerVisualizer : public PhysicsTrackerVisualizer {
public:
  typedef boost::shared_ptr<MultiHypTrackerVisualizer> Ptr;
  int m_drawHypInd; // indicates which tracker's environment is active and rendered. the index refers to the ranking (by scores) of the trackers, i.e. an an index of 0 refers to the tracker with the highest score
  bool m_displayVizInfo;

  MultiHypTrackerVisualizer(Scene* scene, PhysicsTracker::Ptr tracker);
  void update();
  inline MultiHypTracker::Ptr getTracker() { return boost::dynamic_pointer_cast<MultiHypTracker>(m_tracker); }
};
