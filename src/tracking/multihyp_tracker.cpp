#include "multihyp_tracker.h"

vector<float> MultiHypTracker::computeScores() {
	vector<float> scores;
	for (int i=0; i<m_trackers.size(); i++) {
		Eigen::VectorXf alpha_N = m_trackers[i]->m_pZgivenC.colwise().sum();
		scores.push_back(alpha_N.mean());
	}
	return scores;
}

void MultiHypTracker::splitTrackers() {
	for (int iTracker=(m_trackers.size()-1); iTracker>=0; iTracker--) {
		StochasticPhysicsTracker::Ptr tracker = m_trackers[iTracker];
		for (int i=0; i<m_num_splits; i++) {
			StochasticPhysicsTracker::Ptr new_tracker = tracker->split();
			m_trackers.push_back(new_tracker);
		}
	}
}

// removes trackers whose estimates are bad or relatively bad
void MultiHypTracker::reduceTrackers() {
	// remove trackers with bad estimates
	for (int i=m_trackers.size()-1; i>=0; i--) {
		// remove tracker if
		// (1) its points are not finite or
		// (2) its score is less than 0.5 and there is more than 1 tracker
		if (!m_trackers[i]->m_objFeatures->m_obj->isConsistent() || (m_trackers.size()>1 && computeScores()[i]<0.5)) {
			m_trackers.erase(m_trackers.begin()+i);
		}
	}
	// remove worst tracker if there are too many
	while (m_trackers.size() > m_max_num_trackers) {
		m_trackers.erase(m_trackers.begin() + argMin(computeScores()));
	}
}

// updates active tracker ind to the tracker's ind with highest metric
// the active tracker is m_trackers[ind], where the estimate of the multi hypothesis tracker is the estimate of the active tracker.
void MultiHypTracker::updateActiveTracker() {
	int ind = argMax(computeScores());
	assert(ind>=0 && ind<m_trackers.size());
	m_active_tracker_ind = ind;
}

void MultiHypTracker::preGrabCallback(PR2Object::ManipId manip_id) {
	splitTrackers();
	vector<string> stochastic_joints_names = boost::assign::list_of("elbow_flex_joint")("forearm_roll_joint")("wrist_flex_joint")("wrist_roll_joint");
	for (int i=0; i<m_trackers.size(); i++) {
		BOOST_FOREACH(string arm_joint_name, stochastic_joints_names) {
			m_trackers[i]->stochasticChangeDOFValue(manip_id, arm_joint_name);
		}
		m_trackers[i]->m_pr2->grab(manip_id, false);
	}
}

MultiHypTracker::MultiHypTracker(TrackedObjectFeatureExtractor::Ptr object_features, FeatureExtractor::Ptr observation_features,
		VisibilityInterface::Ptr visibility_interface, ObservationVisibility::Ptr observation_visibility, StochasticPhysicsTracker::DistributionType dist, float param, int num_splits, int max_num_trackers) :
		PhysicsTracker(object_features, observation_features, visibility_interface, observation_visibility),
		m_active_tracker_ind(0),
		m_dist(dist),
		m_param(param),
		m_num_splits(num_splits),
		m_max_num_trackers(max_num_trackers)
{
	StochasticPhysicsTracker::Ptr tracker(new StochasticPhysicsTracker(object_features, observation_features, visibility_interface, dist, param, false));

	PR2Object::ManipCallbackPtr pre_grab_release_cb(new PR2Object::ManipCallback(boost::bind(&MultiHypTracker::preGrabCallback, this, _1)));
	tracker->m_pr2->addPreGrabCallback(pre_grab_release_cb);

	m_trackers.push_back(tracker);
}

void MultiHypTracker::updateFeatures() {
	for (int i=0; i<m_trackers.size(); i++)
		m_trackers[i]->updateFeatures();
}

void MultiHypTracker::expectationStep() {
	for (int i=0; i<m_trackers.size(); i++)
		m_trackers[i]->expectationStep();
}

void MultiHypTracker::maximizationStep(bool apply_evidence) {
	for (int i=0; i<m_trackers.size(); i++) {
		m_trackers[i]->maximizationStep(apply_evidence); // this function already does the simulation step
	}
	reduceTrackers();
	updateActiveTracker();
}


MultiHypTrackerVisualizer::MultiHypTrackerVisualizer(Scene* scene, PhysicsTracker::Ptr tracker) :
	PhysicsTrackerVisualizer(scene, tracker),
	m_drawHypInd(0),
	m_displayVizInfo(false)
{
	m_scene->addVoidKeyCallback(',',boost::bind(add, &m_drawHypInd, -1, 0), "decrement the index of the tracker's environment that should be active (smallest index corresponds to the tracker with highest score.");
	m_scene->addVoidKeyCallback('.',boost::bind(add, &m_drawHypInd, 1, 0), "increment the index of the tracker's environment that should be active (smallest index corresponds to the tracker with highest score.");
	m_scene->addVoidKeyCallback('/',boost::bind(toggle, &m_displayVizInfo), "toggle display tracker visualization info");
}

void MultiHypTrackerVisualizer::update() {
	PhysicsTrackerVisualizer::update();

	if (m_drawHypInd < 0)
		m_drawHypInd = 0;
	if (m_drawHypInd >= getTracker()->m_trackers.size())
		m_drawHypInd = getTracker()->m_trackers.size()-1;

	vector<float> scores(getTracker()->computeScores());
	vector<float> sorted_scores(scores); // highest to smallest
	sort(sorted_scores.begin(), sorted_scores.end(), std::greater<float>());
	int active_env_ind;
	for (active_env_ind=0; active_env_ind<scores.size(); active_env_ind++) {
		if (scores[active_env_ind] == sorted_scores[m_drawHypInd]) break;
	}
	m_scene->setEnvironment(getTracker()->m_trackers[active_env_ind]->m_env);

	if (m_displayVizInfo) {
		cout << "hypothesis scores: " << scores << endl;
		cout << "tracker ind of the active env: " << active_env_ind << endl;
		cout << "score rank of this tracker:    " << m_drawHypInd << endl;
	}
}
