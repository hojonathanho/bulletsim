#include "stochastic_tracker.h"

float StochasticPhysicsTracker::sample() {
	switch (m_dist) {
		case UNIFORM:
			return (2.0*rand()/RAND_MAX - 1.0)*m_param;
			break;
		case NORMAL: {
			float n = m_generator();
			return n;
			break;
		}
		default:
			std::runtime_error("Tried to sample from an unknown distribution.");
			break;
	}
	return 0.0;
}

bool StochasticPhysicsTracker::stochasticMoveByIK(PR2Object::ManipId manip_id) {
	for (int i=0; i<m_max_trials; i++) {
		if (m_pr2->moveByIK(manip_id, sample(), sample(), sample()))
			return true;
	}
	return false;
}
void StochasticPhysicsTracker::stochasticChangeDOFValue(PR2Object::ManipId manip_id, string arm_joint_name) {
	m_pr2->changeDOFValue(manip_id, arm_joint_name, sample());
}
void StochasticPhysicsTracker::enablePreGrabAndReleaseCallback() {
	if (m_cb_enabled) ROS_WARN("Pre grab and release callback were already enabled.");
	m_cb_enabled = true;
	vector<string> stochastic_joints_names = boost::assign::list_of("elbow_flex_joint")("forearm_roll_joint")("wrist_flex_joint")("wrist_roll_joint");
	BOOST_FOREACH(string arm_joint_name, stochastic_joints_names) {
		PR2Object::ManipCallbackPtr pre_grab_release_cb(new PR2Object::ManipCallback(boost::bind(&StochasticPhysicsTracker::stochasticChangeDOFValue, this, _1, arm_joint_name)));
		m_callbacks.push_back(pre_grab_release_cb);
		m_pr2->addPreGrabCallback(pre_grab_release_cb);
		m_pr2->addPreReleaseCallback(pre_grab_release_cb);
	}

//	m_pr2->addPreGrabCallback(boost::bind(&StochasticPhysicsTracker::stochasticMoveByIK, this, _1));
//	m_pr2->addPreReleaseCallback(boost::bind(&StochasticPhysicsTracker::stochasticMoveByIK, this, _1));
}

StochasticPhysicsTracker::StochasticPhysicsTracker(TrackedObjectFeatureExtractor::Ptr object_features, FeatureExtractor::Ptr observation_features,
		VisibilityInterface::Ptr visibility_interface, DistributionType dist, float param, bool cb_enable)
	: PhysicsTracker(object_features, observation_features, visibility_interface)
	, m_env(object_features->m_obj->m_sim->getEnvironment())
	, m_dist(dist)
	, m_param(param)
	, m_max_trials(10)
	, m_generator(boost::mt19937(time(NULL)+rand()), boost::normal_distribution<>(0, m_param))
	, m_cb_enable(cb_enable)
	, m_cb_enabled(false)
{
	for (Environment::ObjectList::iterator obj_it = m_env->objects.begin(); obj_it!= m_env->objects.end(); ++obj_it) {
		m_pr2 = boost::dynamic_pointer_cast<PR2Object>(*obj_it);
		if (m_pr2) break;
	}
	if (!m_pr2) std:runtime_error("The tracked object should be in an Environment with a PR2Object");

	if (m_cb_enable) enablePreGrabAndReleaseCallback();
}

StochasticPhysicsTracker::StochasticPhysicsTracker(TrackedObjectFeatureExtractor::Ptr object_features, StochasticPhysicsTracker::Ptr physics_tracker)
	: PhysicsTracker(object_features, physics_tracker)
	, m_env(object_features->m_obj->m_sim->getEnvironment())
	, m_dist(physics_tracker->m_dist)
	, m_param(physics_tracker->m_param)
	, m_max_trials(physics_tracker->m_max_trials)
	, m_generator(boost::mt19937(time(NULL)+rand()), boost::normal_distribution<>(0, m_param))
	, m_cb_enable(physics_tracker->m_cb_enable)
	, m_cb_enabled(false)
{
	for (Environment::ObjectList::iterator obj_it = m_env->objects.begin(); obj_it!= m_env->objects.end(); ++obj_it) {
		m_pr2 = boost::dynamic_pointer_cast<PR2Object>(*obj_it);
		if (m_pr2) break;
	}
	if (!m_pr2) std:runtime_error("The tracked object should be in an Environment with a PR2Object");

	if (m_cb_enable) enablePreGrabAndReleaseCallback();
}

StochasticPhysicsTracker::Ptr StochasticPhysicsTracker::split() {
	Fork::Ptr fork(new Fork(m_env)); // clone environment
	EnvironmentObject::Ptr sim = fork->forkOf(m_objFeatures->m_obj->m_sim);
	PR2Object::Ptr pr2 = boost::dynamic_pointer_cast<PR2Object>(fork->forkOf(m_pr2));
	osg::Vec4f color((float)rand()/RAND_MAX, (float)rand()/RAND_MAX, (float)rand()/RAND_MAX, 0.4);
	sim->setColor(color);
	pr2->setColor(color);
	TrackedObject::Ptr trackedObj = createTrackedObject(sim);
	TrackedObjectFeatureExtractor::Ptr objectFeatures(new TrackedObjectFeatureExtractor(trackedObj));
	StochasticPhysicsTracker::Ptr new_tracker(new StochasticPhysicsTracker(objectFeatures, shared_from_this()));
	return new_tracker;
}

StochasticPhysicsTracker::~StochasticPhysicsTracker() {
	BOOST_FOREACH(PR2Object::ManipCallbackPtr& cb, m_callbacks) {
		m_pr2->removePreGrabCallback(cb);
		m_pr2->removePreReleaseCallback(cb);
	}
}
