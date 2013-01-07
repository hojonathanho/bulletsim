#ifndef STOCHASTIC_TRACKER_H_
#define STOCHASTIC_TRACKER_H_

#include "physics_tracker.h"
#include "feature_extractor.h"
#include "robots/PR2Object.h"
#include "utils/utils_vector.h"
#include <time.h>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/assign.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <Eigen/Dense>

class MultiHypTracker;
class StochasticPhysicsTracker : public PhysicsTracker, public boost::enable_shared_from_this<StochasticPhysicsTracker> {
	vector<PR2Object::ManipCallbackPtr> m_callbacks; // keep track of the callbacks so that they can be removed on deallocation
public:
	typedef boost::shared_ptr<StochasticPhysicsTracker> Ptr;
	friend class MultiHypTracker;
	Environment::Ptr m_env;
	PR2Object::Ptr m_pr2;

	enum { UNIFORM, NORMAL, NONE };
	typedef int DistributionType;
	DistributionType m_dist; // UNIFORM between -m_param and +m_param. NORMAL with variance m_param
	float m_param; // unscaled
	int m_max_trials; //max attempts to moveByIK
	boost::variate_generator<boost::mt19937, boost::normal_distribution<> > m_generator;
	bool m_cb_enable; // do we want the callbacks to be added on initialization?
	bool m_cb_enabled; // has callbacks already being added?

	float sample();
	bool stochasticMoveByIK(PR2Object::ManipId manip_id);
	void stochasticChangeDOFValue(PR2Object::ManipId manip_id, string arm_joint_name);
	void enablePreGrabAndReleaseCallback();

	StochasticPhysicsTracker(TrackedObjectFeatureExtractor::Ptr object_features, FeatureExtractor::Ptr observation_features,
			VisibilityInterface::Ptr visibility_interface, DistributionType dist, float param, bool cb_enable=true);
	StochasticPhysicsTracker(TrackedObjectFeatureExtractor::Ptr object_features, StochasticPhysicsTracker::Ptr physics_tracker);
	// Splits this tracker into another one. This new tracker tracks a clone of the original tracked object, and this cloned object lives in a cloned environment.
	StochasticPhysicsTracker::Ptr split();
	~StochasticPhysicsTracker();
};

#endif /* STOCHASTIC_TRACKER_H_ */
