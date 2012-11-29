#include "multihyp_tracker.h"

MultiHypTracker::MultiHypTracker(TrackedObjectFeatureExtractor::Ptr object_features, FeatureExtractor::Ptr observation_features,
		VisibilityInterface::Ptr visibility_interface, StochasticPhysicsTracker::Distribution dist, float param) :
		PhysicsTracker(object_features, observation_features, visibility_interface),
		m_dist(dist),
		m_param(param)
{
	StochasticPhysicsTracker::Ptr tracker(new StochasticPhysicsTracker(object_features, observation_features, visibility_interface, dist, param));
	m_trackers.push_back(tracker);
}

void MultiHypTracker::updateFeatures() {
	//if (grab)

	for (int i=0; i<m_trackers.size(); i++)
		m_trackers[i]->updateFeatures();
}

void MultiHypTracker::expectationStep() {
	for (int i=0; i<m_trackers.size(); i++)
		m_trackers[i]->expectationStep();
}

void MultiHypTracker::maximizationStep(bool apply_evidence) {
	for (int i=0; i<m_trackers.size(); i++)
		m_trackers[i]->maximizationStep(apply_evidence);
}



/*
void MultiHypTracker::doIteration() {
BOOST_FOREACH(SimplePhysicsTracker::Ptr& tracker, m_trackers)
  tracker->doIteration();
}

void MultiHypTracker::updateInput(ColorCloudPtr obsPts) {
  BOOST_FOREACH(SimplePhysicsTracker::Ptr& tracker, m_trackers) {
  	tracker->m_obsPts = m_obj->extractFeatures(obsPts);
  	tracker->m_obsCloud = obsPts;
  }
}

void MultiHypTracker::calcCosts() {
  for (int i=0; i < m_trackers.size(); ++i) {
    m_costs[i].push_back((m_trackers[i]->m_estPts - m_trackers[i]->m_obsPts).norm());
  }
}

VisibilityInterface::Ptr makeVisibilityWithNewWorld(VisibilityInterface::Ptr oldVis, btDynamicsWorld* newWorld) {
  shared_ptr<BulletRaycastVisibility> maybeOldVisBullet = boost::dynamic_pointer_cast<BulletRaycastVisibility>(oldVis);  
  if (maybeOldVisBullet) {
    return BulletRaycastVisibility::Ptr(new BulletRaycastVisibility(newWorld));
  }
  shared_ptr<BulletRaycastVisibility> maybeOldVisAll = boost::dynamic_pointer_cast<AllOcclusionsVisibility>(oldVis);
  if (maybeOldVisAll) {
    return AllOcclusionsVisibility::Ptr(new AllOcclusionsVisibility(newWorld, maybeOldVisAll->m_depth_image_visibility->m_transformer));
  }
  shared_ptr<BulletRaycastVisibility> maybeOldVisMulti = boost::dynamic_pointer_cast<MultiVisilibty>(oldVis);
  if (maybeOldVisMulti) {
    MultiVisibility::Ptr newVis(new MultiVisibility());
    BOOST_FOREACH(VisibilityInterface::Ptr& oldSubVis, maybeOldVisMulti->visibilities) {
      newVis->addVisibility(makeVisibilityWithNewWorld(oldSubVis, newWorld));
    }
  }
  return oldVis;
}

SimplePhysicsTracker::Ptr makeTrackerWithNewObject(SimplePhysicsTracker::Ptr oldTracker, TrackedObject::Ptr newTrackedObj, osg::ref_ptr<osg::Group> root) {
  BulletInstance::Ptr newBullet(new BulletInstance());
  OSGInstance::Ptr newOSG(new OSGInstance());
  root->addChild(newOSG->root.get());
  Fork::Ptr fork(new Fork(oldTracker->env, newBullet, newOSG));
  VisibilityInterface::Ptr newVis = makeVisibilityWithNewWorld(oldTracker->m_visInt, fork->env->bullet->dynamicsWorld);
  fork->env->remove(oldTracker->m_obj->m_sim);
  fork->env->add(newTrackedObj->m_sim);
  return SimplePhysicsTracker::Ptr(new SimplePhysicsTracker(newTrackedObj, newVis, fork->env));
}
*/
