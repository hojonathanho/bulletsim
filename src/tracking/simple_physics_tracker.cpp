#include "simple_physics_tracker.h"
#include "sparse_utils.h"
#include "config_tracking.h"
#include "algorithm_common.h"

using namespace Eigen;
using namespace std;

SimplePhysicsTracker::SimplePhysicsTracker(TrackedObject::Ptr obj, VisibilityInterface* visInt) :
  m_obj(obj),
  m_visInt(visInt)
{}

void SimplePhysicsTracker::updateInput(ColorCloudPtr obsPts) {
  m_obsPts = toEigenMatrix(obsPts);
}

void SimplePhysicsTracker::doIteration() {
  VectorXf vis = m_visInt->checkNodeVisibility(m_obj);
  m_estPts = m_obj->getPoints();
  SparseMatrixf corr;
  estimateCorrespondence(m_estPts, m_stdev, vis, m_obsPts, TrackingConfig::outlierParam, corr);
  m_obj->applyEvidence(corr, m_obsPts);
  m_env->step(.03,2,.015);
}
