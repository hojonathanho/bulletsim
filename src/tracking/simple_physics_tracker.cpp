#include "simple_physics_tracker.h"
#include "sparse_utils.h"
#include "config_tracking.h"
#include "algorithm_common.h"
#include "plotting_tracking.h"
#include "utils/conversions.h"
#include <fstream>

using namespace Eigen;
using namespace std;

SimplePhysicsTracker::SimplePhysicsTracker(TrackedObject::Ptr obj, VisibilityInterface* visInt, Environment::Ptr env) :
  m_obj(obj),
  m_visInt(visInt),
  m_env(env),
  m_obsPlot(new PointCloudPlot(6)),
  m_estPlot(new PlotSpheres()),
  m_corrPlot(new PlotLines()),
  m_enableObsPlot(false),
  m_enableEstPlot(false),
  m_enableCorrPlot(false)
{
	m_env->add(m_obsPlot);
	m_env->add(m_estPlot);
	m_env->add(m_corrPlot);
	m_corrPlot->setDefaultColor(1,1,0,.3);
}

void SimplePhysicsTracker::updateInput(ColorCloudPtr obsPts) {
  m_obsPts = toBulletVectors(obsPts);
}

void SimplePhysicsTracker::doIteration() {
  VectorXf vis = m_visInt->checkNodeVisibility(m_obj);
  m_estPts = m_obj->getPoints();
  SparseMatrixf corr;
  m_stdev = (.01*METERS)*VectorXf::Ones(m_obj->m_nNodes);

  MatrixXf obsPtsEigen = toEigenMatrix(m_obsPts);

  // E STEP
  estimateCorrespondence(toEigenMatrix(m_estPts), m_stdev, vis, obsPtsEigen, TrackingConfig::outlierParam, corr);


  VectorXf inlierFrac = colSums(corr);

  if (m_enableObsPlot) plotObs(m_obsPts, inlierFrac, m_obsPlot);
  else m_obsPlot->clear();
  if (m_enableEstPlot) plotNodesAsSpheres(m_estPts, vis, m_stdev, m_estPlot);
  else m_estPlot->clear();
  if (m_enableCorrPlot) drawCorrLines(m_corrPlot, m_estPts, m_obsPts, corr);
  else m_corrPlot->clear();

  // M STEP
  m_obj->applyEvidence(corr, obsPtsEigen);
  m_env->step(.03,2,.015);


}
